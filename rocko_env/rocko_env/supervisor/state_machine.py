"""The supervisor's rules, as plain Python with no ROS and no I/O (docs/architecture/robot-supervisor.md).

The StateMachine holds the state (mode, enabled, phase, ...) and decides. It never starts a process or
calls a service. When a decision means "do something", it appends an *action* to a list; Supervisor.py
takes the actions (take_actions()) and does them. That split is why this file is tested on a laptop.

Actions are tuples; the first item says what to do:
    ("START_STACK", mode)   start that mode's launch file
    ("STOP_STACK",)         stop the running stack (SIGINT to its process group, then SIGKILL)
    ("ACTIVATE",)           activate the current mode's motion controllers
    ("DEACTIVATE",)         deactivate them (this is what stops the wheels)
    ("PUBLISH_REARM",)      publish /tipover_rearm (balance mode only, after ACTIVATE)
    ("KILL_STACK",)         SIGKILL the stack's process group now (no SIGINT first): controller_manager is
                            not answering, or a disable could not be confirmed, so nothing else can stop it

Time is passed in as `now_s` [s, any steady clock] so the tests control it.
"""
import math

from rocko_env.supervisor import modes
from rocko_env.teleop import gamepad

# PHASE_* and SOURCE_* strings are the wire values of rocko_interfaces/msg/RobotState: Dashboard.py and
# dashboard.html compare against them, so renaming one breaks the page.
PHASE_STARTING = "starting"
PHASE_READY = "ready"
PHASE_STOPPING = "stopping"
PHASE_FAULT = "fault"

SOURCE_GAMEPAD = "gamepad"
SOURCE_DASHBOARD = "dashboard"
SOURCE_SUPERVISOR = "supervisor"   # an automatic disable or fault
SOURCE_BOOT = "boot"

# How long controller_manager may go without answering list_controllers (asked once a second) before the
# supervisor stops trusting the stack. A missing answer is UNKNOWN, not "controller missing": in balance
# mode, treating a slow answer as a lost controller would disable the cascade and drop the robot.
CM_UNANSWERED_FAULT_S = 3.0   # [s] unanswered this long -> phase fault
CM_UNANSWERED_KILL_S = 5.0    # [s] still unanswered this long AFTER the fault -> kill the stack (tank, or disabled)
# After a disable, list_controllers must show every motion controller not active within this time, or the
# supervisor assumes the disable did not reach the motors and kills the stack.
# [s] Longer than controller_manager's own 5 s limit for one switch (SWITCH_REQUEST_TIMEOUT_S in Supervisor.py), so a
# slow but healthy deactivate is not mistaken for a stuck one.
DEACTIVATE_CONFIRM_S = 6.0


class StateMachine:
    def __init__(self, initial_mode=modes.DEFAULT_MODE):
        if initial_mode not in modes.MODES:
            raise ValueError(f"unknown mode {initial_mode!r}; choose one of {modes.mode_names()}")
        # Rule 1 (boot): the initial mode, disabled, its stack starting.
        self.mode = initial_mode
        self.enabled = False
        self.phase = PHASE_STARTING
        self.requested_mode = ""      # the mode being started during a change, else ""
        self.last_source = SOURCE_BOOT
        self.message = ""             # why the last request was refused, or what went wrong
        self._actions = [("START_STACK", initial_mode)]
        self._combos = gamepad.ComboDetector()
        self._last_joy_s = None       # [s] when /joy was last heard, None before the first message
        self._joy_heard_while_enabled = False
        # Verified disable: after a disable, retry the deactivate on every list_controllers reply until
        # a reply shows no motion controller active. motion_epoch counts enable/disable changes, so
        # a reply to a question asked BEFORE the latest change is not mistaken for proof.
        self.motion_epoch = 0
        self._disable_unconfirmed = False
        self._disable_since_s = None
        self._cm_noted = False        # the "controller_manager not answering" fault is already raised
        self._kill_sent = False       # a KILL_STACK for this stack is already queued

    # --- reading what to do ---------------------------------------------------------------------
    def take_actions(self):
        """The actions queued since the last call (and clear them)."""
        actions, self._actions = self._actions, []
        return actions

    # --- requests from a person (gamepad or dashboard) ------------------------------------------
    def request_enable(self, enabled, source, pitch_deg=None, rearm_max_pitch_deg=None):
        """Enable or disable. pitch_deg [deg] and rearm_max_pitch_deg [deg] matter only for enabling
        balance mode; None means "unknown". Returns (ok, message)."""
        if not enabled:
            # Rule 4: disabling is always allowed, in any phase.
            if self.enabled:
                self._disable(source, "")
                return True, "disabled"
            return True, "already disabled"

        if self.enabled:
            return True, "already enabled"
        # Rule 2: only a ready stack can be enabled.
        if self.phase != PHASE_READY:
            return self._refuse(f"cannot enable while the stack is {self.phase}")
        if self.mode == "balance":
            # Never assume upright: an unknown pitch is a refusal, not a pass.
            if pitch_deg is None or not math.isfinite(pitch_deg):
                return self._refuse("cannot enable balance: pitch is unknown")
            if rearm_max_pitch_deg is None or not math.isfinite(rearm_max_pitch_deg):
                return self._refuse("cannot enable balance: the allowed start pitch is unknown")
            if abs(pitch_deg) > rearm_max_pitch_deg:
                return self._refuse(f"cannot enable balance: pitch {pitch_deg:.1f} deg is outside "
                                    f"+-{rearm_max_pitch_deg:.1f} deg; stand the robot up")
        self.enabled = True
        self.motion_epoch += 1
        self._disable_unconfirmed = False   # an enable right after a disable replaces the wait for its proof
        self.last_source = source
        self.message = ""
        self._actions.append(("ACTIVATE",))
        if self.mode == "balance":
            # After the controllers are active, so a cutoff that tripped while disabled is cleared.
            self._actions.append(("PUBLISH_REARM",))
        return True, "enabled"

    def request_mode(self, mode, source):
        """Change mode. Returns (ok, message); the change itself finishes later (stack events)."""
        if mode not in modes.MODES:
            return self._refuse(f"unknown mode {mode!r}; choose one of {modes.mode_names()}")
        # Rule 3: only while disabled, and only when the stack is settled (ready or fault).
        if self.enabled:
            return self._refuse("disable the robot before changing mode")
        if self.phase not in (PHASE_READY, PHASE_FAULT):
            return self._refuse(f"cannot change mode while the stack is {self.phase}")
        if mode == self.mode and self.phase == PHASE_READY:
            return True, f"already in {mode} mode"
        # Stop the old stack, then (stack_stopped) start the new one.
        self.requested_mode = mode
        self.phase = PHASE_STOPPING
        self.last_source = source
        self.message = ""
        self._disable_unconfirmed = False   # the stack that was meant to deactivate is being stopped anyway
        self._actions.append(("STOP_STACK",))
        return True, f"changing to {mode} mode"

    # --- what the supervisor saw happen to the stack --------------------------------------------
    def stack_stopped(self):
        """The old stack is gone. Start the requested mode's stack."""
        if self.phase != PHASE_STOPPING or not self.requested_mode:
            return
        self.mode = self.requested_mode
        self.phase = PHASE_STARTING
        self._cm_noted = False
        self._kill_sent = False
        self._actions.append(("START_STACK", self.mode))

    def stack_ready(self):
        """Every expected controller is in its expected state and every expected node is up."""
        if self.phase == PHASE_STARTING:
            self.phase = PHASE_READY
            self.requested_mode = ""
            self.message = ""

    def stack_failed(self, reason, controllers_intact=False):
        """The stack did not start, died, lost something it needs, or would not stop.
        Rule 5: also disables, EXCEPT in a mode that falls when disabled (balance) while its motion
        controllers are still there. `controllers_intact` means exactly that: only a helper node (teleop,
        a sensor) is lost, every controller that holds the robot up is still listed. Then disabling
        would drop the robot, so it stays enabled, in `fault`, and a person disables it when it is held.
        Only a mode change (from fault) can leave `fault`."""
        if controllers_intact and self.enabled and modes.falls_when_disabled(self.mode):
            self.phase = PHASE_FAULT
            self.requested_mode = ""
            self.last_source = SOURCE_SUPERVISOR
            self.message = (reason + "; STILL ENABLED: disabling would make the robot fall. "
                            "Hold it, then disable")
            return
        was_enabled = self.enabled
        self.enabled = False
        self.phase = PHASE_FAULT
        self.requested_mode = ""
        self.last_source = SOURCE_SUPERVISOR
        self.message = reason
        self._joy_heard_while_enabled = False
        if was_enabled:
            self._queue_deactivate()

    # --- what the supervisor saw of controller_manager ------------------------------------------
    def controller_manager_unanswered(self, unanswered_s):
        """controller_manager has not answered list_controllers for `unanswered_s` [s] (counted from the
        last answer, or from the stack's start). Called every few tenths of a second."""
        if unanswered_s < CM_UNANSWERED_FAULT_S or self.phase not in (PHASE_READY, PHASE_FAULT):
            return
        if self.enabled and modes.falls_when_disabled(self.mode):
            # Balance: the cascade may still be holding the robot up, so never disable or kill on silence.
            if not self._cm_noted:
                self._cm_noted = True
                self.phase = PHASE_FAULT
                self.last_source = SOURCE_SUPERVISOR
                self.message = ("controller_manager is not answering; still enabled because disabling "
                                "would make the robot fall. Hold it, then disable")
            return
        if self.phase == PHASE_READY:
            self.stack_failed("controller_manager is not answering; "
                              "restart the stack: choose the mode again")
        if self.phase == PHASE_FAULT and unanswered_s >= CM_UNANSWERED_FAULT_S + CM_UNANSWERED_KILL_S:
            self._queue_kill("controller_manager stopped answering and the stack was killed; check the "
                             "motors: the motor plugin may have left a wheel's PWM pin switched on")

    def controller_states_reply(self, states, epoch):
        """list_controllers answered: `states` is {controller name: state}; `epoch` is motion_epoch when
        the question was SENT. While disabled, any motion controller still active means a deactivate did
        not take (or was never sent): ask again, and keep asking on every reply until none is active.

        Row 4.14 of docs/next-steps.md: a motion controller in any OTHER state ("unconfigured", "finalized")
        is where a controller lands when its on_deactivate returned an error, which AddFeedforwardController
        does when it could not write the zero that stops its wheel. Its interfaces are released with the
        last command still in the motor's slot, and asking again cannot help, so the stack is killed AT
        ONCE rather than after DEACTIVATE_CONFIRM_S. Not while the stack is starting: the --inactive
        spawner leaves each controller "unconfigured" between load and configure, which is normal there."""
        if self.enabled or self.phase in (PHASE_STOPPING, PHASE_STARTING) or epoch != self.motion_epoch:
            return
        motion = [n for n in modes.motion_controllers(self.mode) if n in states]
        failed = [n for n in motion if states[n] not in ("active", "inactive")]
        if failed:
            reason = ", ".join(f"{n} is {states[n]}" for n in failed)
            self.stack_failed(f"a motion controller failed to deactivate ({reason})")
            self._queue_kill(f"a motion controller failed to deactivate ({reason}): its stop command may not "
                             "have been written, so the stack was killed; check the motors: the motor plugin "
                             "may have left a wheel's PWM pin switched on")
            return
        if any(states[n] == "active" for n in motion):
            if not self._disable_unconfirmed:
                self._disable_unconfirmed = True
                self._disable_since_s = None   # the clock starts at the next tick()
            self._actions.append(("DEACTIVATE",))
        else:
            self._disable_unconfirmed = False

    def observe_stack(self, controller_states, missing_nodes, in_switch_grace):
        """One list_controllers answer, with the names of expected nodes that are not up.
        Starting: ready once every expected controller is in its expected state and every node is up.
        Ready, or fault while still enabled: a missing expected controller or node is a fault (so a
        controller lost AFTER a first fault is still acted on). A controller that is merely not in
        the answer's list is missing; no answer at all never reaches here (see controller_manager_unanswered)."""
        expected = self.expected_controllers()
        if self.phase == PHASE_STARTING:
            if all(controller_states.get(n) == state for n, state in expected) and not missing_nodes:
                self.stack_ready()
        elif (self.phase == PHASE_READY or (self.phase == PHASE_FAULT and self.enabled)) and not in_switch_grace:
            missing_controllers = [n for n, _ in expected if n not in controller_states]
            if missing_controllers or missing_nodes:
                self.stack_failed("missing: " + ", ".join(missing_controllers + missing_nodes),
                                  controllers_intact=not missing_controllers)

    def expected_controllers(self):
        """[(name, expected state)] for the current mode: always-active ones, then the motion ones."""
        motion_state = "active" if self.enabled else "inactive"
        return ([(n, "active") for n in modes.always_active(self.mode)]
                + [(n, motion_state) for n in modes.motion_controllers(self.mode)])

    def stack_gone(self):
        """The stack's processes are gone (exited, stopped or killed): nothing is left to confirm."""
        self._disable_unconfirmed = False

    # --- the game controller --------------------------------------------------------------------
    def joy_message(self, held, now_s, pitch_deg=None, rearm_max_pitch_deg=None):
        """One /joy message. `held` is gamepad.pressed_inputs(). Returns (ok, message) if it
        completed a combo, else None."""
        self._last_joy_s = now_s
        combo = self._combos.update(True, held)
        result = None
        if combo == "toggle_enable":
            result = self.request_enable(not self.enabled, SOURCE_GAMEPAD, pitch_deg, rearm_max_pitch_deg)
        elif combo == "toggle_mode":
            names = modes.mode_names()
            other = names[(names.index(self.mode) + 1) % len(names)]
            result = self.request_mode(other, SOURCE_GAMEPAD)
        # Rule 5: only a controller that was heard while enabled can be "lost" while enabled.
        if self.enabled:
            self._joy_heard_while_enabled = True
        return result

    def tick(self, now_s):
        """Call a few times a second. Rule 5: auto-disable if the controller, heard while enabled,
        has been silent longer than gamepad.JOY_TIMEOUT_S, in a mode that does not fall when disabled
        (tank). Returns True if it disabled. Also times the verified disable (DEACTIVATE_CONFIRM_S)."""
        self._check_disable_confirmed(now_s)
        if self._last_joy_s is None or now_s - self._last_joy_s <= gamepad.JOY_TIMEOUT_S:
            return False
        self._combos.update(False, set())   # a returning controller must be seen released first
        if self.enabled and self._joy_heard_while_enabled and modes.falls_when_disabled(self.mode):
            # Balance: disabling would drop the robot. BalanceTeleop.py already sends zero velocity on
            # the same silence (finding C-3), so the robot holds still, upright. Say so once; stay enabled.
            self._joy_heard_while_enabled = False
            self.message = "game controller silent: holding zero velocity, still enabled (disabling would make the robot fall)"
            return False
        if self.enabled and self._joy_heard_while_enabled:
            self._disable(SOURCE_SUPERVISOR, f"game controller silent for more than {gamepad.JOY_TIMEOUT_S} s")
            return True
        return False

    # --- what everyone reads --------------------------------------------------------------------
    def snapshot(self):
        return {
            "mode": self.mode, "enabled": self.enabled, "phase": self.phase,
            "requested_mode": self.requested_mode, "last_source": self.last_source,
            "message": self.message, "available_modes": modes.mode_names(),
        }

    # --- helpers --------------------------------------------------------------------------------
    def _disable(self, source, message):
        self.enabled = False
        self.last_source = source
        self.message = message
        self._joy_heard_while_enabled = False
        self._queue_deactivate()

    def _queue_deactivate(self):
        """Ask for the motion controllers to be deactivated, and keep asking until list_controllers shows it."""
        self.motion_epoch += 1
        self._disable_unconfirmed = True
        self._disable_since_s = None   # the clock starts at the next tick()
        self._actions.append(("DEACTIVATE",))

    def _queue_kill(self, message):
        if not self._kill_sent:
            self._kill_sent = True
            self._disable_unconfirmed = False
            self.message = message
            self._actions.append(("KILL_STACK",))

    def _check_disable_confirmed(self, now_s):
        if not self._disable_unconfirmed:
            return
        if self._disable_since_s is None:
            self._disable_since_s = now_s
        elif now_s - self._disable_since_s > DEACTIVATE_CONFIRM_S:
            self.stack_failed("could not confirm that the motion controllers are off")
            self._queue_kill("could not confirm that the motion controllers are off within "
                             f"{DEACTIVATE_CONFIRM_S:g} s, so the stack was killed; check the motors: "
                             "the motor plugin may have left a wheel's PWM pin switched on")

    def _refuse(self, message):
        # Rule 6: a refusal changes nothing but the message.
        self.message = message
        return False, message
