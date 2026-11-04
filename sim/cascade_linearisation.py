#!/usr/bin/env python3
"""Why does the simulated robot drift away and fall ~9-10 s after release? A linear answer.

Runs on the laptop with the standard library only (no NumPy, no ROS), from the repo root:

    python3 -I sim/cascade_linearisation.py

It linearises SimPlant about upright (docs/architecture/sim-plant.md §3.4), closes the two PID
loops exactly as rocko_sim_controllers.yaml wires them, and prints closed-loop poles:

  A. continuous time, idealised (no IMU delay, no sampling): the inner loop alone, the slow
     "creep" of sim-plant.md §2.4 equation (1), and the whole cascade for both encoder signs;
  B. discrete time, as the sim runs it: 100 Hz, zero-order hold on the wheel command, the 20 ms
     IMU delay on pitch (none on pitch_rate), the QuadEncoders.py replica (finding U-2), and the
     pid_controller update order (velocity loop first, its output used by the pitch loop in the
     same cycle);
  C. which way the slow pole moves as the velocity-loop gains are scaled up, for each encoder
     sign: the test of whether the outer loop is negative or positive feedback;
  D. margins: how far the inner loop's gain can be scaled, and how much IMU delay it tolerates.

Tuning modes (stage 5c-1, docs/journal/2026-11-03-stage5c1-sim-gain-tuning.md), all with the
scenarios' signs (encoder_sign = -1, an ASSUMPTION, proposal 0006) and the 20 ms IMU delay:

    python3 -I sim/cascade_linearisation.py --sweep [--top N]     # grid sweep, ~1-2 min
    python3 -I sim/cascade_linearisation.py --candidate 12,0.8,20,0,0.2    # or: --candidate yaml
    python3 -I sim/cascade_linearisation.py --hardware-gains       # this report, hardware YAML gains

--candidate prints the gain margins (up and down), the IMU delay range for which the loop is
stable, and a time simulation of scenarios U, I and V (section 6). A candidate is only a candidate
until verify_sim.sh scenarios confirms it in the nonlinear sim.

The gains and the pitch loop's D-term wiring are READ from the YAML files, so the numbers follow
the files. Plant parameters are copied from SimPlantParams in SimPlantModel.hpp (the defaults the
xacro also uses; sim/check_sim_config.py keeps those two equal).

EVERY PHYSICAL NUMBER HERE IS AN ESTIMATE, NOT A MEASUREMENT (sim-plant.md §4). The results are
statements about an estimated model. They say nothing about whether the real robot balances.

Sign conventions (sim-plant.md §1, §2.4):
  sim-forward   the direction a positive wheel command drives the robot (a definition);
  psi [rad]     true body lean, + toward sim-forward;   phi [rad] wheel angle, + rolling forward;
  pitch [deg]   exported = pitch_sign * degrees(psi)    (pitch_sign = -1 is the sim's default);
  velocity [m/s] exported = encoder_sign * chassis-relative speed, through the U-2 replica
                (encoder_sign = +1 is the sim's default).
"""

import cmath
import math
import os
import re
import sys

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CONFIG = os.path.join(REPO, "rocko_env", "rocko_env", "bringup", "config")
SIM_YAML = os.path.join(CONFIG, "rocko_sim_controllers.yaml")
HW_YAML = os.path.join(CONFIG, "rocko_controllers.yaml")

DEG = 180.0 / math.pi  # [deg per rad]
T = 0.01  # [s] control period: update_rate 100 in both YAML files

# ------------------------------------------------------------------------------------------------
# 1. Plant parameters: SimPlantParams defaults (SimPlantModel.hpp). ESTIMATES, NOT MEASURED.
# ------------------------------------------------------------------------------------------------
r = 0.06  # [m] wheel radius
N_b = 1.355  # [-] belt ratio
m_w = 0.3  # [kg] each wheel
m_b = 2.0  # [kg] body
l = 0.20  # [m] axle to body centre of mass
h_b, d_b = 0.40, 0.10  # [m] body height and depth, for its inertia only
g = 9.81  # [m/s^2]
tau_stall = 0.774725  # [N m] each motor, gearbox output
w_noload = 1150.0 / 60.0 * 2.0 * math.pi  # [rad/s] gearbox output, 120.428
I_stall, I_nl = 9.2, 0.25  # [A]
V_bat = V_rated = 12.0  # [V]
MAX_CODE = (1150 // 60) * 2.0 * math.pi  # [rad/s] 119.381, Motor12VoltQuadEncoder.hpp's integer division (U-3)


def derive_plant():
    """Everything that follows from the estimates above, as module-level names. A function only so
    that set_plant() (the robustness sweep, sim/parameter_sweep.py) can change m_b, l or tau_stall
    and recompute; it runs once, right here, with the defaults."""
    global I_w, I_b, a11, a12, a22, D, K_u, K_w, b, c
    I_w = 2 * 0.5 * m_w * r ** 2  # [kg m^2] both wheels, solid discs
    I_b = m_b * (h_b ** 2 + d_b ** 2) / 12.0  # [kg m^2] uniform box about its centre
    a11 = (2 * m_w + m_b) * r ** 2 + I_w
    a12 = m_b * r * l
    a22 = m_b * l ** 2 + I_b
    D = a11 * a22 - a12 ** 2
    K_u = 2 * N_b * tau_stall * V_bat / (V_rated * MAX_CODE)  # [N m per rad/s of command], both motors
    K_w = 2 * N_b ** 2 * tau_stall / w_noload  # [N m s/rad] back-EMF, both motors
    b = 2 * N_b ** 2 * (tau_stall / I_stall) * I_nl / w_noload  # [N m s/rad] viscous friction, both
    c = K_w + b  # [N m s/rad]


derive_plant()

# Encoder replica constants, copied from QuadEncoders.py (:39 alpha, :71 the assumed 0.01 s).
ENC_ALPHA = 0.05
ENC_ASSUMED_PERIOD = 0.01


def plant_derivative(psi, phi_dot, psi_dot, u):
    """Linearised SimPlant (sim-plant.md §3.4), model_write_quirks = false, both wheels at u [rad/s].

    Returns (d psi/dt, d phi_dot/dt, d psi_dot/dt). phi itself is left out: no force depends on it.
    """
    rel = phi_dot - psi_dot  # [rad/s] wheel speed relative to the body: what the motor shaft sees
    tau = K_u * u - c * rel  # [N m] motor torque on the wheel (minus the same on the body)
    # Solve [a11 a12; a12 a22] [phi_dd; psi_dd] = [tau; -tau + m_b g l psi]
    rhs1 = tau
    rhs2 = -tau + m_b * g * l * psi
    phi_dd = (a22 * rhs1 - a12 * rhs2) / D
    psi_dd = (-a12 * rhs1 + a11 * rhs2) / D
    return psi_dot, phi_dd, psi_dd


# ------------------------------------------------------------------------------------------------
# 2. Small linear-algebra kit (standard library only)
# ------------------------------------------------------------------------------------------------
def matrix_of(linear_map, n):
    """The n x n matrix of a linear function of an n-vector, built column by column."""
    cols = []
    for j in range(n):
        e = [0.0] * n
        e[j] = 1.0
        cols.append(linear_map(e))
    return [[cols[j][i] for j in range(n)] for i in range(n)]


def matmul(A, B):
    return [[sum(A[i][k] * B[k][j] for k in range(len(B))) for j in range(len(B[0]))] for i in range(len(A))]


def expm(A):
    """Matrix exponential by scaling and squaring with a Taylor series (fine for these small sizes)."""
    n = len(A)
    norm = max(sum(abs(x) for x in row) for row in A)
    s = max(0, int(math.ceil(math.log2(norm / 0.25))) if norm > 0.25 else 0)
    As = [[x / 2 ** s for x in row] for row in A]
    E = [[float(i == j) for j in range(n)] for i in range(n)]
    term = [row[:] for row in E]
    for k in range(1, 25):
        term = [[x / k for x in row] for row in matmul(term, As)]
        E = [[E[i][j] + term[i][j] for j in range(n)] for i in range(n)]
    for _ in range(s):
        E = matmul(E, E)
    return E


def det(M):
    """Determinant by Gaussian elimination with partial pivoting (works for complex entries)."""
    M = [row[:] for row in M]
    n = len(M)
    result = 1.0
    for k in range(n):
        p = max(range(k, n), key=lambda i: abs(M[i][k]))
        if M[p][k] == 0:
            return 0.0
        if p != k:
            M[k], M[p] = M[p], M[k]
            result = -result
        result *= M[k][k]
        for i in range(k + 1, n):
            f = M[i][k] / M[k][k]
            for j in range(k, n):
                M[i][j] -= f * M[k][j]
    return result


def eig(A):
    """Eigenvalues: Durand-Kerner roots of det(zI - A), each evaluated by a stable LU, not by
    forming the characteristic polynomial's coefficients. Checked against sim-plant.md §7.1 #4."""
    n = len(A)
    radius = 1.0 + max(sum(abs(x) for x in row) for row in A)
    z = [radius * cmath.exp(1j * (2 * math.pi * k / n + 0.4)) for k in range(n)]

    def p(x):
        return det([[(x if i == j else 0.0) - A[i][j] for j in range(n)] for i in range(n)])

    for _ in range(5000):
        biggest = 0.0
        for i in range(n):
            denom = 1.0
            for j in range(n):
                if j != i:
                    denom *= z[i] - z[j]
            step = p(z[i]) / denom
            z[i] -= step
            biggest = max(biggest, abs(step))
        if biggest < 1e-13 * radius:
            break
    return sorted(z, key=lambda x: (-x.real, -x.imag))


def fmt(z, digits=4):
    if abs(z.imag) < 1e-7 * max(1.0, abs(z)):
        return f"{z.real:+.{digits}f}"
    return f"{z.real:+.{digits}f}{z.imag:+.{digits}f}j"


# ------------------------------------------------------------------------------------------------
# 3. The controllers, read from the YAML (no PyYAML: one regex per gains line)
# ------------------------------------------------------------------------------------------------
def read_yaml(path):
    # Drop comments first: the hardware YAML keeps old gain sets in commented-out lines.
    lines = open(path, encoding="utf-8").read().split("\n")
    text = "\n".join(line.split("#")[0] for line in lines)

    def gains(key):
        m = re.search(re.escape(key) + r":\s*\{p:\s*([-\d.eE+]+),\s*i:\s*([-\d.eE+]+),\s*d:\s*([-\d.eE+]+)", text)
        return tuple(float(x) for x in m.groups())

    pitch = gains("left_add_feedforward_controller/left_wheel_joint")
    velocity = gains("pitch_pid_controller/left_wheel_joint")
    # Does pitch_pid_controller take its D term from pitch_rate (finding H-6) or from the angle?
    block = text.split("pitch_pid_controller:")[1].split("velocity_pid_controller:")[0]
    rate_d = '"pitch_rate"' in block.split("reference_and_state_interfaces:")[1].split("\n")[0]
    return {"pitch": pitch, "velocity": velocity, "gyro_d": rate_d}


# ------------------------------------------------------------------------------------------------
# 4. Continuous-time models (idealised: no delay, no sampling)
# ------------------------------------------------------------------------------------------------
TAU_ENC = -T / math.log((1 - ENC_ALPHA) ** 2)  # [s] the replica's per-cycle pole 0.9025 -> 97.5 ms


def continuous_inner(gains, pitch_sign):
    """State [psi, phi_dot, psi_dot]. Pitch loop alone, target pitch 0, D on the (undelayed) rate."""
    Kp, _, Kd = gains["pitch"]

    def f(x):
        psi, phi_dot, psi_dot = x
        u = Kp * (0.0 - pitch_sign * DEG * psi) + Kd * (0.0 - pitch_sign * DEG * psi_dot)
        return list(plant_derivative(psi, phi_dot, psi_dot, u))

    return matrix_of(f, 3)


def continuous_cascade(gains, pitch_sign, encoder_sign, velocity_scale=1.0):
    """State [psi, phi_dot, psi_dot, y, I]: y [m/s] the encoder replica as a first-order lag
    (steady gain 0.5, finding U-2), I [deg] the velocity loop's integral term. Reference 0."""
    Kp, _, Kd = gains["pitch"]
    Kpv, Kiv, Kdv = (velocity_scale * k for k in gains["velocity"])

    def f(x):
        psi, phi_dot, psi_dot, y, integ = x
        y_dot = (0.5 * encoder_sign * r * (phi_dot - psi_dot) - y) / TAU_ENC
        theta_ref = Kpv * (0.0 - y) + integ + Kdv * (0.0 - y_dot)  # [deg] target pitch
        u = Kp * (theta_ref - pitch_sign * DEG * psi) + Kd * (0.0 - pitch_sign * DEG * psi_dot)
        return list(plant_derivative(psi, phi_dot, psi_dot, u)) + [y_dot, Kiv * (0.0 - y)]

    return matrix_of(f, 5)


# ------------------------------------------------------------------------------------------------
# 5. Discrete-time model: one 10 ms control cycle exactly as the sim runs it
# ------------------------------------------------------------------------------------------------
# Plant over one period with the command held (zero-order hold): x+ = Phi x + Gam u, x = [phi, psi,
# phi_dot, psi_dot]. Built from the 5x5 exponential of [[A, B], [0, 0]].
def _plant_A_B():
    def f(x):
        u = x[4]
        psi_dot, phi_dd, psi_dd = plant_derivative(x[1], x[2], x[3], u)
        return [x[2], psi_dot, phi_dd, psi_dd, 0.0]

    return matrix_of(f, 5)


def _zoh(tau):
    """(Phi, Gam) of the plant over tau seconds with the command held (zero-order hold)."""
    E = expm([[x * tau for x in row] for row in _plant_A_B()])
    return [row[:4] for row in E[:4]], [row[4] for row in E[:4]]


PHI, GAM = _zoh(T)
_ZOH_CACHE = {T: (PHI, GAM)}


def zoh(tau):
    if tau not in _ZOH_CACHE:
        _ZOH_CACHE[tau] = _zoh(tau)
    return _ZOH_CACHE[tau]


def set_plant(body_mass_kg, com_height_m, motor_stall_torque_Nm):
    """Replace three estimates (SimPlant's parameters of the same names) and recompute every number
    derived from them, including the one-cycle plant matrices. For the +/-30 % robustness sweep
    (sim-plant.md §4.3, sim/parameter_sweep.py). As in SimPlant, the body's inertia follows its mass
    (uniform box) but not its centre-of-mass height, and the stall torque scales K_u, K_w and b
    together. Call set_plant(2.0, 0.20, 0.774725) to go back to the defaults."""
    global m_b, l, tau_stall, PHI, GAM
    m_b, l, tau_stall = body_mass_kg, com_height_m, motor_stall_torque_Nm
    derive_plant()
    _ZOH_CACHE.clear()
    PHI, GAM = zoh(T)


def linear_write(u):
    """The idealised write path (model_write_quirks = false): the duty clamp only, as rad/s."""
    return max(-MAX_CODE, min(MAX_CODE, u))


def quirks_write(u):
    """Motor12VoltQuadEncoder::write() as SimPlant copies it (model_write_quirks = true, the sim's
    default; SimPlantModel.hpp write_path_duty): deadband |u| < 0.05 -> 0; else an integer percent
    trunc(|u| / MAX_CODE * 100 + 2), clamped to 100. Returned as the rad/s the duty stands for, so the
    linear plant (K_u per rad/s) applies. Used only by the time simulation (--simulate), never in a
    pole: it is not linear."""
    if not math.isfinite(u) or abs(u) < 0.05:
        return 0.0
    percent = int(min(100.0, abs(u) / MAX_CODE * 100.0 + 2.0))
    return math.copysign(percent / 100.0 * MAX_CODE, u)


# Discrete state z; phi itself is replaced by delta = phi_k - phi_{k-1}, because only its change
# reaches anything (the encoder), so no eigenvalue is spent on where the robot is.
#   0 psi   1 phi_dot   2 psi_dot   3 delta    plant at read() of cycle k
#   4..     psi of earlier cycles, for the IMU delay (psi_{k-1}, psi_{k-2}, ...)
#   then    y (encoder filter after both service calls of the previous cycle),
#           I (velocity integral term), e_v_prev, e_theta_prev (for the two D terms)
#   then    only for a delay that is not a whole number of cycles (delay_ms % 10 != 0): the pitch
#           f ms before each of the last j+1 reads, m_k = psi(t_k - f), where delay = j*10 ms + f.
def discrete_step(gains, pitch_sign, encoder_sign, delay_ms, *, outer=True, gyro_d=None,
                  velocity_scale=1.0, command_scale=1.0):
    """(n, step): the state size and one 10 ms control cycle, step(z, v_ref, u_ff, write, log) -> z+.

    v_ref [m/s] is the velocity loop's reference, u_ff [rad/s] the feedforward AddFeedforwardController
    adds, write(u) the write path (None: none at all, so the matrix of step is the linear model), and
    log, if a dict, receives what the controllers saw this cycle (pitch, v_meas, u)."""
    Kp, _, Kd = gains["pitch"]
    Kpv, Kiv, Kdv = (velocity_scale * k for k in gains["velocity"])
    if not outer:
        Kpv = Kiv = Kdv = 0.0
    if gyro_d is None:
        gyro_d = gains["gyro_d"]
    j, f_ms = divmod(int(delay_ms), 10)  # delay = j whole cycles + f_ms
    nd = max(j, 1) if f_ms == 0 else 1  # always keep psi_{k-1}: the encoder needs it
    nm = j + 1 if f_ms else 0  # length of the m_k line
    n = 4 + nd + 4 + nm
    iy, iI, iev, iet, im = 4 + nd, 5 + nd, 6 + nd, 7 + nd, 8 + nd
    PHI_f, GAM_f = zoh(T - f_ms / 1000.0) if f_ms else (None, None)

    def step(z, v_ref=0.0, u_ff=0.0, write=None, log=None):
        psi, phi_dot, psi_dot, delta = z[0:4]
        past = z[4:4 + nd]  # past[0] = psi_{k-1}
        # --- read(): what the controllers see ---
        if f_ms:
            psi_seen = z[im + j]  # m_{k-j} = psi(t_k - j T - f)
        else:
            psi_seen = psi if j == 0 else past[j - 1]
        pitch = pitch_sign * DEG * psi_seen  # [deg], IMU delayed
        pitch_rate = pitch_sign * DEG * psi_dot  # [deg/s], gyro, not delayed (sim-plant.md §3.7)
        dp = encoder_sign * r * (delta - (psi - past[0]))  # [m] chassis-relative travel this cycle
        s1 = dp / ENC_ASSUMED_PERIOD
        y_left = (1 - ENC_ALPHA) * z[iy] + ENC_ALPHA * s1  # first service call of the cycle
        y_right = (1 - ENC_ALPHA) * y_left  # second call: no motion since the first
        v_meas = 0.5 * (y_left + y_right)  # the cascade is linear: average of the two wheels' loops
        # --- update(): velocity_pid_controller first (upstream), then pitch_pid_controller ---
        e_v = v_ref - v_meas
        integ = z[iI] + Kiv * T * e_v
        theta_ref = Kpv * e_v + integ + Kdv * (e_v - z[iev]) / T  # [deg]
        e_theta = theta_ref - pitch
        if gyro_d:
            d_term = Kd * (0.0 - pitch_rate)  # rate reference 0.0, set once at activation
        else:
            d_term = Kd * (e_theta - z[iet]) / T  # derivative of the error, delayed angle
        u = Kp * e_theta + d_term + u_ff  # [rad/s] AddFeedforwardController's output
        # command_scale stands for a motor gain error. No write path: the linear model (matrix_of
        # feeds unit states, far past any clamp, so a clamp here would corrupt the matrix).
        u_applied = command_scale * (u if write is None else write(u))
        if log is not None:
            log.update(pitch=pitch, pitch_rate=pitch_rate, v_meas=v_meas, u=u)
        # --- write() then the physics over the next 10 ms ---
        x = [0.0, psi, phi_dot, psi_dot]
        x_next = [sum(PHI[i][k] * x[k] for k in range(4)) + GAM[i] * u_applied for i in range(4)]
        out = [0.0] * n
        out[0], out[1], out[2], out[3] = x_next[1], x_next[2], x_next[3], x_next[0]
        out[4] = psi
        for k in range(1, nd):
            out[4 + k] = past[k - 1]
        out[iy], out[iI], out[iev], out[iet] = y_right, integ, e_v, e_theta
        if f_ms:  # m_{k+1} = psi(t_{k+1} - f): the plant f ms before the next read
            out[im] = sum(PHI_f[1][k] * x[k] for k in range(4)) + GAM_f[1] * u_applied
            for k in range(1, nm):
                out[im + k] = z[im + k - 1]
        return out

    return n, step


def discrete_cascade(gains, pitch_sign, encoder_sign, delay_cycles, *, delay_ms=None, **kwargs):
    """The closed-loop matrix of one control cycle (the linear model). delay_ms, if given, replaces
    delay_cycles and may be any whole number of milliseconds, as SimPlant's imu_delay_ms."""
    n, step = discrete_step(gains, pitch_sign, encoder_sign,
                            10 * delay_cycles if delay_ms is None else delay_ms, **kwargs)
    return matrix_of(step, n)


def discrete_eig(*args, **kwargs):
    """Eigenvalues of discrete_cascade(...), without the one that means nothing: with the velocity
    loop's i = 0 (or the loop off) its integral state just holds 0.0 forever, an eigenvalue of
    exactly 1 that no input reaches and nothing reads."""
    return [z for z in eig(discrete_cascade(*args, **kwargs)) if abs(z - 1) > 1e-9]


def spectral(z_list):
    """Largest |z|, and the eigenvalue that has it, as a continuous rate s = ln(z)/T."""
    zmax = max(z_list, key=abs)
    return abs(zmax), cmath.log(zmax) / T


def as_rate(z):
    return cmath.log(z) / T  # [1/s]


def slow_and_fast(z_list):
    """Split the modes in two. Slow: |Im s| < 5 rad/s and Re s > -5 1/s (the creep or drift);
    returns the one growing fastest. Fast: every other mode; returns the largest |z|, which with
    these gains is the ~3.5 Hz pitch oscillation pair (B2 lists every mode of the default case)."""
    rated = [(z, as_rate(z)) for z in z_list if abs(z) > 1e-9]
    slow = [z for z, s in rated if abs(s.imag) < 5 and s.real > -5]
    fast = [z for z in z_list if z not in slow]
    return max(slow, key=lambda z: as_rate(z).real), max(fast, key=abs)


# ------------------------------------------------------------------------------------------------
# 6. The report
# ------------------------------------------------------------------------------------------------
# 6. Tuning tools (stage 5c-1): margins, a time simulation of scenarios U, I, V, and a sweep.
#    All with pitch_sign = -1 and encoder_sign = -1 (the scenarios' labelled ASSUMPTION, proposal
#    0006) and the 20 ms IMU delay, unless stated. A candidate found here is confirmed in the
#    nonlinear sim (verify_sim.sh scenarios) before anyone believes it.
# ------------------------------------------------------------------------------------------------
SCEN_PITCH_SIGN, SCEN_ENCODER_SIGN, SCEN_DELAY_MS = -1, -1, 20


def radius(gains, delay_ms=SCEN_DELAY_MS, **kwargs):
    """Largest |z| of the closed loop (< 1 means stable), scenario signs."""
    return max(abs(z) for z in discrete_eig(gains, SCEN_PITCH_SIGN, SCEN_ENCODER_SIGN, 0,
                                            delay_ms=delay_ms, **kwargs))


def margins(gains):
    """Gain margins and the IMU delay range for which the model is stable.

    Gain margin: the factor on everything the motors deliver (command_scale: Kp, Kd and the velocity
    loop together, as a motor-gain error or a battery-voltage change would do) at which the loop goes
    unstable, upward and downward. A balancing robot is open-loop unstable, so too LITTLE gain also
    fails: both sides are reported. Delay: whole milliseconds of IMU delay (SimPlant's imu_delay_ms
    resolution), walking out from 20 ms in both directions to the first unstable delay, within 0 to
    80 ms; the result is the stable range that contains 20 ms.
    """
    if radius(gains) >= 1.0:
        return None

    def crossing(lo, hi):  # lo stable, hi unstable: bisect on the log of the factor
        for _ in range(30):
            mid = math.sqrt(lo * hi)
            lo, hi = (mid, hi) if radius(gains, command_scale=mid) < 1.0 else (lo, mid)
        return math.sqrt(lo * hi)

    up = crossing(1.0, 20.0) if radius(gains, command_scale=20.0) >= 1.0 else math.inf
    down = crossing(1.0, 0.01) if radius(gains, command_scale=0.01) >= 1.0 else 0.0
    hi = SCEN_DELAY_MS  # walk out from 20 ms one millisecond at a time, to the first unstable delay
    while hi + 1 <= 80 and radius(gains, delay_ms=hi + 1) < 1.0:
        hi += 1
    lo = SCEN_DELAY_MS
    while lo - 1 >= 0 and radius(gains, delay_ms=lo - 1) < 1.0:
        lo -= 1
    return {"gain_up": up, "gain_down": down, "delay_lo_ms": lo, "delay_hi_ms": hi}


def db(x):
    return 20 * math.log10(x) if 0 < x < math.inf else (math.inf if x == math.inf else -math.inf)


def simulate(gains, scenario, *, write=quirks_write, delay_ms=SCEN_DELAY_MS):
    """Scenario U, I or V on the discrete model, with the write path (default: SimPlant's quirks)
    and the clamp, judged with the same thresholds as verify_sim.sh's scenario block (copied below
    by name; verify_sim.sh is the one place they are decided). Linear plant: no large-angle
    physics, no open-loop gap after release, no DDS or executor jitter. A screen, not a verdict."""
    n, step = discrete_step(gains, SCEN_PITCH_SIGN, SCEN_ENCODER_SIGN, delay_ms)
    j, f_ms = divmod(delay_ms, 10)
    z = [0.0] * n
    psi0 = math.radians(2.0) * SCEN_PITCH_SIGN if scenario == "U" else 0.0  # U_START_DEG
    # The ring buffer starts full of the initial lean (sim-plant.md §3.6, on_activate).
    nd = max(j, 1) if f_ms == 0 else 1
    z[0] = psi0
    for k in range(4, 4 + nd):
        z[k] = psi0
    for k in range(8 + nd, n):
        z[k] = psi0
    seconds = {"U": 10.0, "I": 8.0, "V": 9.0}[scenario]  # U_RECORD_S, I_RECORD_S, V_BEFORE_S + V_STEP_S
    rows = []  # (t [s after release], pitch [deg], v_meas [m/s], u [rad/s])
    for k in range(int(round(seconds / T)) + 1):
        t = k * T
        if scenario == "I" and k == int(round(2.0 / T)):  # I_IMPULSE_AT_S, I_IMPULSE_DEG_S
            z[2] += math.radians(100.0) * SCEN_PITCH_SIGN
        stepping = scenario == "V" and t >= 1.0  # V_BEFORE_S
        v_ref = 0.1 if stepping else 0.0  # V_STEP_M_S
        u_ff = v_ref / 0.06  # DiffDriveController.py: wheel_radius_meters = 0.06
        log = {}
        z = step(z, v_ref, u_ff, write, log)
        rows.append((t, log["pitch"], log["v_meas"], log["u"]))
    TIP = 35.0
    fell = next((t for t, p, _, _ in rows if abs(p) >= TIP), None)
    out = {"fell_at_s": fell, "max_abs_u": max(abs(u) for *_, u in rows),
           "saturated_cycles": sum(1 for *_, u in rows if abs(u) >= MAX_CODE)}
    if scenario == "U":
        late = [row for row in rows if row[0] >= 5.0]  # U_SETTLE_FROM_S
        out["late_pitch"] = max(abs(p) for _, p, _, _ in late)
        out["late_speed"] = max(abs(v) for _, _, v, _ in late)
        out["pass"] = fell is None and out["late_pitch"] <= 2.0 and out["late_speed"] <= 0.05
    elif scenario == "I":
        after = [row for row in rows if row[0] >= 2.0]
        last_out = max((t for t, p, _, _ in after if abs(p) > 2.0), default=None)
        out["peak_pitch"] = max(abs(p) for _, p, _, _ in after)
        out["settle_s"] = (last_out - 2.0) if last_out is not None else 0.0
        out["pass"] = fell is None and out["settle_s"] <= 3.0  # I_SETTLED_DEG, I_SETTLE_S
    else:
        after = [row for row in rows if row[0] >= 1.0]
        rise = next((t for t, _, v, _ in after if v >= 0.09), None)
        out["rise_s"] = rise - 1.0 if rise is not None else None
        out["overshoot_pct"] = 100.0 * (max(v for _, _, v, _ in after) - 0.1) / 0.1
        out["peak_pitch"] = max(abs(p) for _, p, _, _ in rows)
        out["final_v"] = rows[-1][2]
        out["pass"] = (fell is None and out["rise_s"] is not None and out["rise_s"] <= 3.0
                       and out["overshoot_pct"] <= 30 and out["peak_pitch"] < 10.0)
    return out


def describe(name, res):
    if res["fell_at_s"] is not None:
        return f"{name} FELL at {res['fell_at_s']:.2f} s"
    if name == "U":
        detail = f"|pitch| {res['late_pitch']:.3f} deg, |v| {res['late_speed']:.4f} m/s (5-10 s)"
    elif name == "I":
        detail = f"peak {res['peak_pitch']:.2f} deg, settled {res['settle_s']:.2f} s after the push"
    else:
        rise = "never" if res["rise_s"] is None else f"{res['rise_s']:.2f} s"
        detail = (f"rise {rise}, overshoot {res['overshoot_pct']:.1f} %, peak |pitch| "
                  f"{res['peak_pitch']:.2f} deg, v at 9 s {res['final_v']:.4f}")
    return f"{name} {'pass' if res['pass'] else 'FAIL'}: {detail}; peak |u| {res['max_abs_u']:.1f} rad/s"


def gains_with(base, numbers):
    """base gains with (pitch p, pitch d, velocity p, velocity i, velocity d) replaced; pitch i = 0."""
    kp, kd, kpv, kiv, kdv = numbers
    return dict(base, pitch=(kp, 0.0, kd), velocity=(kpv, kiv, kdv))


def report_candidate(gains, label):
    print(f"\n{label}: pitch p={gains['pitch'][0]} d={gains['pitch'][2]}; velocity p={gains['velocity'][0]} "
          f"i={gains['velocity'][1]} d={gains['velocity'][2]}  (encoder_sign -1 ASSUMED, 20 ms)")
    rad = radius(gains)
    print(f"   largest |z| = {rad:.5f} per 10 ms cycle (slowest decay {-math.log(rad) / T:.3f} 1/s)"
          if rad < 1 else f"   largest |z| = {rad:.5f}: UNSTABLE in the linear model")
    m = margins(gains)
    if m:
        print(f"   gain margin: x{m['gain_up']:.3f} up ({db(m['gain_up']):+.2f} dB), "
              f"x{m['gain_down']:.3f} down ({db(m['gain_down']):+.2f} dB)")
        print(f"   IMU delay: stable from {m['delay_lo_ms']} to {m['delay_hi_ms']} ms (walked out from 20 ms, within 0-80 ms); "
              f"delay margin {m['delay_hi_ms'] - SCEN_DELAY_MS} ms over the modelled 20 ms")
    for name in ("U", "I", "V"):
        print("   time sim, SimPlant write quirks: " + describe(name, simulate(gains, name)))
    for name in ("U", "I", "V"):
        print("   time sim, linear write (clamp only): " + describe(name, simulate(gains, name, write=linear_write)))
    print("   time sim, 30 ms IMU delay (held out): "
          + "; ".join(describe(nm, simulate(gains, nm, delay_ms=30)).split(";")[0] for nm in ("U", "I", "V")))
    return m


SWEEP_GRID = {  # pitch p, pitch d, velocity p, velocity i, velocity d
    "kp": [6.0, 8.0, 10.0, 12.0, 15.0],
    "kd": [0.2, 0.3, 0.4, 0.5, 0.6],
    "kpv": [4.0, 6.0, 8.0, 10.0, 12.0, 16.0],
    "kiv": [0.0, 1.0, 2.0, 5.0],
    "kdv": [0.0, 0.1, 0.2],
}


def sweep(base, top):
    """Every grid point's largest |z| (scenario signs, 20 ms), then the `top` most damped points
    screened in the time simulation and given margins. Prints; changes nothing."""
    g = SWEEP_GRID
    points = [(kp, kd, kpv, kiv, kdv) for kp in g["kp"] for kd in g["kd"] for kpv in g["kpv"]
              for kiv in g["kiv"] for kdv in g["kdv"]]
    scored = []
    for numbers in points:
        scored.append((radius(gains_with(base, numbers)), numbers))
    stable = [s for s in scored if s[0] < 1.0]
    print(f"SWEEP: {len(points)} grid points, {len(stable)} stable in the linear model "
          f"(encoder_sign -1 ASSUMED, 20 ms IMU delay)")
    for name, values in g.items():
        counts = [sum(1 for _, nb in stable if nb[list(g).index(name)] == v) for v in values]
        print(f"   stable points by {name}: " + ", ".join(f"{v}: {c}" for v, c in zip(values, counts)))
    stable.sort()
    print(f"\nThe {top} most damped (smallest largest |z|), time-simulated with SimPlant's write quirks:")
    print("   kp    kd    kpv   kiv   kdv  | |z|     | U late |pitch|, |v| | I settle | V rise, overshoot, |pitch| | 30 ms")
    for rad, numbers in stable[:top]:
        gains = gains_with(base, numbers)
        u, i, v = (simulate(gains, s) for s in ("U", "I", "V"))
        held = all(simulate(gains, s, delay_ms=30)["pass"] for s in ("U", "I", "V"))
        cells = [
            "fell" if u["fell_at_s"] is not None else f"{u['late_pitch']:.2f}, {u['late_speed']:.3f}",
            "fell" if i["fell_at_s"] is not None else f"{i['settle_s']:.2f}",
            "fell" if v["fell_at_s"] is not None else
            f"{'-' if v['rise_s'] is None else round(v['rise_s'], 2)}, {v['overshoot_pct']:.0f}%, {v['peak_pitch']:.1f}",
        ]
        print("   " + " ".join(f"{x:<5}" for x in numbers) + f" | {rad:.4f} | {cells[0]:<20} | {cells[1]:<8} | "
              f"{cells[2]:<25} | {'all pass' if held else 'fails'}"
              + ("   <- U, I, V all pass" if u["pass"] and i["pass"] and v["pass"] else ""))


def main():
    import argparse
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--hardware-gains", action="store_true",
                        help="the default report with rocko_controllers.yaml's gains (and the sim's H-6 "
                             "wiring), as the stage 5 diagnosis was run before the sim gains were tuned")
    parser.add_argument("--candidate", metavar="KP,KD,KPV,KIV,KDV",
                        help="margins and time simulation of one gain set (pitch p, pitch d, velocity p, "
                             "i, d; pitch i = 0); 'yaml' = the sim YAML's gains")
    parser.add_argument("--sweep", action="store_true", help="grid sweep (about 1-2 minutes)")
    parser.add_argument("--top", type=int, default=15, help="--sweep: how many to time-simulate")
    args = parser.parse_args()
    if args.candidate or args.sweep:
        print("ESTIMATED MODEL (sim-plant.md §4), encoder_sign = -1 ASSUMED (proposal 0006): numbers "
              "about a model, not about the robot. Not a gain recommendation for the robot.\n")
        base = read_yaml(SIM_YAML)
        if args.sweep:
            sweep(base, args.top)
        if args.candidate:
            if args.candidate == "yaml":
                report_candidate(base, f"YAML ({os.path.relpath(SIM_YAML, REPO)})")
            else:
                numbers = tuple(float(x) for x in args.candidate.split(","))
                report_candidate(gains_with(base, numbers), "candidate")
        return 0
    return report(args.hardware_gains)


def report(hardware_gains):
    sim = read_yaml(SIM_YAML)
    hw = read_yaml(HW_YAML)
    if hardware_gains:
        sim = dict(sim, pitch=hw["pitch"], velocity=hw["velocity"])
    print("ESTIMATED MODEL (sim-plant.md §4): numbers about a model, not about the robot.\n")
    print(f"Gains read from {os.path.relpath(HW_YAML if hardware_gains else SIM_YAML, REPO)}"
          + (" (--hardware-gains; D-term wiring from the sim YAML):" if hardware_gains else ":"))
    print(f"  pitch_pid    p={sim['pitch'][0]} i={sim['pitch'][1]} d={sim['pitch'][2]}  "
          f"D from {'pitch_rate (gyro, H-6)' if sim['gyro_d'] else 'the angle'}")
    print(f"  velocity_pid p={sim['velocity'][0]} i={sim['velocity'][1]} d={sim['velocity'][2]}")
    print(f"  hardware YAML gains equal: {sim['pitch'] == hw['pitch'] and sim['velocity'] == hw['velocity']}; "
          f"hardware D from {'pitch_rate' if hw['gyro_d'] else 'the angle'}\n")

    # Self-checks against sim-plant.md §4 and §7.1 (catch a typo in this file).
    assert abs(a11 - 0.01044) < 1e-6 and abs(a12 - 0.024) < 1e-9 and abs(a22 - 0.108333) < 1e-6
    assert abs(K_u - 0.017587) < 1e-5 and abs(c - 0.024265) < 1e-5

    def braked(x):
        return list(plant_derivative(x[0], x[1], x[2], 0.0))

    braked_poles = eig(matrix_of(braked, 3))
    expect = [6.9306, -2.0304, -12.1916]
    assert all(abs(z.real - e) < 1e-3 and abs(z.imag) < 1e-6 for z, e in zip(braked_poles, expect)), braked_poles
    print("Check: braked open-loop poles " + ", ".join(fmt(z) for z in braked_poles)
          + "  (sim-plant.md §7.1 #4: +6.9306, -2.0304, -12.1916, plus 0 for phi)\n")

    print("=" * 100)
    print("A. CONTINUOUS TIME, idealised (no IMU delay, no 100 Hz sampling), pitch_sign = -1")
    print("=" * 100)
    inner = eig(continuous_inner(sim, -1))
    print("A1 inner loop alone (velocity loop off), poles [1/s]: " + ", ".join(fmt(z) for z in inner))
    print("   the complex pair is the pitch oscillation; the small positive real pole is the creep of eq. (1)")
    inner_wrong = eig(continuous_inner(sim, +1))
    print("   same with pitch_sign = +1 (the authors' apparent intent):  " + ", ".join(fmt(z) for z in inner_wrong))

    Kp, _, Kd = sim["pitch"]
    Kpv, Kiv, Kdv = sim["velocity"]
    Kp_rad = Kp * DEG
    m_eff = K_u * Kp_rad * (a11 + a12) / (m_b * g * l) - a11
    k = K_u * Kp * 0.5 * r
    print(f"\nA2 slow-time-scale reduction, sim-plant.md §2.4 eq. (1): m_eff = {m_eff:.4f} kg m^2, "
          f"c = {c:.6f} N m s/rad, creep rate c/m_eff = {c / m_eff:.4f} 1/s")
    print(f"   outer loop: m_eff s^2 - (c + s_v k Kpv) s - s_v k Kiv = 0, k = K_u Kp 0.5 r = {k:.6f}")
    for s_v in (+1, -1):
        a2, a1, a0 = m_eff - s_v * k * Kdv, -(c + s_v * k * Kpv), -s_v * k * Kiv
        disc = cmath.sqrt(a1 * a1 - 4 * a2 * a0)
        roots = sorted([(-a1 + disc) / (2 * a2), (-a1 - disc) / (2 * a2)], key=lambda z: -z.real)
        verdict = ("constant term < 0: a real positive root for ANY Kiv > 0 -> POSITIVE feedback at DC"
                   if a0 < 0 else "constant term > 0: negative feedback at DC")
        print(f"   encoder_sign {s_v:+d}: roots " + ", ".join(fmt(z) for z in roots) + f"   [{verdict}]")
    print(f"   with encoder_sign = -1 it is stable only if k*Kpv > c, i.e. Kpv > {c / k:.2f} deg/(m/s); "
          f"the YAML has {Kpv}")

    print("\nA3 whole cascade, continuous (encoder replica as a 97.5 ms lag of gain 0.5), poles [1/s]:")
    for s_v in (+1, -1):
        Acl = continuous_cascade(sim, -1, s_v)
        poles = eig(Acl)
        p0 = det([[-x for x in row] for row in Acl])  # = characteristic polynomial at s = 0
        print(f"   encoder_sign {s_v:+d}: " + ", ".join(fmt(z) for z in poles)
              + f"   char. poly at s=0: {p0:+.3e} ({'<0: odd number of real RHP roots' if p0 < 0 else '>0'})")

    print("\n" + "=" * 100)
    print("B. DISCRETE TIME, as the sim runs: 100 Hz, ZOH, IMU delay on pitch only, U-2 encoder replica")
    print("   z per 10 ms cycle (|z| > 1 grows); s = ln(z)/0.01 [1/s]. pitch_sign = -1 throughout.")
    print("   'slow' = the creep/drift mode, 'fast' = the ~3.5 Hz pitch oscillation pair")
    print("=" * 100)
    for label, gyro in (("D from pitch_rate (sim YAML, H-6)", True), ("D from the angle (hardware YAML)", False)):
        print(f"\n{label}")
        print("   delay  | inner alone: slow s, fast |z| | cascade, encoder_sign +1        "
              "| cascade, encoder_sign -1")
        print("          |                              | slow s, fast |z|               | slow s, fast |z|")
        for delay in (0, 1, 2, 3):
            row = f"   {delay * 10:3d} ms "
            for outer, s_v in ((False, +1), (True, +1), (True, -1)):
                slow, fast = slow_and_fast(discrete_eig(sim, -1, s_v, delay, outer=outer, gyro_d=gyro))
                row += f"| {fmt(as_rate(slow), 3)}, {abs(fast):.5f}".ljust(33 if outer else 31)
            print(row)

    print("\nB2 the sim's default case in full (H-6, 20 ms, encoder_sign +1), every eigenvalue:")
    for z in discrete_eig(sim, -1, +1, 2):
        s = fmt(as_rate(z), 3) if abs(z) > 1e-9 else "-inf"
        print(f"   z = {fmt(z, 5):>22}   |z| = {abs(z):.5f}   s = {s}")

    print("\n" + "=" * 100)
    print("C. LOOP SIGN: scale all three velocity gains by m and watch the modes (discrete, H-6, 20 ms)")
    print("   negative feedback: more gain pulls the slow mode left; positive feedback: pushes it right")
    print("=" * 100)
    print("   m     | encoder_sign +1: slow s, fast |z|   | encoder_sign -1: slow s, fast |z|")
    for m in (0.0, 0.25, 0.5, 1.0, 2.0, 4.0, 8.0):
        row = f"   {m:<5} "
        for s_v in (+1, -1):
            slow, fast = slow_and_fast(discrete_eig(sim, -1, s_v, 2, velocity_scale=m))
            row += f"| {fmt(as_rate(slow), 3)}, {abs(fast):.5f}".ljust(38)
        print(row)

    print("\nC2 which velocity-loop term moves the fast pitch pair (discrete, H-6, 20 ms; YAML pitch gains):")
    for name, vel in (("P only", (Kpv, 0.0, 0.0)), ("I only", (0.0, Kiv, 0.0)), ("D only", (0.0, 0.0, Kdv)),
                      ("P+I+D (YAML)", (Kpv, Kiv, Kdv))):
        g2 = dict(sim, velocity=vel)
        row = f"   {name:<13}"
        for s_v in (+1, -1):
            slow, fast = slow_and_fast(discrete_eig(g2, -1, s_v, 2))
            row += f"| enc {s_v:+d}: slow s {fmt(as_rate(slow), 3)}, fast |z| {abs(fast):.5f}  "
        print(row)

    print("\nC3 is there ANY stable gain set nearby? (discrete, H-6, 20 ms; velocity i, d at YAML values;")
    print("   a map of where the model is stable, NOT a gain recommendation; '#' stable, '.' unstable)")
    kpvs = [1, 2, 3, 4, 5, 6, 8, 10, 12, 16, 20]
    kds = [0.1, 0.15, 0.2, 0.25, 0.3, 0.4]
    for s_v in (+1, -1):
        print(f"   encoder_sign {s_v:+d}   Kpv = " + " ".join(f"{x:>3}" for x in kpvs))
        for kd in kds:
            marks = []
            for kpv in kpvs:
                g2 = dict(sim, pitch=(Kp, 0.0, kd), velocity=(float(kpv), Kiv, Kdv))
                marks.append("  #" if spectral(discrete_eig(g2, -1, s_v, 2))[0] < 1.0 else "  .")
            print(f"     pitch d = {kd:<4}            " + " ".join(marks))

    print("\n" + "=" * 100)
    print("D. INNER-LOOP MARGINS (discrete, inner loop alone, H-6, 20 ms delay)")
    print("=" * 100)
    print("   multiplier on the pitch loop's output (Kp and Kd together): slow creep s, fast |z|")
    for mult in (0.25, 0.5, 0.75, 1.0, 1.5, 2.0, 3.0):
        slow, fast = slow_and_fast(discrete_eig(sim, -1, +1, 2, outer=False, command_scale=mult))
        print(f"   x{mult:<5} slow s = {fmt(as_rate(slow), 3):>7} 1/s, fast |z| = {abs(fast):.5f}")
    print("   IMU delay: table B, 'inner alone' column (the fast pair crosses |z| = 1 between 10 and 20 ms)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
