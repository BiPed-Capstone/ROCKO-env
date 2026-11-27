# The status screen shows storage and records a minute; the dashboard lists and downloads each recorded file

- **Date / commit:** 2026-11-27, built on `214e07a`
- **Branch:** `ai-forward`
- **Environment:** macOS laptop (host tests); dev container (Jazzy) for the build, the mock, the simulation and the
  screen drawing. Nothing ran on the Pi.

## Goal and success criteria

The owner asked (2026-11-27): the Pi's storage capacity and use on the screen's System page; a button on the robot's
screen that records for one minute; the folder recordings are saved in, shown; recordings saved to the laptop, with
the dashboard showing which files are on the robot and a way to download them.

Recording happens on the robot (the signals are there); "saved to the laptop" is a download from the dashboard, which
a browser puts in the laptop's Downloads folder. Read that way:

1. The System page shows the Pi's disk: used, total, free; red below the recorder's own minimum
   (`RecordRun.DEFAULT_MIN_FREE_MB`). (host `tools/extras_test.py`; container `tools/screen_render_check.py`)
2. The bottom bar has a fourth button, **Rec 1 min**; a tap asks the dashboard (`/rocko/record`, std_srvs/SetBool)
   to record for `record_button_s` (60 s) and stop by itself; a tap while recording stops it. The button shows the
   time left. With no dashboard it says so and does nothing. (host; T3 `verify_tools.sh` dashboard part)
3. A new Record page on the screen shows where recordings are saved on the robot, whether one is running and its
   folder, the free space, and the dashboard's address for downloading. (host; container drawing)
4. The dashboard lists each run's files with sizes and serves each one (the `.mcap` alone, for Foxglove), only files
   it listed, only from a closed run; and shows the robot's recordings folder and free space. (T3 `verify_tools.sh`)
5. The dashboard still commands nothing: it publishes only its recording report on `/diagnostics`, and calls only
   the services it called before. (T3 `check_dashboard.py`)
6. Every tier that ran before still passes. (`tools/run_ladder.sh`)

## Plan

`sysinfo.py`: disk use. `screen_layout.py`: storage on System, a Record page, four buttons with a colour each.
`screen_draw.py`: button colours. `StatusScreen.py`: the record tap and the dashboard's report. `Dashboard.py`: the
`/rocko/record` service, the timed stop, a once-a-second `rocko/recording` report on `/diagnostics`, the file list
and single-file download. `dashboard.html`: the files and the folder. `check_dashboard.py`: the service, the timed
stop (with `record_button_s` set short at runtime), the file download, the climbing-path refusal for files.
Risks: a stop takes up to 25 s, so it must never run in a ROS callback (it would freeze the dashboard's executor);
a file path from the browser must never reach outside the run folder.

## What changed

- **`sysinfo.py`:** `read_disk_gb()` (used, total, free) and `disk_low()`, the one low-space rule (below
  `RecordRun.DEFAULT_MIN_FREE_MB`, where the recorder refuses to start).
- **`screen_layout.py`, `screen_draw.py`:** the System page has a storage line (CPU and temperature now share a line,
  the IP addresses are smaller, so it still fits); a new Record page; four buttons, each with its own colour, the
  record button red with `Stop m:ss` while recording.
- **`StatusScreen.py`:** the record tap calls `/rocko/record` (asynchronously, one request at a time); the view takes
  the dashboard's `rocko/recording` report, or None once it is older than `health.STALE_AFTER_S`.
- **`Dashboard.py`:** the `/rocko/record` service (`record_button_s`, default 60 s, read at each press); a once-a-second
  timer that stops a timed recording in a background thread and publishes `rocko/recording` on `/diagnostics`; a stop
  lock so a second stop waits instead of signalling the recorder twice; the folder of the running recording (the run
  folder that did not exist when it started); `runs()` lists each file; `/api/files/<run>/<path>` serves one file of a
  closed run, only a path the list holds; `battery_now()` fills a screen run's battery voltage from the battery monitor.
- **`dashboard.html`:** where the runs are saved and the free space; each file a download link; the seconds left of a
  screen recording.
- **Checks:** `check_dashboard.py` presses the record button (above); `check_foxglove_layout.py` waits for its topics
  instead of a fixed 3 s (below); host and drawing checks for the new pages and buttons.
- **Docs:** [the robot extras](../architecture/robot-extras.md#status-screen), [Use the
  dashboard](../guides/use-the-dashboard.md#record-from-the-page), [Record a run](../guides/record-a-run.md), the
  fitting guide, the ladder, next-steps row 4.3, three new screen pictures.

## Verification

- **Host, `python3 -I tools/extras_test.py`:** PASS, with new checks: four quarters, the record button's labels and
  colour (idle, recording, no time limit, no dashboard, no duration sent), the Record page's lines, the storage line
  and its red, and that the System page (two addresses, three extras) and the Record page fit.
- **Container, `tools/screen_render_check.py`:** PASS, now 13 checks (every page, four of them, for three robot states;
  the record button red while recording). The pictures were looked at.
- **T3, `verify_tools.sh`:** PASS; the dashboard part's new record-button section: `the screen's record button made
  20261127_152014_screen, which stopped by itself and downloaded file by file`. Made red: with the timed stop removed
  (`the button's recording was still running 33 s after it started (record_button_s 5)`), and with file paths not
  compared with the list (`..%2F20261127_152345_dashboard%2Frun.yaml gave status 200, expected 404`); restored. The
  first try of the second break crashed the checker instead of printing a FAIL (a dropped connection); its HTTP helper
  now reports that as a FAIL line, and the escape test now aims at a file that exists.
- **A check made sturdier on the way:** a full ladder failed `verify_tools.sh`'s new battery-layout check with
  `topic /pitch_pid_controller/controller_state does not exist` while it existed: the simulation was overloaded at that
  moment (`Update time : 36231.916 us` in the launch log) and `check_foxglove_layout.py` judged after a fixed 3 s of
  discovery. It now waits until the layout's topics have publishers, up to 15 s. Made red with a made-up topic in the
  layout (`topic /no_such_topic does not exist`).
- **Not caused by this change, recorded:** the dashboard check's history overlap (`only N history points overlap`,
  limit 40) failed twice, at 38 and 39, both right after deliberate-break runs. Alternating runs of the old and the new
  dashboard: new 97, 107, 65; old 65, 61, 67, and the old one gave 59 earlier. The count swings between about 60 and
  120 in the container either way, so the margin over 40 is thin; it is a pre-existing weakness, not fixed here.
- **Full ladder, `tools/run_ladder.sh`:** `RESULT: PASS`, 31 steps, 0 compiler warnings, after the review fixes.
- **Not run:** anything on the Pi; the touch panel (the button was driven by calling the service as the screen does,
  not by a tap); downloading a large bag over Wi-Fi; the disk numbers on a real SD card.

## Review rounds

| Round | Reviewer | Findings | Fixed / declined |
| --- | --- | --- | --- |
| 1 | ros2-control-reviewer | a page Stop or a shutdown during a timed stop signalled the recorder a second time while it closed the bag; the check waited a fixed 2.5 s for a report; a dashboard that does not answer crashed the check | all fixed: a stop lock held for the whole stop; wait for the report up to the recording's length; a `recording_state()` helper that yields a FAIL |
| 1 | explainability-reviewer | the record time and the low-space rule written twice; the report's "" convention parsed in three places; `-math.inf`; import order; long docstring lines; a bare `"screen"` | all fixed: the dashboard sends `button_s` and `low_space`, `sysinfo.disk_low` is the one rule, `seconds_left()`, `SCREEN_LABEL` |
| 1 | docs-reviewer | "greys it out" (it reads `Rec ?`); "counts down" (it reads `Stop 0:59`); 500 MB copied | all fixed (the number linked to Record a run's table) |
| 2 | ros2-control-reviewer (the fixes) | clean: no deadlock between the two locks, a second stop waits and sends no second signal, the check's waits bounded, the low-space rule shared | none needed |

## What a human should do on the robot

After [fitting the screen](../guides/fit-the-extras.md) (its step 5): compare the System page's storage line with
`df -h /` on the Pi; tap **Rec 1 min** with the robot disabled and watch it count down and stop by itself; then, on a
laptop, open the dashboard, find the run (label `screen`) and download its `.mcap`; open it in Foxglove.
