# Closure resume validation — 2026-11-07

These are new measurements for the closure-resume change, separate from all
historical controller, accuracy and resume campaigns. Raw evidence is retained
locally under ignored `log/mission-closures-20261107/`.

## Repository and isolated environment

Initial branch was `main`, HEAD and `origin/main` were
`7e23912ddd7115b1bed1e74f3083aec73d41eab6`. A read-only `git ls-remote` also
confirmed that remote HEAD and `refs/heads/main` still named that commit.
There were no staged changes. The preexisting 97-line README addition was preserved;
this change adds a separate feature section and updates the old status sentence.
The preexisting untracked `docs/mission-controller-accuracy-diagnosis.md` was
verified unchanged by SHA-256. Initial status, the original README and its
patch/hash evidence are saved alongside the new measurements. No applicable
`AGENTS.md` was found in the repository or parent directories. No staging,
stash, commit or push was performed.

All ROS builds and trials used an owned rootless Podman container named
`warehouse-closures-20261107`, based on the existing
`localhost/var-e9t-warehouse:humble` image (`c5df849eb3d9`), with a read-only
source mount and a writable dedicated evidence mount. Build/install products
were in its private `/tmp`. Private networking/IPC, `ROS_LOCALHOST_ONLY=1`,
ROS domain 173 and Ignition partition `warehouse_closures_20261107` isolated
the simulation. RViz replay used domain 174. No other project container was
operated. GUI dependencies were installed only in the owned temporary
container, without changing the source image.

## Build and regression

The final source built all four packages successfully in ROS Humble/Fortress:
`warehouse_interfaces`, `warehouse_sim`, `warehouse_planning`,
`warehouse_control`. The installed RViz configuration was compared with source.
The final planning/control regression passed **954 tests: 439 planning and
515 control, zero errors, failures or skips**. This includes 24 new closure
service/marker cases. Logs are `verified-build.txt`, `verified-tests.txt`,
`verified-test-results.txt`, and the corresponding colcon log directories.

New cases verify real typed DDS calls; valid/reversed edges; complete nonempty
replacement; explicit empty reopening; legacy-service closure retention;
unknown endpoints/nonexistent edges/self-loops/reversed duplicates;
unreachable request targets; unknown or position-mismatched start nodes;
non-PAUSED states; moving/short/gapped/stale feedback; injected planning and
precommit failure; unchanged plan/checkpoint/accepted closures and zero commands
on rejection; completed/repeated original request IDs; final depot execution;
marker coordinates in the installed and a rotated static transform; removal
of old closures/labels with DELETEALL; stationary one-node display; and a late
transient-local marker subscriber. Existing pause/resume, planning, execution,
timestamp, shutdown and odometry regressions also ran.

Earlier work-in-progress runs passed 952 and 954 tests. They remain recorded
but do not replace the final-source result above. A later validation-script
import typo (`zero_service` instead of the existing `zero_streak`) prevented
one harness invocation before it started any simulator; its traceback is
preserved as `verified-gazebo.txt`. The harness was corrected before the final
trial. No failed measurement was discarded or relabeled as a pass.

## Final fresh Gazebo trials

`scripts/validate_mission_closures.py` ran two fresh, separately owned worlds
against the final installed source. Both passed. The full results and raw
streams are in `verified-gazebo/`; `verified-gazebo-run.txt` records the run.
The earlier two successful exploratory worlds remain separately under
`gazebo/` and are not used for the final numbers below.

Both final trials requested `r1c0`, `r1c0`, `r0c0`. Request 1 completed before
pause during request 2's simulated service. After braking, independently
observed pause-rest windows contained 58/59 world poses over 0.969/0.986 s
(detour/unreachable), with maximum gaps 0.017 s. Displacement, sampled speed
and yaw change were zero at recorded precision in both windows.

### Accepted detour

Pause was observed at simulation time 19.470 s. The explicit start was
`r1c0`, with the single closed edge `(r0c0, r1c0)`. The accepted remaining
route was 18 m:

```text
r1c0 -> r1c1 -> r1c2 -> r1c3 -> r1c4
     -> r0c4 -> r0c3 -> r0c2 -> r0c1 -> r0c0
```

The first remaining leg serves original request 2 at `r1c0`; the second
traverses that detour and serves original request 3 at the depot; the final
zero-length return leg completes the mission. Every logged post-resume
waypoint, including repeated leg endpoints, exactly matched those planned
legs. Raw independent world poses and odometry record the actual motion.
No closed edge occurs in the accepted route. The pre-resume full plan was
`r0c0 -> r1c0 -> r0c0`, so the return genuinely changed.

The service accepted at observer simulation time 21.505 s with
`Resumed from r1c0; 1 closed edge(s).` Before acceptance, 25 calls were
rejected by the unchanged strictly-newer-feedback check as the service raced
the paused timer; those responses are all preserved. They did not publish
accepted candidate markers or move the paused robot.

| Original request | Target | Completion feedback time (s) | Fresh service wait: feedback / ROS clock (s) | Nearest world-pose target error (m) |
|---|---|---:|---:|---:|
| 1 | r1c0 | 19.400 | 1.000 / 1.000 | 0.049578 |
| 2 | r1c0 | 22.600 | 1.050 / 1.047 | 0.049578 |
| 3 | r0c0 | 167.100 | 1.000 / 1.000 | 0.087436 |

Completion IDs were exactly **1, 2, 3**, once each. Request 2 arrived twice
(before and after pause), and its post-resume wait was a full new interval.
Observed zero-command streaks covered every service wait. Final depot
REACHED used feedback time **167.150 s**, with independent nearest world-pose
error **0.087436 m**. Maximum measured post-resume waypoint error was
0.088443 m. These are sampled results for this route, not a correction of the
historical NN accuracy failure.

### Unreachable closure set

In a separate fresh world, pause was observed at simulation time 19.416 s.
The request supplied `r1c0` and both edges incident to the depot:
`(r0c0, r1c0)` and `(r0c0, r0c1)`. After 28 preserved strictly-newer-feedback
rejections, the planner returned at observer time 21.452 s:

```text
Resume unavailable: mission leg 2 (r1c0 -> r0c0) is unreachable
```

Only original request **1** had completed. No request 2 completion, accepted
resume-plan log, FAULT or REACHED appeared. The last logged controller state
remained PAUSED, all observed commands after the rejected request were zero,
and the accepted marker snapshot remained unchanged when comparing all ROS
message fields. There was deliberately no depot return in this case. The
unit regressions separately verify exact in-memory plan/checkpoint identity
and unchanged accepted closures on rejection.

| Final physical observation | Detour | Unreachable |
|---|---:|---:|
| Recorded world-pose samples | 10220 | 1600 |
| Maximum source-time sample gap (s) | 0.018 | 0.017 |
| Sampled inflated-obstacle intersections | 0 | 0 |
| Rest after one-second braking allowance (s) | 5.015 | 5.013 |
| Rest displacement / speed / yaw change | 0 / 0 / 0 | 0 / 0 / 0 |
| Final velocity command (linear, angular) | (0, 0) | (0, 0) |

Rest thresholds remained 0.001 m, 0.001 m/s and 0.001 rad. World-pose errors
use nearest source-time samples, with the source gaps above; no interpolation
or continuous-position guarantee is claimed. Geometry checks use the existing
sampled-segment/inflated-obstacle method. Manual closures did not spawn an
obstacle. World poses were observations only, never navigation feedback.

The accepted marker message was checked against exact transformed graph
coordinates for old/new routes and the closed edge. Remaining labels were
original IDs 2/3; completion removed them. Every publication began with
DELETEALL. The unreachable run published no replacement accepted snapshot.

## Actual RViz check

The image contains RViz, but the isolated container had no X socket or direct
desktop/GPU access. The first `QT_QPA_PLATFORM=offscreen` attempt failed with
`Couldn't open X display`; this is preserved in `rviz-offscreen.txt`. An owned
Xvfb virtual display and Mesa software rendering then ran the actual RViz
GUI, reporting OpenGL 4.5. The first config attempt had no marker subscription:
inspection of the installed Humble header identified `Topic` as the property
name, and the configuration was corrected. Early screenshots and logs remain
under `rviz-attempt2/` and `rviz-attempt3/`.

The final installed configuration subscribed successfully in isolated domain
174. It displayed an exact replay of the final Gazebo trial's accepted
MarkerArray, including the red closure, blue old plan, green detour, explicit
start and original request IDs 2/3. This was an actual rendering check, not
only a check that the RViz process stayed alive. The final screenshot was
visually inspected; label overlap found in an earlier attempt was corrected.
Evidence: `verified-rviz/rviz-plans.png`, `rviz-input.json`, `rviz-result.json`
and `rviz.txt`. A second screenshot in `verified-rviz-status/` expands the
global warning: `No tf data`. There is no complete TF tree; same-frame odom
markers still render correctly. No host desktop passthrough or interactive
hardware-accelerated desktop session was established or claimed.

## Changed file scope

Modified tracked files:

- `README.md` — only the feature/status hunks belong to this candidate;
  preexisting historical additions remain separate in the working tree.
- `src/warehouse_interfaces/CMakeLists.txt` — generate the new message/service.
- `src/warehouse_control/package.xml` — visualization message dependency.
- `src/warehouse_control/setup.py` — install RViz configuration.
- `src/warehouse_control/warehouse_control/mission_controller.py` — shared
  resume validation/commit, accepted closures and marker publication.
- `src/warehouse_control/test/test_controller.py` — clock mock now returns a
  real ROS Time, including the marker timestamp conversion contract.

New files:

- `src/warehouse_interfaces/msg/ClosedEdge.msg`.
- `src/warehouse_interfaces/srv/ResumeMissionWithClosures.srv`.
- `src/warehouse_control/warehouse_control/mission_markers.py`.
- `src/warehouse_control/rviz/mission_plans.rviz`.
- `src/warehouse_control/test/test_mission_closures_service.py`.
- `scripts/validate_mission_closures.py`.
- `docs/mission-closures.md`.
- `docs/mission-closures-validation.md` (this report).

The diagnosis document remains a preexisting untracked file, not a new change
from this task. No planner algorithm, motion-control law, robot model, graph,
calibration or physical acceptance threshold was changed.

## Final review and limits

The complete tracked diff and every new source/document/configuration file
were reviewed. Whitespace checks cover both tracked changes and untracked
new files. The final branch/HEAD and absence of staged changes were checked
again. Installed controller and marker-module SHA-256 values matched source;
the generated service was inspected through `ros2 interface show`. Colcon
JUnit XML files were copied to the evidence directory before cleanup. After
all trials and RViz processes exited, the owned temporary container was
removed; its absence was checked. Build products in private `/tmp` were
temporary; raw measurements, test XML, source hashes, logs, screenshots and
the compact results remain in the host evidence directory.

Closures are manual logical data, accepted only with a complete replacement
plan. No automatic obstacle detection, physical obstacle insertion, mid-edge
resume, dynamic localization, persistence across node restart, full TF tree,
drive watchdog, continuous collision proof or real-hardware safety guarantee
was added. The original motion law and historical physical-accuracy failure
remain unchanged. One final detour and one final unreachable run establish
this demonstrator's observed behavior, not broad statistical reliability.

## Final candidate review

A separate read-through checked the complete diff/new files, the shared
precommit path, closure replacement/retention, marker replacement and static
coordinate conversion against the saved evidence. No runtime defect was
identified. One documentation defect was corrected: the guide linked to the
untracked accuracy diagnosis, which would be absent from this commit. It now
names that separate local document without a hyperlink and links to committed
odometry diagnostics for calibration limits.

The saved JUnit XML and per-package logs confirm 439 + 515 successful tests.
A read-only offline audit reconciled the final Gazebo result JSON with raw
commands, world poses, controller events and marker streams, including the
recorded RViz input. It reconfirmed request IDs, the 18 m route, depot error,
5.015/5.013 s final rest and the rejected unreachable request. No ROS tests,
Gazebo trials or GUI processes were rerun for this review. Runtime sources
still match the installed source hashes recorded with those measurements.

The exact 14-file commit candidate is captured locally as
`log/mission-closures-review-20261107/candidate.patch`, with a file/hash manifest.
It applies to the recorded HEAD and contains only the closure feature's README
hunks; the earlier 97-line historical README addition is excluded. The
diagnosis and ignored evidence directories are also excluded. The patch was
applied and its relative documentation links checked in a temporary directory,
without touching this repository's index or working files. All original
measurement artifacts were checked against a preservation hash manifest.
