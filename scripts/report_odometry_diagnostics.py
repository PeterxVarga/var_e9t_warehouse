#!/usr/bin/env python3
"""Publish an English report only after all diagnostic/validation batches finish."""

import argparse
import hashlib
import io
import json
from pathlib import Path
import statistics
import tarfile
import xml.etree.ElementTree as ET

from analyze_odometry import geometry_audit, waypoint_metrics


def stats(values):
    return dict(n=len(values), mean=statistics.mean(values), min=min(values), max=max(values),
                sample_std=statistics.stdev(values) if len(values) > 1 else 0.0)


def primitive_stats(results):
    data = {}
    for case in sorted({result['case'] for result in results}):
        selected = [result for result in results if result['case'] == case]
        phases = []
        for index in range(len(selected[0]['phases'])):
            values = [result['phases'][index] for result in selected]
            metrics = ('yaw_ratio', 'yaw_change_error_rad', 'distance_error_m',
                       'direction_error_rad', 'lateral_error_m', 'physical_distance_m', 'odom_distance_m')
            phases.append({key: stats([value[key] for value in values])
                           for key in metrics if all(value[key] is not None for value in values)})
        data[case] = phases
    return data


def draw_figure(data, path):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    before_color, after_color = '#c25122', '#156082'
    fig, axes = plt.subplots(2, 2, figsize=(12, 8))
    for state, color in (('baseline', before_color), ('corrected', after_color)):
        groups = data['primitive_statistics'][state]
        for direction, marker in (('positive', 'o'), ('negative', '^')):
            rates = [0.2, 0.4, 0.6]
            values = [groups[f'turn_{direction}_{rate}'][0]['yaw_ratio'] for rate in rates]
            axes[0, 0].errorbar(rates, [v['mean'] for v in values],
                                yerr=[v['sample_std'] for v in values], capsize=3,
                                marker=marker, color=color, linestyle='-' if direction=='positive' else ':',
                                label=f'{state}, {direction} turn')
    axes[0, 0].axhline(1, color='black', linestyle='--', linewidth=1)
    axes[0, 0].set(xlabel='Angular command magnitude (rad/s)', ylabel='Physical / odom yaw gain',
                  title='A. Turn gain: rate dependence remains')
    axes[0, 0].legend(fontsize=8)
    names = ['straight_015', 'straight_030', 'turn_then_straight']
    for state, offset, color in (('baseline', -.18, before_color), ('corrected', .18, after_color)):
        values = [data['primitive_statistics'][state][name][-1]['lateral_error_m'] for name in names]
        axes[0, 1].bar([i+offset for i in range(3)], [v['mean'] for v in values], width=.36,
                       yerr=[v['sample_std'] for v in values], capsize=3, color=color, label=state)
    axes[0, 1].set_xticks(range(3), ['Fresh 0.15 m/s', 'Fresh 0.30 m/s', 'After turn, 0.15 m/s'])
    axes[0, 1].set(ylabel='Lateral disagreement (m)', title='B. Two-metre straight primitives')
    axes[0, 1].legend(fontsize=8)
    for state, color in (('baseline', before_color), ('corrected', after_color)):
        values = data['waypoint_statistics'][state]['r3c2']
        axes[1, 0].errorbar(range(len(values)), [v['position_disagreement_m']['mean'] for v in values],
                            yerr=[v['position_disagreement_m']['sample_std'] for v in values],
                            marker='o', capsize=3, color=color, label=state)
    axes[1, 0].set_xticks(range(6), [v['node'] for v in data['waypoint_statistics']['baseline']['r3c2']], rotation=25)
    axes[1, 0].set(ylabel='Physical / odom position disagreement (m)', title='C. Accumulation at outer-route waypoints')
    axes[1, 0].legend(fontsize=8)
    route_names = ['r3c2', 'r2c2', 'r3c4']
    for state, offset, color in (('baseline', -.18, before_color), ('corrected', .18, after_color)):
        available = [goal for goal in route_names if goal in data['route_statistics'][state]]
        values = [data['route_statistics'][state][goal]['physical_error_m'] for goal in available]
        axes[1, 1].bar([route_names.index(goal)+offset for goal in available], [v['mean'] for v in values],
                       width=.36, yerr=[v['sample_std'] for v in values], capsize=3, color=color, label=state)
    axes[1, 1].axhline(.15, linestyle='--', color='black', linewidth=1, label='0.15 m limit')
    axes[1, 1].text(1.82, .01, 'not\nmeasured', ha='center', fontsize=8, color='#555555')
    axes[1, 1].set_xticks(range(3), ['r3c2', 'r2c2', 'r3c4 (held out)'])
    axes[1, 1].set(ylabel='Final physical goal error (m)', title='D. Route acceptance after braking')
    axes[1, 1].legend(fontsize=8)
    for ax in axes.flat:
        ax.grid(axis='y', alpha=.2)
        ax.set_axisbelow(True)
    fig.suptitle('Wheel odometry calibration — Fortress / DART\nThree fresh runs per condition; error bars are sample SD', fontsize=13)
    fig.tight_layout(rect=(0, 0, 1, .94))
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=160)
    svg_path = path.with_suffix('.svg')
    fig.savefig(svg_path)
    # Matplotlib leaves spaces before newlines in SVG path attributes.
    svg_path.write_text('\n'.join(line.rstrip() for line in svg_path.read_text().splitlines()) + '\n')
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, required=True)
    parser.add_argument('--plot', action='store_true')
    args = parser.parse_args()
    root = args.root.resolve()
    artifacts = root / 'log/odometry-diagnostics'
    batches = {}
    sizes = {'baseline-primitives':27, 'baseline-outer':3, 'baseline-inner':3,
             'narrow-contact-turns':6, 'candidate-turns':6, 'corrected-primitives':27,
             'corrected-outer':3, 'corrected-inner':3, 'holdout':3, 'corrected-faults':8}
    for name, size in sizes.items():
        results = json.loads((artifacts / name / 'results.json').read_text())
        if len(results) != size:
            raise ValueError(f'{name}: {len(results)} of {size} cases complete')
        if name not in ('baseline-outer', 'baseline-inner'):
            assert all(result['passed'] for result in results), name
        else:
            assert all(not result['gates']['physical_accuracy']
                       and all(value for key, value in result['gates'].items() if key!='physical_accuracy')
                       for result in results), name
        batches[name] = results
    package_tests = {}
    for name in ('warehouse_control', 'warehouse_planning'):
        element = ET.parse(root / 'build' / name / 'pytest.xml').getroot()
        suite = element if element.tag=='testsuite' else element.find('testsuite')
        assert all(int(suite.attrib[key])==0 for key in ('errors','failures','skipped'))
        package_tests[name] = int(suite.attrib['tests'])
    assert sum(package_tests.values())==242
    fit = json.loads((artifacts / 'effective-track-candidate/fit.json').read_text())
    with tarfile.open(artifacts / 'before-diagnostics.tar.gz') as archive:
        baseline_model = archive.extractfile('src/warehouse_sim/models/warehouse_robot/model.sdf').read()
    data = dict(date='2026-10-10', source_base_commit='8286d3076aeea5a4c2f9fd0f4cd6bbf8ae76949c',
                environment=dict(image='localhost/var-e9t-warehouse:humble', ros='Humble',
                                 gazebo='6.18.0', physics_engine='DART 5.4.0', python='3.10.12'),
                calibration=fit, package_tests=package_tests,
                baseline_geometry=geometry_audit(io.BytesIO(baseline_model)),
                corrected_geometry=geometry_audit(root/'src/warehouse_sim/models/warehouse_robot/model.sdf'),
                batches=batches, primitive_statistics={}, route_statistics={}, waypoint_statistics={},
                waypoint_measurements={}, holdout_goal='r3c4', physical_acceptance_limit_m=.15,
                raw_artifact_directory='log/odometry-diagnostics',
                baseline_model_sha256=hashlib.sha256(baseline_model).hexdigest())
    graph, world = root/'src/warehouse_planning/config/warehouse_graph.json', root/'src/warehouse_sim/worlds/warehouse.sdf'
    for state in ('baseline','corrected'):
        data['primitive_statistics'][state] = primitive_stats(batches[f'{state}-primitives'])
        data['route_statistics'][state], data['waypoint_statistics'][state] = {}, {}
        goals = [('r3c2', f'{state}-outer'), ('r2c2', f'{state}-inner')]
        if state=='corrected': goals.append(('r3c4','holdout'))
        for goal, batch in goals:
            selected = batches[batch]
            data['route_statistics'][state][goal] = {
                key:stats([result[key] for result in selected]) for key in
                ('physical_error_m','odom_error_m','execution_sim_seconds','rest_seconds',
                 'physical_distance_m','odom_distance_m','max_physical_gap_seconds')}
            per_repeat = []
            for repeat in range(1,4):
                directory = artifacts/batch/f'repeat-{repeat}'/goal
                values = waypoint_metrics(directory, graph, world)
                (directory/'waypoints.json').write_text(json.dumps(values,indent=2)+'\n')
                per_repeat.append(values)
                data['waypoint_measurements'][str(directory.relative_to(root))] = values
            data['waypoint_statistics'][state][goal] = [dict(
                node=per_repeat[0][index]['node'],
                **{key:stats([values[index][key] for values in per_repeat]) for key in
                   ('position_disagreement_m','yaw_disagreement_rad','physical_goal_error_m','odom_goal_error_m')})
                for index in range(len(per_repeat[0]))]
    source_files = sorted((root/'src').rglob('*.py')) + sorted((root/'scripts').glob('*.py'))
    source_files += [root/'src/warehouse_sim/models/warehouse_robot/model.sdf', world, graph,
                     root/'src/warehouse_sim/config/bridge.yaml']
    data['source_sha256'] = {str(path.relative_to(root)):hashlib.sha256(path.read_bytes()).hexdigest()
                             for path in source_files if '__pycache__' not in str(path)}
    (root/'docs/odometry-diagnostics-results.json').write_text(json.dumps(data,indent=2)+'\n')
    if args.plot: draw_figure(data, root/'docs/images/odometry-diagnostics.png')
    report = '''# Wheel odometry diagnosis and calibration — 2026-10-10

The original physical goal-error failure is resolved for the tested simulation
configuration. Both original routes and the reserved `r3c4` route passed all
physical gates in three fresh runs each. The acceptance limit remains **0.15 m**.
Gazebo world pose remained an independent verification stream throughout.

## Measurement design

Every trial starts a fresh installed world in an owned rootless Humble
container. Concurrent batches use separate ROS domains (80–88) and Ignition
partitions; each world has one command publisher at most. The fault batch uses
domain 89. The container image is `localhost/var-e9t-warehouse:humble`.
Versions: Ubuntu 22.04, ROS 2 Humble, Gazebo Fortress 6.18.0, loaded DART physics
plugin 5.4.0 and Python 3.10.12.

The baseline includes 27 primitive trials and six full routes. The primitive
matrix has three repeats of each +/-0.2, +/-0.4 and +/-0.6 rad/s turn, each
0.15 and 0.30 m/s two-metre straight, and a +0.6 turn followed by a 0.15 m/s
straight. Wheel odometry decides when to send zero; physical pose does not
control motion. A turn stops commanding at pi/2 odom yaw change, then brakes,
so the measured final angle includes braking and need not equal pi/2.

Physical poses come directly from Gazebo transport with source simulation
timestamps. Endpoint and waypoint analysis brackets and interpolates both
streams at common timestamps, with yaw interpolation along the short arc.
Unbracketed data, non-increasing timestamps or gaps above 0.1 s are rejected.
Waypoint logs now expose the odometry timestamp used at each arrival. Endpoint
distance is the displacement norm; direction error is the difference between
physical and odom displacement bearings; lateral error projects the physical
straight displacement normal to its initial odom heading. World frame origin
and yaw are taken from the installed SDF spawn.

All reported means, ranges and sample SDs describe three fresh deterministic
simulation runs. They are not hardware accuracy or localization uncertainty.

## Geometry audit and contact sensitivity

The original model was geometrically consistent: wheel centers `(0,+/-0.19,
0.08)` m, separation 0.38 m, collision radius 0.08 m, tread width 0.02 m, joint
axes along y and cylinder axes parallel to the joints after the collision pose
rotation. Both wheel centers are at radius height. Cylinder inertias match
0.20 kg, 0.08 m radius and 0.02 m width. Joint names also match DiffDrive.

Straight-only distance and heading data do not support a radius correction.
The turn data identify a systematic, nearly direction-symmetric yaw gain with
rate dependence. A separate six-run experiment changed only the *temporary*
wheel collision width from 0.02 to 0.002 m, retaining nominal centers, radius,
visuals, mass and inertia. The physical/odom yaw gain at +/-0.6 fell from about
1.0469 to 1.0044. This demonstrates contact-width sensitivity; it does not
identify individual contact forces or prove a physics-engine defect. The narrow
contact proxy was not deployed and did not participate in the calibration fit.

## Offline calibration

Only the six standalone +/-0.6 rad/s baseline turns (three per direction) were
used. All route results, other rates, turn-then-straight results and narrowed
contact data were excluded. The zero-intercept least-squares relation is
`physical_delta_yaw = k * odom_delta_yaw`.

'''
    report += (f"Measured `k = {fit['fitted_physical_to_odom_yaw_gain']:.9f}`. "
               f"The implied effective track is `0.38 / k = {fit['unrounded_effective_track_m']:.9f}` m. "
               f"The predefined millimetre rounding gives **{fit['candidate_track_m']:.3f} m**. "
               f"Fit residual sample SD was {fit['residual_yaw_std_rad']:.3g} rad.\n\n")
    report += '''Before editing the production model, six independent temporary-candidate
turns verified the value. The largest absolute yaw mismatch was below 0.000231
rad. The production change is only DiffDrive's effective `wheel_separation`:
**0.38 → 0.363 m**. Physical center separation remains 0.38 m; radius, tread,
collision geometry, mass, inertia, world, graph and controller gains remain
unchanged. This is an empirical effective rolling track for these dynamics,
not a claim that the mechanical center distance changed.

The [Fortress DiffDrive source](https://github.com/gazebosim/gz-sim/blob/ign-gazebo6/src/systems/diff_drive/DiffDrive.cc)
uses the configured separation for both wheel-command conversion and encoder
odometry. Calibration therefore corrects the simulated kinematic mapping;
it introduces no runtime world-pose input, goal-specific offset or localization.
The installed model SHA-256 was checked against the source and matched.

## Repeated primitive results

Gain below is physical yaw change divided by odom yaw change. Every row has
three fresh runs; SD is sample SD. Angles include braking.

| Angular command | Before gain, mean ± SD | After gain, mean ± SD | Before / after absolute yaw error, mean (rad) |
|---|---:|---:|---:|
'''
    for sign in ('positive','negative'):
        for rate in (.2,.4,.6):
            before = data['primitive_statistics']['baseline'][f'turn_{sign}_{rate}'][0]
            after = data['primitive_statistics']['corrected'][f'turn_{sign}_{rate}'][0]
            b, a = before['yaw_ratio'], after['yaw_ratio']
            report += (f"| {'+' if sign=='positive' else '-'}{rate:.1f} rad/s | {b['mean']:.6f} ± {b['sample_std']:.2g} | "
                       f"{a['mean']:.6f} ± {a['sample_std']:.2g} | {abs(before['yaw_change_error_rad']['mean']):.6f} / "
                       f"{abs(after['yaw_change_error_rad']['mean']):.6f} |\n")
    report += '''
| Straight condition | Before / after maximum distance mismatch (mm) | Before / after direction error, mean (rad) | Before / after lateral error, mean (m) |
|---|---:|---:|---:|
'''
    def distance_max_mm(values):
        maximum = max(abs(values['min']),abs(values['max']))*1000
        return '<0.001' if maximum<.001 else f'{maximum:.3f}'
    for case, label in [('straight_015','Fresh, 0.15 m/s'),('straight_030','Fresh, 0.30 m/s'),
                         ('turn_then_straight','After +0.6 turn, 0.15 m/s')]:
        b=data['primitive_statistics']['baseline'][case][-1]
        a=data['primitive_statistics']['corrected'][case][-1]
        report += (f"| {label} | {distance_max_mm(b['distance_error_m'])} / {distance_max_mm(a['distance_error_m'])} | "
                   f"{b['direction_error_rad']['mean']:.6f} / {a['direction_error_rad']['mean']:.6f} | "
                   f"{b['lateral_error_m']['mean']:.6f} / {a['lateral_error_m']['mean']:.6f} |\n")
    report += '''
Fresh straight runs have negligible additional heading drift. The baseline
turn-then-straight direction error is inherited from the turn, rather than
created by straight distance scaling. Raw values below the displayed distance
precision are available in the JSON; they are solver measurements, not a
micrometre hardware accuracy claim. Lower-rate yaw bias remains after the
single effective-track correction, so it is not a universal calibration.

## Accumulation at waypoints

Position disagreement is the physical pose minus transformed odometry at the
same arrival timestamp. It is distinct from error relative to the graph goal.
Values are means of three runs. Per-run vectors, goal errors, yaw values and
statistics are preserved in the machine-readable report.

'''
    for goal in ('r3c2','r2c2','r3c4'):
        report += f'### `{goal}`'+(' — reserved route' if goal=='r3c4' else '')+'\n\n'
        report += '| Waypoint | Before / after position disagreement (m) | Before / after yaw disagreement (rad) |\n|---|---:|---:|\n'
        after=data['waypoint_statistics']['corrected'][goal]
        before=data['waypoint_statistics']['baseline'].get(goal)
        for index,a in enumerate(after):
            before_position=f"{before[index]['position_disagreement_m']['mean']:.6f}" if before else 'not measured'
            before_yaw=f"{before[index]['yaw_disagreement_rad']['mean']:.6f}" if before else 'not measured'
            report += (f"| {a['node']} | {before_position} / {a['position_disagreement_m']['mean']:.6f} | "
                       f"{before_yaw} / {a['yaw_disagreement_rad']['mean']:.6f} |\n")
        report+='\n'
    report += '''The second turn on the original routes largely cancels the accumulated yaw
bias but does not undo the position error acquired on earlier straight legs.
The correction greatly reduces that positional accumulation. Final graph-goal
error can be smaller than odom error when the residual vector and termination
offset oppose each other; waypoint disagreement and the reserved route are
therefore reported independently.

The corrected moving legs retain roughly -0.0037 to -0.0039 rad mean heading
disagreement. Standalone turn accuracy does not eliminate the residual from
combined turning and advancing; the tables quantify its remaining accumulation.

## Full route validation after calibration

The same gates remain: ordered waypoints, at most 180 simulation seconds,
0.05 m final odom error, **0.15 m physical error**, zero command, at least five
seconds of physical rest after braking, and conservative sampled segment checks
against seven shelf/wall obstacles expanded by a 0.3202 m robot radius.
Each trial also has a finite 240 s wall-time deadline. The installed controller
is launched through `ros2 run` from `/tmp` with `use_sim_time:=true`.

| Route | Length | Runs | Time range (s) | Odom error range (m) | Physical error range (m) | Result |
|---|---:|---:|---:|---:|---:|---|
'''
    for goal in ('r3c2','r2c2','r3c4'):
        group=data['route_statistics']['corrected'][goal]
        batch=batches[{'r3c2':'corrected-outer','r2c2':'corrected-inner','r3c4':'holdout'}[goal]]
        t,o,p=group['execution_sim_seconds'],group['odom_error_m'],group['physical_error_m']
        report += (f"| {goal}{' (reserved)' if goal=='r3c4' else ''} | {batch[0]['planned_length_m']:.0f} m | 3 | "
                   f"{t['min']:.3f}–{t['max']:.3f} | {o['min']:.6f}–{o['max']:.6f} | "
                   f"{p['min']:.6f}–{p['max']:.6f} | all gates passed |\n")
    maximum_gap=max(result['max_physical_gap_seconds'] for name in ('corrected-outer','corrected-inner','holdout') for result in batches[name])
    minimum_rest=min(result['rest_seconds'] for name in ('corrected-outer','corrected-inner','holdout') for result in batches[name])
    report += (f"\nAll nine routes had zero sampled obstacle intersections and zero final commands. "
               f"The largest physical sample gap was {maximum_gap:.3f} s; minimum verified rest was {minimum_rest:.3f} s. "
               "Maximum measured rest translation, sampled speed and yaw change were all zero.\n\n")
    report += '''`r3c4` was declared as the holdout before fitting and was first run only after
the parameter was frozen. Its 14 m route traverses the bottom aisle first,
then the right-hand vertical aisle, with one left turn; the original routes
start up the left aisle and then turn right. All eight held-out waypoints
arrived in order. No parameter was revised from route outcomes.

The calibrated model also passed the eight fault/regression cases: odom bridge
interruption while the command bridge remains active, feedback recovery without
rearming, actual terminal Ctrl+C, direct node SIGTERM, invalid goal, unreachable
graph overlay, synthetic invalid odom start, real moved start, and the existing
single-goal regression. The odom interruption/recovery is one case. Each
included independent physical rest checking. Final Humble package results:
**242 passed, 0 failures, 0 errors, 0 skipped**.

'''
    if args.plot:
        report += '![Repeated odometry and route measurements](images/odometry-diagnostics.png)\n\n'
    report += '''## Reproduction and artifacts

Build/source the installed workspace inside a dedicated isolated container.
Use fresh output paths; the harness refuses to overwrite existing measurements.

```bash
export ROS_DOMAIN_ID=80 IGN_PARTITION=warehouse-diagnostic-reproduction
source /opt/ros/humble/setup.bash
source /workspace/install/setup.bash
cd /tmp
python3 /workspace/scripts/diagnose_odometry.py \\
  --repeat 3 --output /workspace/log/new-primitive-run
python3 /workspace/scripts/validate_route_following.py \\
  --case r3c2 --case r2c2 --case r3c4 --repeat 3 \\
  --output /workspace/log/new-route-run
```

[Machine-readable measurements](odometry-diagnostics-results.json) include
training inputs, per-run outcomes, waypoint vectors, summary statistics, model
audits and source hashes. The ignored `log/odometry-diagnostics/` directory holds
raw timestamped physical/odom/command streams, logs, fit parameters and temporary
models. The original local changes were snapshotted before this follow-up.
The [pre-calibration validation record](route-following-validation.md) remains
available as historical evidence. No commit or push was performed.

To reproduce the baseline without changing source, copy the model directory to
a fresh temporary `models/warehouse_robot` resource tree and set only its
DiffDrive `wheel_separation` back to `0.38`. Pass that tree with `--model-dir`
to the primitive and route harnesses. The fit tool
`scripts/fit_diffdrive_calibration.py` accepts the resulting primitive
`results.json` and writes a candidate into a new output directory; it never
edits the production model. Test the candidate before applying any source change.

## Limits

This is a calibrated simulation configuration, not online localization or a
hardware guarantee. The effective track differs from mechanical center spacing
and is specific to the observed Fortress/DART contact dynamics, friction,
tread, mass, motion rates and solver settings. The measured low-rate yaw bias
remains; changed dynamics need independent validation. Three repeats quantify
repeatability under these conditions, not a population reliability estimate.

Gazebo world pose was recorded during motion and analyzed offline; it never
selected a waypoint, steered a running controller or corrected its live pose. Route
odometry covariance still does not represent measured localization uncertainty.
Sampled geometry is not a continuous collision/contact guarantee. No independent
drive watchdog, dynamic obstacle handling or replanning was added; a crash,
SIGKILL or command-bridge failure can still leave DiffDrive holding a command.
'''
    (root/'docs/odometry-diagnostics.md').write_text(report)
    print(json.dumps({'package_tests':sum(package_tests.values()), 'validated_routes':9,
                      'physical_error_ranges':{goal:{'min':value['physical_error_m']['min'],
                                                     'max':value['physical_error_m']['max']}
                                               for goal,value in data['route_statistics']['corrected'].items()}}))


if __name__ == '__main__':
    main()
