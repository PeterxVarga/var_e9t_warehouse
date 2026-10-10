#!/usr/bin/env python3
"""Fit one offline effective-track candidate from repeated +/-0.6 yaw primitives.

No route outcomes or runtime world-pose feedback participate in this fit.
The candidate is written to a temporary model directory, never to source.
"""

import argparse
import json
from pathlib import Path
import shutil
import statistics
import xml.etree.ElementTree as ET


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--measurements', type=Path, required=True)
    parser.add_argument('--model-source', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    data = json.loads(args.measurements.read_text())
    cases = ('turn_positive_0.6', 'turn_negative_0.6')
    training = [result for result in data if result['case'] in cases]
    for case in cases:
        selected = [result for result in training if result['case'] == case]
        if len(selected) != 3 or not all(result['passed'] for result in selected):
            raise ValueError('exactly three successful repeats per +/-0.6 case are required')
    nominal_values = {result['geometry']['diffdrive_separation_m'] for result in training}
    if len(nominal_values) != 1:
        raise ValueError('training geometries differ')
    nominal = nominal_values.pop()
    pairs = [(result['phases'][0]['odom_yaw_change'], result['phases'][0]['physical_yaw_change'])
             for result in training]
    gain = sum(odom * physical for odom, physical in pairs) / sum(odom**2 for odom, _ in pairs)
    raw = nominal / gain
    candidate = round(raw, 3)  # Predeclared millimetre precision; no route-based tuning.
    residuals = [physical - gain * odom for odom, physical in pairs]
    args.output.mkdir(parents=True, exist_ok=True)
    model_dir = args.output / 'models/warehouse_robot'
    shutil.copytree(args.model_source, model_dir)
    path = model_dir / 'model.sdf'
    tree = ET.parse(path)
    plugin = tree.getroot().find("model/plugin[@name='ignition::gazebo::systems::DiffDrive']")
    plugin.find('wheel_separation').text = f'{candidate:.3f}'
    tree.write(path, encoding='unicode', xml_declaration=True)
    report = dict(training_file=str(args.measurements), training_cases=cases,
                  training_repetitions_per_direction=3, angular_command_magnitude=0.6,
                  nominal_track_m=nominal, fitted_physical_to_odom_yaw_gain=gain,
                  unrounded_effective_track_m=raw, candidate_track_m=candidate,
                  rounding_resolution_m=0.001, residual_yaw_std_rad=statistics.stdev(residuals),
                  training=[dict(case=result['case'], repeat=result['repeat'],
                                 phase=result['phases'][0]) for result in training],
                  excluded_inputs='All route outcomes, all other rates, and narrowed-contact data')
    (args.output / 'fit.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps({key: value for key, value in report.items() if key != 'training'}))


if __name__ == '__main__':
    main()
