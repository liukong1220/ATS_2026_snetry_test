"""Run the existing independent q1/q2 recorder per S3 route leg."""
import json
import os
from pathlib import Path
import signal
import subprocess
import sys

import yaml

ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / 'src/sim/ats_mujoco_sim/config'
GEOMETRY = ('footprint_length', 'footprint_width', 'footprint_safety_margin')


def profile_geometry(profile, override=None):
    paths = ([Path(override)] if override else
             [CONFIG / 'rmuc_2025_navigation.yaml',
              CONFIG / profile['launch'].replace('_mujoco.launch.py', '_navigation.yaml')])
    values, sources = {}, []
    for path in dict.fromkeys(paths):
        section = yaml.safe_load(path.read_text()).get('minco_planner', {}).get('ros__parameters', {})
        values.update({k: section[k] for k in GEOMETRY if k in section})
        sources.append(str(path))
    if set(values) != set(GEOMETRY):
        raise ValueError('profile lacks the complete MINCO footprint')
    return values, sources


def verify_runtime_geometry(configured, effective):
    for key in GEOMETRY:
        if abs(float(configured[key]) - float(effective[key])) > 1e-9:
            raise ValueError(f'tracking footprint differs from runtime {key}: '
                             f'{configured[key]} != {effective[key]}')


class TrackingLeg:
    def __init__(self, probe, profile, index):
        self.probe, self.profile = probe, profile
        self.directory = probe.directory / 'nav_tracking' / f'goal_{index+1}'
        self.process = self.output = None

    def __enter__(self):
        from s3_fault_scenarios import get_parameters, require
        configured, sources = profile_geometry(self.profile, os.environ.get('NAV_TRACKING_PROFILE_YAML'))
        self.parameters = get_parameters(self.probe, '/minco_planner',
                                         [*GEOMETRY, 'obstacle_value_threshold', 'unknown_is_obstacle'])
        verify_runtime_geometry(configured, self.parameters)
        self.directory.mkdir(parents=True, exist_ok=False)
        (self.directory / 'effective-footprint.json').write_text(json.dumps({
            'profile_sources': sources, 'effective_parameters': self.parameters}, indent=2)+'\n')
        command = [sys.executable, str(ROOT / 'scripts/nav_tracking_recorder.py'),
                   '--output-dir', str(self.directory), '--rate-hz', '50',
                   '--startup-timeout-sec', '30', '--grid-frame', self.probe.expected_grid_frame,
                   '--max-layer-payloads', os.environ.get('NAV_TRACKING_MAX_LAYER_PAYLOADS', '3000'),
                   '--max-snapshot-payloads', os.environ.get('NAV_TRACKING_MAX_SNAPSHOT_PAYLOADS', '3000')]
        (self.directory / 'recorder-command.json').write_text(json.dumps(command)+'\n')
        self.output = (self.directory / 'recorder.out').open('w')
        self.process = subprocess.Popen(command, stdout=self.output, stderr=self.output, start_new_session=True)
        try:
            def ready():
                if self.process.poll() is not None:
                    raise RuntimeError(f'tracking recorder exited {self.process.returncode}')
                return f'RECORDER_READY pid={self.process.pid} first_tick=1' in (
                    self.directory / 'recorder.out').read_text(errors='replace')
            require(self.probe, ready, 32, 'independent tracking recorder first complete tick')
        except BaseException:
            self.stop()
            raise
        return self

    def stop(self):
        if self.process and self.process.poll() is None:
            os.killpg(self.process.pid, signal.SIGINT)
            try:
                self.process.wait(timeout=20)
            except subprocess.TimeoutExpired:
                os.killpg(self.process.pid, signal.SIGTERM)
                self.process.wait(timeout=10)
        if self.output:
            self.output.close()

    def __exit__(self, error_type, error, traceback):
        self.stop()
        p = self.parameters
        command = [sys.executable, '-s', str(ROOT / 'scripts/analyze_nav_tracking.py'),
                   '--input-dir', str(self.directory), '--output', str(self.directory / 'verdict.json'),
                   '--length', str(p['footprint_length']), '--width', str(p['footprint_width']),
                   '--safety-margin', str(p['footprint_safety_margin']),
                   '--obstacle-value-threshold', str(p['obstacle_value_threshold']),
                   '--unknown-is-obstacle' if p['unknown_is_obstacle'] else '--known-only',
                   '--persist-min-ticks', os.environ.get('NAV_TRACKING_PERSIST_MIN_TICKS', '3'),
                   '--persist-min-snapshots', os.environ.get('NAV_TRACKING_PERSIST_MIN_SNAPSHOTS', '2')]
        if os.environ.get('NAV_TRACKING_STRICT') == '1':
            command.append('--strict-diagnostics')
        (self.directory / 'analysis-command.json').write_text(json.dumps(command)+'\n')
        with (self.directory / 'analysis.out').open('w') as output:
            analysis = subprocess.Popen(command, stdout=output, stderr=output, start_new_session=True)
            try:
                if not self.probe.spin_until(lambda: analysis.poll() is not None, 120, 'tracking analysis'):
                    raise RuntimeError('tracking analysis timeout')
            finally:
                if analysis.poll() is None:
                    os.killpg(analysis.pid, signal.SIGTERM)
                    analysis.wait(timeout=10)
        result = {'directory': str(self.directory), 'recorder_exit': self.process.returncode,
                  'analysis_exit': analysis.returncode,
                  'passed': self.process.returncode == 0 and analysis.returncode == 0}
        self.probe.__dict__.setdefault('tracking_results', []).append(result)
        if error_type is None and not result['passed']:
            raise RuntimeError('independent tracking evidence failed: '+str(result))
        return False
