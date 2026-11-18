"""Run the existing independent q1/q2 recorder per S3 route leg."""
import json
import os
from pathlib import Path
import signal
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
GEOMETRY = ('footprint_length', 'footprint_width', 'footprint_safety_margin')


class TrackingLeg:
    def __init__(self, probe, index):
        self.probe = probe
        self.directory = probe.directory / 'nav_tracking' / f'goal_{index+1}'
        self.process = self.output = None

    def __enter__(self):
        from s3_fault_scenarios import get_parameters
        self.parameters = get_parameters(self.probe, '/minco_planner',
                                         [*GEOMETRY, 'obstacle_value_threshold', 'unknown_is_obstacle'])
        self.directory.mkdir(parents=True, exist_ok=False)
        (self.directory / 'effective-footprint.json').write_text(json.dumps({
            'effective_parameters': self.parameters}, indent=2)+'\n')
        command = [sys.executable, str(ROOT / 'scripts/nav_tracking_recorder.py'),
                   '--output-dir', str(self.directory), '--rate-hz', '50',
                   '--startup-timeout-sec', '30', '--grid-frame', self.probe.expected_grid_frame,
                   '--max-layer-payloads', os.environ.get('NAV_TRACKING_MAX_LAYER_PAYLOADS', '3000'),
                   '--max-snapshot-payloads', os.environ.get('NAV_TRACKING_MAX_SNAPSHOT_PAYLOADS', '3000')]
        (self.directory / 'recorder-command.json').write_text(json.dumps(command)+'\n')
        self.output = (self.directory / 'recorder.out').open('w')
        self.process = subprocess.Popen(command, stdout=self.output, stderr=self.output, start_new_session=True)
        def ready():
            return f'RECORDER_READY pid={self.process.pid} first_tick=1' in (
                self.directory / 'recorder.out').read_text(errors='replace')

        try:
            self.probe.spin_until(lambda: self.process.poll() is not None or ready(),
                                  32, 'independent tracking recorder first tick')
            self.ready_before_goal = ready()
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
                  'ready_before_goal': self.ready_before_goal,
                  'passed': self.ready_before_goal and self.process.returncode == 0 and analysis.returncode == 0}
        self.probe.__dict__.setdefault('tracking_results', []).append(result)
