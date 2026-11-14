#!/usr/bin/env python3
"""Read actual ROS parameter services in an already allocated experiment domain."""
import argparse
import json
import os
from pathlib import Path
import time


VALUE_FIELDS = ('', 'bool_value', 'integer_value', 'double_value', 'string_value',
                'byte_array_value', 'bool_array_value', 'integer_array_value',
                'double_array_value', 'string_array_value')
NODES = ('ats_rog_map', 'ats_rog_map_adapter', 'terrain_analysis_ext', 'minco_planner',
         'ats_goal_manager', 'ats_swerve_mpc', 'small_gicp_relocalization', 'localization_fusion')


def parameter_value(value):
    if value.type == 0:
        return None
    if not 0 < value.type < len(VALUE_FIELDS):
        raise ValueError(f'unknown ROS parameter type {value.type}')
    result = getattr(value, VALUE_FIELDS[value.type])
    return list(result) if value.type >= 5 else result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--timeout', type=float, default=180.0, help='wall seconds')
    args = parser.parse_args()
    if 'ROS_DOMAIN_ID' not in os.environ:
        parser.error('set the experiment ROS_DOMAIN_ID explicitly')
    if args.output.exists():
        parser.error('choose a fresh evidence output')
    import rclpy
    from rcl_interfaces.srv import GetParameters, ListParameters
    rclpy.init()
    node = rclpy.create_node('navigation_parameter_evaluator')
    jobs = {}
    for name in NODES:
        jobs[name] = {
            'list': node.create_client(ListParameters, f'/{name}/list_parameters'),
            'get': node.create_client(GetParameters, f'/{name}/get_parameters'),
            'future': None, 'phase': 'waiting'}
    result = {'domain': os.environ['ROS_DOMAIN_ID'], 'nodes': {},
              'pid': os.getpid(), 'pgid': os.getpgrp(),
              'scope': 'actual service responses; no atomic cross-node/config snapshot'}
    deadline = time.monotonic() + args.timeout
    try:
        while time.monotonic() < deadline and len(result['nodes']) < len(jobs):
            rclpy.spin_once(node, timeout_sec=0.1)
            for name, job in jobs.items():
                if name in result['nodes']:
                    continue
                if job['phase'] == 'waiting':
                    if job['list'].service_is_ready() and job['get'].service_is_ready():
                        job['future'] = job['list'].call_async(ListParameters.Request(depth=0))
                        job['phase'] = 'listing'
                elif job['future'].done():
                    reply = job['future'].result()
                    if job['phase'] == 'listing':
                        job['names'] = reply.result.names
                        job['future'] = job['get'].call_async(GetParameters.Request(names=job['names']))
                        job['phase'] = 'getting'
                    else:
                        if len(reply.values) != len(job['names']):
                            raise RuntimeError(f'{name}: incomplete parameter response')
                        result['nodes'][name] = {
                            'received_steady_ns': time.monotonic_ns(),
                            'values': dict(zip(job['names'], map(parameter_value, reply.values))),
                            'types': dict(zip(job['names'], [v.type for v in reply.values]))}
        result['missing_nodes'] = sorted(set(jobs) - set(result['nodes']))
        result['complete'] = not result['missing_nodes']
    except BaseException as error:
        result.update(complete=False, error=str(error))
    finally:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        with args.output.open('x') as stream:
            json.dump(result, stream, indent=2)
            stream.write('\n')
        node.destroy_node()
        rclpy.shutdown()
    print(json.dumps({'complete': result['complete'], 'nodes': list(result['nodes']),
                      'missing': result.get('missing_nodes'), 'output': str(args.output)}), flush=True)
    return 0 if result['complete'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
