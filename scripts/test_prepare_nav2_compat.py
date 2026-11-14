import io
from pathlib import Path
import tarfile
import tempfile
import unittest

from prepare_nav2_compat import unpack_regular_archive, prepare, validate_map_points, tighten_goal_checker


def archive(entries):
    data = io.BytesIO()
    with tarfile.open(fileobj=data, mode='w') as stream:
        for name, kind in entries:
            member = tarfile.TarInfo(name)
            member.type = kind
            if kind == tarfile.REGTYPE:
                member.size = 3
                stream.addfile(member, io.BytesIO(b'abc'))
            else:
                member.linkname = '/tmp/outside'
                stream.addfile(member)
    return data.getvalue()


class Nav2ExportTest(unittest.TestCase):
    def test_strict_variant_only_tightens_run_local_checker(self):
        from copy import deepcopy
        original = {'controller_server': {'ros__parameters': {
            'goal_checker_plugins': ['general'],
            'general': {'xy_goal_tolerance': 0.25, 'yaw_goal_tolerance': 6.28}}}}
        runtime = deepcopy(original)
        result = tighten_goal_checker(runtime, 0.05)
        self.assertEqual(result['original_xy_m'], 0.25)
        self.assertEqual(original['controller_server']['ros__parameters']['general']['xy_goal_tolerance'], 0.25)
        self.assertEqual(runtime['controller_server']['ros__parameters']['general'],
                         {'xy_goal_tolerance': 0.05, 'yaw_goal_tolerance': 6.28})
        for invalid in (0.0, -0.1, 0.09, float('nan'), float('inf')):
            with self.assertRaises(ValueError):
                tighten_goal_checker(deepcopy(original), invalid)
        runtime['controller_server']['ros__parameters']['general']['xy_goal_tolerance'] = 0.02
        with self.assertRaises(ValueError):
            tighten_goal_checker(runtime, 0.05)

    def test_traversal_links_and_foreign_packages_rejected_before_writes(self):
        for name, kind in [('pkg/../../escape', tarfile.REGTYPE),
                           ('/absolute', tarfile.REGTYPE),
                           ('pkg/link', tarfile.SYMTYPE),
                           ('pkg/hardlink', tarfile.LNKTYPE),
                           ('production/file', tarfile.REGTYPE)]:
            with self.subTest(name=name), tempfile.TemporaryDirectory() as directory:
                dest = Path(directory) / 'new'
                with self.assertRaises(ValueError):
                    unpack_regular_archive(archive([('pkg/valid', tarfile.REGTYPE),
                                                     (name, kind)]), dest, {'pkg'})
                self.assertFalse(dest.exists())

    def test_existing_file_kept_and_regular_export_hashed(self):
        with tempfile.TemporaryDirectory() as directory:
            dest = Path(directory) / 'src'
            data = archive([('pkg/file', tarfile.REGTYPE)])
            hashes = unpack_regular_archive(data, dest, {'pkg'})
            self.assertEqual(len(hashes['pkg/file']), 64)
            with self.assertRaises(ValueError):
                unpack_regular_archive(data, dest, {'pkg'})
            self.assertEqual((dest / 'pkg/file').read_bytes(), b'abc')

    def test_production_tree_and_existing_output_refused_before_git(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            for output in [root, root / 'src' / 'compat']:
                with self.assertRaises(ValueError):
                    prepare(root, output)

    def test_old_negative_start_rejected_by_new_positive_origin_map(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / 'map.pgm').write_bytes(b'P5\n2 2\n255\n' + bytes([255, 255, 255, 0]))
            path = root / 'map.yaml'
            path.write_text('image: map.pgm\nresolution: 1\norigin: [0, 0, 0]\nfree_thresh: 0.25\n')
            with self.assertRaisesRegex(ValueError, 'outside'):
                validate_map_points(path, [(-10.66, 1.47)])
            with self.assertRaisesRegex(ValueError, 'not known-free'):
                validate_map_points(path, [(1.5, 0.5)])
            self.assertEqual(validate_map_points(path, [(0.5, 1.5)])[0]['cell'], [0, 1])


if __name__ == '__main__':
    unittest.main()
