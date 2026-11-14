"""Static metrology fixture only; never used by a navigation acceptance launch."""
import hashlib
from pathlib import Path
import re
import xml.etree.ElementTree as ET


def make_stationary_fixture(source, target, start, roll=0.0, pitch=0.0):
    import mujoco
    from ats_mujoco_sim.spawn_pose import quaternion_from_rpy

    source, target = Path(source).resolve(), Path(target).resolve()
    original = source.read_bytes()
    model = mujoco.MjModel.from_xml_path(str(source))
    tree = ET.fromstring(re.sub(r'<!--.*?-->', '', original.decode(), flags=re.S))
    base = tree.find("worldbody/body[@name='base_link']")
    if base is None or base.find('freejoint') is None:
        raise ValueError('stationary fixture requires the existing free base_link')
    # The simulator resets this same free-joint pose from its normal CLI.
    # Setting qpos0 here lets MuJoCo derive the world-weld reference consistently.
    base.set('pos', ' '.join(map(str, start[:3])))
    base.set('quat', ' '.join(map(str, quaternion_from_rpy(roll, pitch, start[3]))))
    equality = tree.find('equality')
    if equality is None:
        equality = ET.SubElement(tree, 'equality')
    # Keep the fixed world as body1. Reversing the pair anchors at the distant
    # world origin in the moving body frame, creating an unnecessary lever arm.
    ET.SubElement(equality, 'weld', name='h_stationary_world_fixture',
                  body1='world', body2='base_link')
    statistic = tree.find('statistic')
    if statistic is None:
        statistic = ET.SubElement(tree, 'statistic')
    for key in ('meanmass', 'meaninertia', 'meansize', 'extent'):
        statistic.set(key, repr(float(getattr(model.stat, key))))
    statistic.set('center', ' '.join(map(str, model.stat.center)))
    for element in tree.iter():
        if 'file' in element.attrib:
            element.set('file', str((source.parent / element.get('file')).resolve()))
    target.parent.mkdir(parents=True, exist_ok=True)
    mesh_link = target.parent / 'meshes/mid360.obj'
    mesh_link.parent.mkdir(exist_ok=True)
    if not mesh_link.exists():
        mesh_link.symlink_to((source.parent / 'meshes/mid360.obj').resolve())
    ET.ElementTree(tree).write(target, encoding='unicode')
    return {'source_model': str(source), 'source_sha256': hashlib.sha256(original).hexdigest(),
            'fixture_model': str(target), 'fixture_sha256': hashlib.sha256(target.read_bytes()).hexdigest(),
            'constraint': 'world weld; solver-visible support, no post-step state reset',
            'start': list(start), 'roll': roll, 'pitch': pitch,
            'scope': 'fixed-body static scan metrology; not parking or navigation acceptance'}
