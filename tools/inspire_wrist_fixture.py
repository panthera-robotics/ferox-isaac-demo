"""Declared six-axis laboratory wrist fixture around an unchanged donor hand.

This is a supported diagnostic fixture, never a robot wrist or standing model.
The six fixture axes are dynamically driven; no hand or object pose is replayed.
"""
from copy import deepcopy
import hashlib
from pathlib import Path
import xml.etree.ElementTree as ET


AXES = (
    ('x', 'prismatic', '1 0 0', -.05, .05, 20., .15, 5000., 50.),
    ('y', 'prismatic', '0 1 0', -.05, .05, 20., .15, 5000., 50.),
    ('z', 'prismatic', '0 0 1', -.05, .05, 20., .15, 5000., 50.),
    ('roll', 'revolute', '1 0 0', -.35, .35, 5., .5, 50., 2.),
    ('pitch', 'revolute', '0 1 0', -.35, .35, 5., .5, 50., 2.),
    ('yaw', 'revolute', '0 0 1', -.35, .35, 5., .5, 50., 2.),
)


def write_wrist_fixture(source, output):
    source, output = Path(source).resolve(), Path(output).resolve()
    if output.exists():
        raise FileExistsError(output)
    hand = ET.parse(source).getroot()
    links = {link.get('name'): link for link in hand.findall('link')}
    children = {joint.find('child').get('link') for joint in hand.findall('joint')}
    roots = set(links) - children
    if roots != {'right_base_link'}:
        raise ValueError('Expected the pinned standalone right-hand bench root')
    if any(name.startswith('fixture_') for name in links):
        raise ValueError('Fixture namespace already occupied')
    unchanged_links = {name: ET.tostring(link) for name, link in links.items()}
    original_joints = {j.get('name'): ET.tostring(j) for j in hand.findall('joint')}
    result = deepcopy(hand)
    for mesh in result.iter('mesh'):
        filename = Path(mesh.get('filename'))
        if not filename.is_absolute():
            mesh.set('filename', str((source.parent / filename).resolve()))
    parent = 'fixture_anchor'
    joint_facts = []
    for i, (axis, kind, direction, lower, upper, effort, velocity, kp, kd) in enumerate(AXES):
        # Each upstream carriage is a 20 g declared fixture link without a
        # collision shape. The original hand links/inertias are never rescaled.
        link = ET.SubElement(result, 'link', name=parent)
        inertial = ET.SubElement(link, 'inertial')
        ET.SubElement(inertial, 'origin', xyz='0 0 0', rpy='0 0 0')
        ET.SubElement(inertial, 'mass', value='.02')
        ET.SubElement(inertial, 'inertia', ixx='.00001', iyy='.00001', izz='.00001',
                      ixy='0', ixz='0', iyz='0')
        child = 'right_base_link' if i == len(AXES)-1 else 'fixture_' + axis + '_carriage'
        name = 'fixture_' + axis + '_joint'
        joint = ET.SubElement(result, 'joint', name=name, type=kind)
        ET.SubElement(joint, 'origin', xyz='0 0 0', rpy='0 0 0')
        ET.SubElement(joint, 'parent', link=parent)
        ET.SubElement(joint, 'child', link=child)
        ET.SubElement(joint, 'axis', xyz=direction)
        ET.SubElement(joint, 'limit', lower=str(lower), upper=str(upper),
                      effort=str(effort), velocity=str(velocity))
        joint_facts.append({'name': name, 'type': kind, 'axis': direction,
                            'position_unit': 'm' if kind == 'prismatic' else 'rad',
                            'effort_unit': 'N' if kind == 'prismatic' else 'Nm',
                            'limits': [lower, upper], 'effort_limit': effort,
                            'velocity_limit': velocity, 'kp': kp, 'kd': kd})
        parent = child
    # Compare original data before mesh-path resolution, which is the only
    # serialized difference inside pre-existing links.
    for name, original in unchanged_links.items():
        actual = deepcopy(next(link for link in result.findall('link') if link.get('name') == name))
        for mesh, old_mesh in zip(actual.iter('mesh'), links[name].iter('mesh')):
            mesh.set('filename', old_mesh.get('filename'))
        assert ET.tostring(actual) == original
    assert all(ET.tostring(next(j for j in result.findall('joint') if j.get('name') == name)) == blob
               for name, blob in original_joints.items())
    output.parent.mkdir(parents=True, exist_ok=True)
    ET.indent(result)
    ET.ElementTree(result).write(output, encoding='utf-8', xml_declaration=True)
    return {'fixture_id': 'supported_xyz_rpy_wrist_v1', 'source_sha256': hashlib.sha256(source.read_bytes()).hexdigest(),
            'generated_sha256': hashlib.sha256(output.read_bytes()).hexdigest(),
            'source_hand_geometry_mass_inertia_and_joints_preserved': True,
            'fixture_added_mass_kg': .12, 'fixture_link_collision_shapes': False,
            'kinematic_hand_or_object_pose_overwrite': False, 'anchor_fixed_to_world': True,
            'robot_wrist_qualification': False, 'standing_qualification': False,
            'parameter_provenance': 'declared_virtual_laboratory_fixture_not_measured_hardware',
            'axes': joint_facts}
