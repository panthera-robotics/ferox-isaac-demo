import math
from pathlib import Path
import sys
import numpy as np
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from urdf_kinematics import UrdfKinematics


def model(tmp_path,joints,links=('base','arm','tip')):
 p=tmp_path/'tree.urdf';p.write_text('<robot name="test">'+''.join(f'<link name="{n}"/>' for n in links)+joints+'</robot>');return UrdfKinematics(p)


def joint(name,parent,child,kind='fixed',origin='0 0 0',rpy='0 0 0',axis='0 0 1',extra=''):
 return f'<joint name="{name}" type="{kind}"><parent link="{parent}"/><child link="{child}"/><origin xyz="{origin}" rpy="{rpy}"/><axis xyz="{axis}"/>{extra}</joint>'


def test_rotating_offset_is_applied_in_parent_frame(tmp_path):
 k=model(tmp_path,joint('yaw','base','arm','revolute','1 0 0',f'0 0 {math.pi/2}')+joint('tip_fixed','arm','tip',origin='1 0 0'))
 assert np.allclose(k.transforms({'yaw':0})['tip'][:3,3],[1,1,0],atol=1e-12)
 assert np.allclose(k.transforms({'yaw':math.pi/2})['tip'][:3,3],[0,0,0],atol=1e-12)


def test_urdf_roll_pitch_order(tmp_path):
 k=model(tmp_path,joint('pose','base','arm',origin='1 2 3',rpy=f'{math.pi/2} {math.pi/2} 0')+joint('tip_fixed','arm','tip',origin='.1 .2 .3'))
 assert np.allclose(k.transforms({})['tip'][:3,3],[1.2,1.7,2.9],atol=1e-12)


def test_mimic_is_resolved_from_named_parent(tmp_path):
 k=model(tmp_path,joint('root','base','arm','revolute')+joint('coupled','arm','tip','revolute',extra='<mimic joint="root" multiplier="2" offset=".1"/>'))
 r=k.transforms({'root':.2})['tip'][:3,:3]
 assert np.allclose(r@np.array([1,0,0]),[math.cos(.7),math.sin(.7),0],atol=1e-12)


def test_prismatic_axis_is_normalized(tmp_path):
 k=model(tmp_path,joint('slide','base','arm','prismatic',axis='2 0 0')+joint('fixed','arm','tip'))
 assert np.allclose(k.transforms({'slide':.25})['tip'][:3,3],[.25,0,0])


@pytest.mark.parametrize('positions',[{'unknown':0},{'yaw':float('nan')},{'yaw':True}])
def test_rejects_invalid_named_state(tmp_path,positions):
 k=model(tmp_path,joint('yaw','base','arm','revolute')+joint('fixed','arm','tip'))
 with pytest.raises(ValueError):k.transforms(positions)


def test_rejects_zero_prismatic_axis(tmp_path):
 k=model(tmp_path,joint('slide','base','arm','prismatic',axis='0 0 0')+joint('fixed','arm','tip'))
 with pytest.raises(ValueError):k.transforms({'slide':.2})
