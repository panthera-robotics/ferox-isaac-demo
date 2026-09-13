import copy
import sys
from pathlib import Path
import numpy as np
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from rigid_inertia import source_properties,compare_properties


def source(tmp_path):
    p=tmp_path/'body.urdf'
    p.write_text('<robot name="test"><link name="body"><inertial><origin xyz="1 0 0" rpy="0 0 1.5707963267948966"/><mass value="2"/><inertia ixx="1" ixy="0" ixz="0" iyy="2" iyz="0" izz="3"/></inertial></link></robot>')
    return p


def test_inertia_origin_rotation_and_com_translation(tmp_path):
    t=np.eye(4);t[:3,3]=[0,2,0]
    result=source_properties(source(tmp_path),{'body':t})['body']
    assert result['mass_kg']==2
    np.testing.assert_allclose(result['com_runtime_link_m'],[1,2,0])
    # At COM: translating the frame must NOT add a parallel-axis term.
    np.testing.assert_allclose(result['inertia_at_com_runtime_link_kg_m2'],np.diag([2,1,3]),atol=1e-14)


def test_changed_body_orientation_rotates_com_and_tensor(tmp_path):
    t=np.array([[0,-1,0,0],[1,0,0,0],[0,0,1,0],[0,0,0,1]],dtype=float)
    result=source_properties(source(tmp_path),{'body':t})['body']
    np.testing.assert_allclose(result['com_runtime_link_m'],[0,1,0],atol=1e-14)
    np.testing.assert_allclose(result['inertia_at_com_runtime_link_kg_m2'],np.diag([1,2,3]),atol=1e-14)


@pytest.mark.parametrize('key,value',[('mass_kg',1.9),('com_runtime_link_m',[1.0001,0,0]),('inertia_at_com_runtime_link_kg_m2',np.diag([2,1,3.01]).tolist())])
def test_each_property_change_fails(tmp_path,key,value):
    want=source_properties(source(tmp_path),{'body':np.eye(4)});got=copy.deepcopy(want)
    assert all(compare_properties(want,got)['checks'].values())
    got['body'][key]=value
    assert not any(compare_properties(want,got)['checks'].values())


def test_nonfinite_and_wrong_names_rejected(tmp_path):
    want=source_properties(source(tmp_path),{'body':np.eye(4)})
    with pytest.raises(ValueError):compare_properties(want,{})
    got=copy.deepcopy(want);got['body']['mass_kg']=float('nan')
    with pytest.raises(ValueError):compare_properties(want,got)
