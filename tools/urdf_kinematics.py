"""Independent named URDF forward kinematics for simulator frame comparisons."""
import math
from pathlib import Path
import xml.etree.ElementTree as ET


def _numbers(text,count):
    values=tuple(float(v) for v in text.split())
    if len(values)!=count or not all(math.isfinite(v) for v in values):raise ValueError('Invalid URDF vector')
    return values


class UrdfKinematics:
    def __init__(self,path):
        root=ET.parse(Path(path)).getroot()
        self.links={l.get('name') for l in root.findall('link')}
        if len(self.links)!=len(root.findall('link')):raise ValueError('Duplicate link name')
        self.joints={j.get('name'):j for j in root.findall('joint')}
        if len(self.joints)!=len(root.findall('joint')):raise ValueError('Duplicate joint name')
        self.children={}
        for name,joint in self.joints.items():
            parent=joint.find('parent').get('link');child=joint.find('child').get('link')
            if parent not in self.links or child not in self.links or child in self.children:raise ValueError('Invalid URDF tree')
            self.children[child]=(parent,name)
        roots=self.links-set(self.children)
        if len(roots)!=1:raise ValueError('Expected one root')
        self.root=roots.pop()

    def transforms(self,positions,root_transform=None):
        import numpy as np
        if not isinstance(positions,dict) or set(positions)-set(self.joints):raise ValueError('Unknown joint positions')
        if any(isinstance(v,bool) or not math.isfinite(float(v)) for v in positions.values()):raise ValueError('Invalid joint position')
        origin=np.eye(4) if root_transform is None else np.asarray(root_transform,dtype=float)
        if origin.shape!=(4,4) or not np.isfinite(origin).all():raise ValueError('Invalid root transform')
        result={self.root:origin.copy()};active=set()
        def rotation(axis,angle):
            a=np.asarray(axis,dtype=float);norm=np.linalg.norm(a)
            if norm<1e-12:raise ValueError('Zero joint axis')
            x,y,z=a/norm;cross=np.array([[0,-z,y],[z,0,-x],[-y,x,0]])
            return np.eye(3)+math.sin(angle)*cross+(1-math.cos(angle))*(cross@cross)
        def position(name,seen=None):
            if name in positions:return float(positions[name])
            seen=set() if seen is None else seen
            if name in seen:raise ValueError('Mimic cycle')
            seen.add(name);m=self.joints[name].find('mimic')
            return 0. if m is None else float(m.get('multiplier',1))*position(m.get('joint'),seen)+float(m.get('offset',0))
        def visit(link):
            if link in result:return result[link]
            if link in active:raise ValueError('Kinematic cycle')
            active.add(link);parent,name=self.children[link];joint=self.joints[name]
            origin=joint.find('origin');xyz=(0.,0.,0.);rpy=(0.,0.,0.)
            if origin is not None:
                xyz=_numbers(origin.get('xyz','0 0 0'),3);rpy=_numbers(origin.get('rpy','0 0 0'),3)
            t=np.eye(4);t[:3,3]=xyz
            t[:3,:3]=rotation((0,0,1),rpy[2])@rotation((0,1,0),rpy[1])@rotation((1,0,0),rpy[0])
            kind=joint.get('type');motion=np.eye(4)
            if kind in {'revolute','continuous','prismatic'}:
                element=joint.find('axis');axis=_numbers(element.get('xyz') if element is not None else '1 0 0',3)
                q=position(name)
                if kind=='prismatic':
                    a=np.asarray(axis);norm=np.linalg.norm(a)
                    if norm<1e-12:raise ValueError('Zero joint axis')
                    motion[:3,3]=a/norm*q
                else:motion[:3,:3]=rotation(axis,q)
            elif kind!='fixed':raise ValueError('Unsupported joint type '+kind)
            result[link]=visit(parent)@t@motion;active.remove(link);return result[link]
        for link in self.links:visit(link)
        return result
