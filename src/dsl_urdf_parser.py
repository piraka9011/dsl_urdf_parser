#!/usr/bin/env python

import roslib; roslib.load_manifest('urdfdom_py')
from os.path import realpath, dirname
from urdf_parser_py.urdf import URDF


class Inertia:
    def __init__(self, Ix=0, Iy=0, Iz=0, Ixy=0, Ixz=0, Iyz=0):
        self.Ix = Ix
        self.Iy = Iy
        self.Iz = Iz
        self.Ixy = Ixy
        self.Ixz = Ixz
        self.Iyz = Iyz
        self._data = [self.Ix, self.Iy, self.Iz, self.Ixy, self.Ixz, self.Iyz]

    def __iter__(self):
        for i in self._data:
            yield i

    def __str__(self):
        return 'Ix={}, Iy={}, Iz={}, Ixy={}, Ixz={}, Iyz={}'.format(
            self.Ix, self.Iy, self.Iz, self.Ixy, self.Ixz, self.Iyz)


class InertiaProperties:
    def __init__(self, mass=1.0, com=(0, 0, 0), inertia=Inertia(0, 0, 0, 0, 0, 0)):
        self.mass = mass
        self.com = com
        self.inertia = inertia

    def __str__(self):
        return '\tinertia_properties {\n' \
               '\t\tmass = {}\n' \
               '\t\tCoM = {}\n' \
               '\t\t{}\n' \
               '\t}}'.format(self.mass, self.com, self.inertia)


class Children:
    def __init__(self, parent='linkX', child='jointX'):
        self.parent = parent
        self.child = child

    def __str__(self):
        return '\tchildren {{\n' \
               '\t\t{} via {}\n' \
               '\t}}'


class Link:
    def __init__(self, link_name='linkX', ind=1,
                 inertia_properties=InertiaProperties(), children=Children(), is_base=False):
        self.link_name = link_name
        self.ind = ind
        self.inertia_properties = inertia_properties
        self.children = children
        self.isBase = is_base

    def __str__(self):
        if self.isBase:
            return 'RobotBase {} {{\n' \
                   '{}\n\n{}\n}}'.format(self.link_name, self.inertia_properties,
                                         self.children)
        else:
            return 'link {} {{\n' \
                   '\tid = {}\n' \
                   '{}\n\n{}\n}}'.format(self.link_name, self.ind,
                                         self.inertia_properties, self.children)


class Joint:
    def __init__(self, name='jX', joint_type='p',
                 translation=(0.0, 0.0, 0.0), rotation=(0.0, 0.0, 0.0)):
        self.name = name
        self.type = joint_type
        self.translation = translation
        self.rotation = rotation

    def __str__(self):
        return '{}_joint {} {{\n' \
               '\tref_frame {{\n' \
               '\t\ttranslation = {}\n' \
               '\t\trotation = {}\n' \
               '\t}}\n}}'


class Robot:
    def __init__(self, name='Robo', base=Link(is_base=True), links=[Link()], joints=[Joint()]):
        self.name = name
        self.links = links
        self.joints = joints
        self.base = base

    def add_joint(self, joint):
        self.joints.append(joint)

    def add_link(self, link):
        self.links.append(link)

    def __str__(self):
        return 'Robot {} {{\n' \
               ''.format(self.name)


robot = URDF.from_xml_file(dirname(realpath(__file__)) + '/sawyer.urdf')
r = URDF()
print robot.joint_map
print r