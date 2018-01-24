#!/usr/bin/env python

import roslib; roslib.load_manifest('urdfdom_py')
from os.path import realpath, dirname
from urdf_parser_py.urdf import URDF


class Robot:
    def __init__(self, name='Robo'):
        self.name = name

    def __str__(self):
        return 'Robot {} {{'.format(self.name)


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
        return 'mass = {}\nCoM = {}\n {}\n'.format(self.mass, self.com, self.inertia)


class Link:
    def __init__(self, link_name='linkX', id=1, inertia_properties = InertiaProperties()):
        self.link_name = link_name
        self.id = id
        self.inertia_properties = inertia_properties

    def __str__(self):
        pass


robot = URDF.from_xml_file(dirname(realpath(__file__)) + '/sawyer.urdf')
r = URDF()
print robot.joint_map
print r