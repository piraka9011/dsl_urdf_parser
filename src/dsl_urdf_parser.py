#!/usr/bin/env python

"""A URDF to .kidsl/.dtdsl parser for the RobCoGen package.
Maintained by Anas Abou Allaban - abouallaban.a@husky.neu.edu"""

import roslib; roslib.load_manifest('urdfdom_py')
import rospy
from os.path import realpath, dirname
from urdf_parser_py.urdf import URDF

class Inertia:
    """This class holds all the inertia variables associated with a link.
    Behavior is that of a list.
    Input: Ix, Iy, Iz. Ixy, Ixz, Iyz"""
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


class Frame:
    """Defines a ref_frame that will be used to express inertia w/r to CoM"""
    def __init__(self, name='frameX', translation=(0.0, 0.0, 0.0), rotation=(0.0, 0.0, 0.0)):
        self.translation = translation
        self.rotation = rotation
        self.name = name

    def __str__(self):
        return '\tframes {{\n' \
               '\t\t{} {{\n' \
               '\t\t\ttranslation = {}\n' \
               '\t\t\trotation = {}\n' \
               '\t\t}}\n' \
               '\t}}\n'.format(self.name, self.translation, self.rotation)


class InertiaProperties:
    """Stores inertial related parameters.
    URDF: Inertia tensor expressed with respect to the CoM.
    KIDSL: CoM and inertia tensor are expressed with respect to the link-frame.
    Thus, we always need to have a ref_frame with values of CoM"""
    def __init__(self, mass=1.0, com=(0.0, 0.0, 0.0), inertia=Inertia(), ref_frame_name='frameX'):
        self.mass = mass
        self.com = com
        self.inertia = inertia
        self.ref_frame_name = ref_frame_name

    def __str__(self):
        return '\tinertia_properties {{\n' \
               '\t\tmass = {}\n' \
               '\t\tCoM = {}\n' \
               '\t\t{}\n' \
               '\t\tref_frame = {}\n' \
               '\t}}'.format(self.mass, self.com, self.inertia, self.ref_frame_name).replace("'", "")


class Children:
    """Creates a link-joint association"""
    def __init__(self, parent='linkX', child='jointX', no_children=False):
        self.parent = parent
        self.child = child
        self.no_children = no_children

    def __str__(self):
        if self.no_children:
            return '\tchildren {}\n'
        else:
            return '\tchildren {{\n' \
                   '\t\t{} via {}\n' \
                   '\t}}'.format(self.parent, self.child)


class Link:
    """Defines a link with ID, inertia properties, children, and frames"""
    def __init__(self, link_name='linkX', ind=1, frame=Frame(),
                 inertia_properties=InertiaProperties(), children=Children(), is_base=False):
        self.link_name = link_name
        self.ind = ind
        self.inertia_properties = inertia_properties
        self.children = children
        self.isBase = is_base
        self.frame = frame

    def __str__(self):
        if self.isBase:
            return 'RobotBase {} {{\n' \
                   '{}\n\n{}\n}}\n\n'.format(self.link_name, self.inertia_properties,
                                             self.children)
        else:
            return 'link {} {{\n' \
                   '\tid = {}\n' \
                   '{}\n\n' \
                   '{}\n\n' \
                   '{}\n\n}}\n\n'.format(self.link_name, self.ind, self.inertia_properties,
                                         self.children, self.frame)


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
               '\t}}\n}}\n'.format(self.type, self.name, self.translation, self.rotation)


class Robot:
    def __init__(self, name='Robo', base=Link(is_base=True), links=Link(), joints=Joint()):
        self.name = name
        self.joints = [joints]
        self.base = base
        self.links = [base]
        self.links.append(links)
        self.id = 1

    def set_name(self, name):
        self.name = name

    def set_base(self, base):
        self.base = base

    def add_joint(self, joint):
        self.joints.append(joint)

    def add_link(self, link):
        self.id += 1
        link.ind = self.id
        self.links.append(link)

    def __str__(self):
        link_string = ''
        joint_string = ''
        for link in self.links:
            link_string += str(link)
        for joint in self.joints:
            joint_string += str(joint)
        return 'Robot {} {{\n\n' \
               '{}{}\n}}\n'.format(self.name, link_string, joint_string)


class DslUrdfParser:
    def __init__(self, filename='robot.urdf'):
        self.filename = filename
        self.urdf = URDF.from_xml_file(dirname(realpath(__file__)) + '/sawyer.urdf')
        self.dsl = Robot()

    def set_robot_name(self):
        self.dsl.set_name(self.urdf.name)

    def set_robot_base(self):
        self.dsl.base = Link(link_name='base', is_base=True)

    def get_links(self):
        for link in self.urdf.links:
            new_link = Link()
            new_link.link_name = link.name
            if link.inertial is not None:
                # Set inertia values
                i = link.inertial.inertia
                new_link.inertia_properties.inertia = \
                    Inertia(i.ixx, i.iyy, i.izz, i.ixy, i.ixz, i.iyz)
                # Set mass
                new_link.inertia_properties.mass = link.inertial.mass
                # Set CoM
                t = link.inertial.origin.position
                r = link.inertial.origin.rotation
                new_link.inertia_properties.com = (t[0], t[1], t[2])
                # Set reference frame
                new_link.frame.name = 'ref_' + str(link.name)

    def parse(self):
        self.set_robot_base()
        self.get_links()


if __name__ == '__main__':
    urdf_robot = URDF.from_xml_file(dirname(realpath(__file__)) + '/sawyer.urdf')
    dsl_robot = DslUrdfParser()
    dsl_robot.parse()
    print 'test'
    print 'parse'
