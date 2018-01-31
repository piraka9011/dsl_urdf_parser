#!/usr/bin/env python

"""A URDF to .kidsl/.dtdsl parser for the RobCoGen package.
Maintained by Anas Abou Allaban - abouallaban.a@husky.neu.edu

Note: Default None arguments used to prevent mutable defaults
changing entire structure of kinematic tree."""

import roslib; roslib.load_manifest('urdfdom_py')
from urdf_parser_py.urdf import URDF
from os.path import realpath, dirname, expanduser


class Inertia:
    """This class holds all the inertia variables associated with a link.
    Behavior is that of a list.
    Input: Ix, Iy, Iz. Ixy, Ixz, Iyz"""
    def __init__(self, Ix=0.0, Iy=0.0, Iz=0.0, Ixy=0.0, Ixz=0.0, Iyz=0.0):
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
        return 'Ix={:.8f} Iy={:.8f} Iz={:.8f} Ixy={:.8f} Ixz={:.8f} Iyz={:.8f}'.format(
            self.Ix, self.Iy, self.Iz, self.Ixy, self.Ixz, self.Iyz)


class Frame:
    """Defines a ref_frame that will be used to express inertia w/r to CoM"""
    def __init__(self, name='frameX', translation=(0.0, 0.0, 0.0), rotation=(0.0, 0.0, 0.0)):
        self.translation = translation
        self.rotation = rotation
        self.name = name

    def __str__(self):
        translation = '({:.8f}, {:.8f}, {:.8f})'.format(*self.translation)
        rotation = '({:.8f}, {:.8f}, {:.8f})'.format(*self.rotation)
        return '\tframes {{\n' \
               '\t\t{} {{\n' \
               '\t\t\ttranslation = {}\n' \
               '\t\t\trotation = {}\n' \
               '\t\t}}\n' \
               '\t}}\n'.format(self.name, translation, rotation)


class InertiaProperties:
    """Stores inertial related parameters.
    URDF: Inertia tensor expressed with respect to the CoM.
    KIDSL: CoM and inertia tensor are expressed with respect to the link-frame.
    Thus, we always need to have a ref_frame with values of CoM"""
    def __init__(self, mass=1.0, com=(0.0, 0.0, 0.0), inertia=None, ref_frame_name='frameX'):
        self.mass = mass
        self.com = com
        if inertia is None: self.inertia = Inertia()
        self.ref_frame_name = ref_frame_name

    def __str__(self):
        return '\tinertia_properties {{\n' \
               '\t\tmass = {:.8f}\n' \
               '\t\tCoM = {}\n' \
               '\t\t{}\n' \
               '\t\tref_frame = {}\n' \
               '\t}}'.format(self.mass, self.com, self.inertia, self.ref_frame_name).replace("'", "")


class Children:
    """Creates a link-joint association"""
    def __init__(self, no_children=False):
        self.children = []
        self.joints = []
        self.no_children = no_children

    def __str__(self):
        if self.no_children:
            return '\tchildren {}\n'
        else:
            child_string = ''
            for i, x in enumerate(self.children):
                child_string += '\t\t{} via {}\n'.format(x, self.joints[i])
            return '\tchildren {{\n' \
                   '{}' \
                   '\t}}'.format(child_string)


class Link:
    """Defines a link with ID, inertia properties, children, and frames"""
    def __init__(self, link_name='linkX', ind=1, frame=None,
                 inertia_properties=None, children=None, is_base=False):
        self.link_name = link_name
        self.ind = ind
        if inertia_properties is None:
            self.inertia_properties = InertiaProperties()
        if children is None:
            self.children = Children()
        if frame is None:
            self.frame = Frame()
        self.isBase = is_base


    def set_ref_frame(self, name):
        self.frame.name = name
        self.inertia_properties.ref_frame_name = name

    def __str__(self):
        if self.isBase:
            return 'RobotBase {} {{\n' \
                   '{}\n\n' \
                   '{}\n\n' \
                   '{}}}\n\n'.format(self.link_name, self.inertia_properties,
                                     self.children, self.frame)
        else:
            return 'link {} {{\n' \
                   '\tid = {}\n' \
                   '{}\n\n' \
                   '{}\n\n' \
                   '{}}}\n\n'.format(self.link_name, self.ind, self.inertia_properties,
                                     self.children, self.frame)


class Joint:
    def __init__(self, name='jX', joint_type='p',
                 translation=(0.0, 0.0, 0.0), rotation=(0.0, 0.0, 0.0)):
        self.name = name
        self.type = joint_type
        self.translation = translation
        self.rotation = rotation

    def __str__(self):
        translation = '({:.8f}, {:.8f}, {:.8f})'.format(*self.translation)
        rotation = '({:.8f}, {:.8f}, {:.8f})'.format(*self.rotation)
        return '{}_joint {} {{\n' \
               '\tref_frame {{\n' \
               '\t\ttranslation = {}\n' \
               '\t\trotation = {}\n' \
               '\t}}\n}}\n\n'.format(self.type, self.name, translation, rotation)


class Robot:
    def __init__(self, name='Robo', base=None):
        self.name = name
        self.joints = []
        self.joints_map = {}
        if base is None:
            self.base = Link(is_base=True)
        self.links = [base]
        self.links_map = {self.base.link_name: self.base}
        self.id = 0

    def set_name(self, name):
        self.name = name

    def set_base(self, base=None):
        if base is None:
            self.links[0] = self.base
        else:
            self.links[0] = base

    def add_joint(self, joint):
        self.joints.append(joint)
        self.joints_map[joint.name] = joint

    def add_link(self, link):
        self.id += 1
        link.ind = self.id
        self.links.append(link)
        self.links_map[link.link_name] = link

    def add_child_joint(self, parent, child, joint):
        for link in self.links:
            if link.link_name == parent:
                link.children.children.append(child)
                link.children.joints.append(joint)
                break

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
    def __init__(self, name='robot', filename='robot.urdf'):
        self.urdf = URDF.from_xml_file(filename)
        self.dsl = Robot(name=name)

    def _parse_inertia(self, urdf_link):
        i = urdf_link.inertial.inertia
        return Inertia(i.ixx, i.iyy, i.izz, i.ixy, i.ixz, i.iyz)

    def _parse_link_translation(self, urdf_link):
        t = urdf_link.inertial.origin.position
        return (t[0], t[1], t[2])

    def _parse_link_rotation(self, urdf_link):
        r = urdf_link.inertial.origin.rotation
        return (r[0], r[1], r[2])

    def set_robot_name(self):
        self.dsl.set_name(self.urdf.name)

    def set_robot_base(self, name):
        """Set the robot's base (root of kinematic tree)
        :param name: Name of the base link in the URDF file"""
        urdf_base_link = self.urdf.link_map[name]
        self.dsl.base = Link(link_name=name, is_base=True)
        if urdf_base_link.inertial is not None:
            self.dsl.base.inertia_properties.inertia = self._parse_inertia(urdf_base_link)
            self.dsl.base.inertia_properties.mass = urdf_base_link.inertial.mass
            self.dsl.base.frame.rotation = self._parse_link_rotation(urdf_base_link)
            self.dsl.base.frame.translation = self._parse_link_translation(urdf_base_link)
        self.dsl.base.set_ref_frame('ref_' + name)
        self.dsl.set_base()

    def get_links(self):
        """Iterate through URDF links and create new DSL links"""
        for link in self.urdf.links:
            # Don't add the link if it has no properties
            if link.inertial is not None:
                # Create new link
                new_link = Link()
                new_link.link_name = link.name
                # Set inertia values
                new_link.inertia_properties.inertia = self._parse_inertia(link)
                # Set mass
                new_link.inertia_properties.mass = link.inertial.mass
                # Set CoM
                new_link.frame.rotation = self._parse_link_rotation(link)
                new_link.frame.translation = self._parse_link_translation(link)
                # Set reference frame
                new_link.set_ref_frame('ref_' + str(link.name))
                # Add the link
                self.dsl.add_link(new_link)

    def get_joints(self):
        """Iterate through URDF joints and create new DSL joints"""
        valid_joints = {'revolute': 'r', 'prismatic': 'p'}
        for joint in self.urdf.joints:
            if joint.type in valid_joints:
                # Create a new joint
                new_joint = Joint()
                # Set the name
                new_joint.name = joint.name
                # Set the type
                new_joint.type = valid_joints[joint.type]
                # Set the position wr to parent link
                t = joint.origin.xyz
                r = joint.origin.rpy
                new_joint.translation = (t[0], t[1], t[2])
                new_joint.rotation = (r[0], r[1], r[2])
                self.dsl.add_joint(new_joint)

    def get_children(self):
        """Iterate through kinematic joints and links and assign them parents and children"""
        for child_link, parent in self.urdf.parent_map.iteritems():
            parent_link = parent[1]
            joint = parent[0]
            if self.urdf.joint_map[joint].type == 'fixed':
                continue
            self.dsl.add_child_joint(parent_link, child_link, joint)

    def auto_parse(self):
        # Start
        self.set_robot_base('base')
        self.get_links()
        self.get_joints()
        self.get_children()

    def write_kindsl_file(self, name='robo', path='~/'):
        file_path = expanduser(path) + name + '.kindsl'
        with open(file_path, 'w') as kindsl_file:
            kindsl_file.write(str(self.dsl))


if __name__ == '__main__':
    file_name = dirname(realpath(__file__)) + '/sawyer.urdf'
    dsl_robot = DslUrdfParser('Sawyer', file_name)
    dsl_robot.auto_parse()
    dsl_robot.write_kindsl_file(name='sawyer')
    print dsl_robot.dsl

