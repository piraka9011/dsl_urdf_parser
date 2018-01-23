#!/usr/bin/env python

import roslib; roslib.load_manifest('urdfdom_py')
import rospy
from os.path import realpath, dirname
from urdf_parser_py.urdf import URDF

robot = URDF.from_xml_file(dirname(realpath(__file__)) + '/sawyer.urdf')
r = URDF()
print robot.joint_map
print r