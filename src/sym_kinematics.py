#!/usr/bin/python
# A file that gives me analytical Transforms and Jacobians
# Philip Long 2017
#
#
#
import roslib; roslib.load_manifest('urdfdom_py')
import rospy
import sympy
import time
from copy import deepcopy
from urdf_parser_py.urdf import URDF
import sys, getopt
import textwrap


# Read command line arguments
def readArguments(argv):
    root_name='torso'
    tip_name='rightPalm'
    try:
        opts, args = getopt.getopt(argv,"hr:t:",["root_name=","tip_name="])
        for opt, arg in opts:
            if opt == '-h':
		print textwrap.dedent("""\

		Usage: rosrun <package_name> sym_kinematics.py -r <root_name> -t <tip_name> \n 
		Description: This program obtains the analytic value for the jacobian
		and transformation matrix from the root joint to the tip joint for a robot
		description on the parameter server.This program supports prismatic and 
		revloute joints only. 
                The output are saved as functions in C++ (.cpp  .h matrix library ViSP) and 
		matlab .m. \n \n
		To Do: Add support for different joints, print out matrices in python format
		""")
                sys.exit()
            elif opt in ("-r", "--root_name"):
                root_name = arg
            elif opt in ("-t", "--tip_name"):
                tip_name = arg
    except getopt.GetoptError:
        print 'sym_kinematics.py -r <root_name> -t <tip_name> ,using default values'
        time.sleep(1)

    print 'Root name is ',root_name, "tip name is ",tip_name
    return root_name,tip_name


# simplify a symbolic matrix
def simpMatrix(M): #olivier's function thanks!!
    for i in xrange(M.rows):
        for j in xrange(M.cols):
            try:
                M[i,j] = sympy.simplify(M[i,j])
            except ValueError:
                print 'ValueError skipping simplification'
                pass
    return M

# skew symmetric matrix
def skew(u): # returns the skew operator
    assert(len(u)==3), "Vector should have 3 elements"
    return sympy.Matrix([[0,-u[2],u[1]],[u[2],0,-u[0]],[-u[1],u[0],0]])

# Return a transformation matrix from an and axis of rotation
def transMat(y,axis_type,output_type): # returns the homogenous transformation matrix
    # if its Rotational
    assert(axis_type.count(1)==1), "Joint should have one single axis"
    assert(len(axis_type)==3), "Joint should have three components"

    if(axis_type.index(1)==0):
            T=sympy.Matrix([[1.0, 0.0,0.0,0.0],[0.0,sympy.cos(y),-sympy.sin(y),0.0], [0.0,sympy.sin(y),sympy.cos(y),0.0],[0.0,0.0,0.0,1.0]])
    elif(axis_type.index(1)==1):
            T=sympy.Matrix([[sympy.cos(y),  0.0,      sympy.sin(y), 0.0], [0.0,       1.0,      0.0,      0.0], [-sympy.sin(y), 0.0,      sympy.cos(y), 0.0], [0.0,       0.0    ,  0.0,      1.0] ])
    elif(axis_type.index(1)==2):
            T=sympy.Matrix([[sympy.cos(y),-sympy.sin(y), 0.0, 0.0], [sympy.sin(y),sympy.cos(y), 0.0,0.0], [0.0, 0.0,1.0,0.0], [0.0, 0.0, 0.0,      1.0] ])
    # What to return
    if output_type in ['R','Rot','r','rot']:
        return T[0:3,0:3]
    else:
        return T

def transMatPrismatic(y,axis_type): # returns the homogenous transformation matrix
    # if its Rotational
    assert(axis_type.count(1)==1), "Joint should have one single axis"
    assert(len(axis_type)==3), "Joint should have three components"

    if(axis_type.index(1)==0):
            T=sympy.Matrix([[1.0, 0.0,0.0,y],[0.0,1.0,0.0,0.0],[0.0, 0.0,1.0,0.0],[0.0,0.0,0.0,1.0]])
    elif(axis_type.index(1)==1):
            T=sympy.Matrix([[1.0, 0.0,0.0,0.0],[0.0,1.0,0.0,y],[0.0, 0.0,1.0,0.0],[0.0,0.0,0.0,1.0]])
    elif(axis_type.index(1)==2):
            T=sympy.Matrix([[1.0, 0.0,0.0,0.0],[0.0,1.0,0.0,0.0],[0.0, 0.0,1.0,y],[0.0,0.0,0.0,1.0]])

    return T

# Converts RPY to Rotational cosine
def rpyToRot(u,output_type='T'):
    assert(len(u)==3), "Vector should have 3 elements"
    if output_type in ['R','Rot','r','rot']:
        return transMat(u[0],[1,0,0],'rot')*transMat(u[1],[0,1,0],'rot')*transMat(u[2],[0,0,1],'rot')
    else:
        return transMat(u[0],[1,0,0],'T')*transMat(u[1],[0,1,0],'T')*transMat(u[2],[0,0,1],'T')

# Translation vector to transformation matrix
def translationToTransform(t):
    T=sympy.Matrix([[1.0,0.0,0.0, t[0]], [0.0,1.0, 0.0,t[1]], [0.0, 0.0,1.0,t[2]], [0.0,0.0,0.0,1.0] ])
    return T

# Get kinematic information from a serial chain
def getKinematicInformation(robot,root_name,tip_name):

    # Get the deisred chain
    desired_chain=robot.get_chain(root=root_name,tip=tip_name,joints=True,links=False)

    axes=[]
    transforms=[]
    sigma=[]
    for i in reversed(desired_chain):
        print i
        for j in robot.joints:
            if(j.name==i):
                print "joint name =",j.name
                print "joint child ",j.child
                print "joint parent ",j.parent
                print "joint type",j.type
                print "origin xyz",j.origin.xyz
                print "origin rpy",j.origin.rpy

                if(j.type=='revolute' or j.type=='prismatic'):
                    sigma.append(j.type)
                else:
                    raise Exception("Sorry we only support revolute and prismatic at the moment :(")

                axes.append(j.axis) # convert numerical axis to string
                transforms.append(translationToTransform(j.origin.xyz)*rpyToRot(j.origin.rpy))
                print '================='

    # Put in ascending order
    axes.reverse()
    transforms.reverse()
    sigma.reverse()
    return (axes,transforms,sigma)

# Obtain all transforms from root to tip
def directGeometricModel(transform_dh,q,axes,sigma): # returns the transformation matrix from each root frame to tip
    T=[]
    # initialse w^T_w as identify
    Tpre=sympy.eye(4)

    for i in range(len(transform_dh)):
        # i-1^Transform_i= fixed_transform*transform due to joint motion
        if(sigma[i]=='revolute'):
            T.append(transform_dh[i]*transMat(q[i],axes[i],'T'))
        elif(sigma[i]=='prismatic'):
            print "Tpris=",sympy.pprint(transMatPrismatic(q[i],axes[i]))
            T.append(transform_dh[i]*transMatPrismatic(q[i],axes[i]))

        # w^Transform_i =w^Transform_i-1* i-1^Transform_i

        T[i]=Tpre*T[i]
        T[i]=simpMatrix(T[i])

        # w^Transform_i-1=w^Transform_i
        Tpre=T[i]
    return T

def allDirectKinematicModels(transforms_sym,axes,sigma):
    jacobians=[]
    while(transforms_sym):
        jacobians.append(directKinematicModel(transforms_sym,axes,sigma))
        transforms_sym.pop()
        axes.pop()
        sigma.pop()
    # put back in asceding order
    jacobians.reverse()
    return jacobians

def directKinematicModel(transforms_sym,axes,sigma):
    number_of_joints=len(transforms_sym)
    jacobian=sympy.zeros(6,number_of_joints) # initialise jacobians
    # obtain columns of jacobian matrix
    for k in range(number_of_joints):
        a=transforms_sym[k][0:3,axes[k].index(1)] # extract joint axis
        askew=skew(a) # find skew symmetric matrix
        if(sigma[k]=='revolute'):
            jac_k=askew*(transforms_sym[number_of_joints-1][0:3,3]-transforms_sym[k][0:3,3]) # find linear velocity effect at terminal joint
            jac_k=jac_k.col_join(a) # find angular velocity effect
        elif(sigma[k]=='prismatic'):
            jac_k=a;
            jac_k=jac_k.col_join(sympy.zeros(3,1))
            print "jac_k=",jac_k

        jacobian[:,k]=jac_k
    return jacobian

def write_mat(root_name,tip_name,t,prefix):
    to_write=""
    for j in range(t.shape[0]): # rows
        for k in range(t.shape[1]): # columns
            to_write+=prefix+root_name+"_"+tip_name+"("+str(j+1)+","+str(k+1)+")="+str(t[j,k])+";\n"
    return to_write


def write_cpp(root_name,tip_name,t,variable_type,prefix):
    cpp_variable=prefix+"_"+root_name+"_"+tip_name
    to_write=variable_type+" "+cpp_variable+";\n\n"
    for j in range(t.shape[0]): # rows
        for k in range(t.shape[1]): # columns
            to_write+=cpp_variable+"["+str(j)+"]["+str(k)+"]="+str(t[j,k])+";\n"
    to_write+=prefix+"="+  cpp_variable+";\n\n"
    return to_write


def write_transforms(desired_chain,joint_chain,transforms):
    root_name=desired_chain[0]
    tip_name=desired_chain[-1]
    file_name="Transforms_"+root_name+"_"+tip_name

    fcpp = open(file_name+".cpp", "w+")
    fmat = open(file_name+".m", "w+")

    mat_header="function T="+file_name+"(q) \n \n"
    mat_footer="T={"

    mat_joints=""
    cpp_joints=""
    cpp_write=""
    mat_write=""
    prefix="T_"
    # in matlab since we're not doing real time stuff we can group all transforms together
    for i in range(len(joint_chain)):
        mat_joints+=joint_chain[i]+"=q("+str(i+1)+");\n"
        cpp_joints+="double "+joint_chain[i]+"=q["+str(i)+"];\n"


    counter=1
    for t in transforms: # cycle through transforms
        cpp_header="void transform"+root_name+desired_chain[counter]+"(vpColVector q,vpHomogenousMatrix& T) \n { \n"

        #cpp_transform=write_transform(root_name,desired_chain[counter],t)
        cpp_transform=write_cpp(root_name,desired_chain[counter],t,"vpHomogenousMatrix ","T")

        cpp_write+=cpp_header+cpp_joints+cpp_transform+"}\n\n\n"

        mat_footer+=prefix+root_name+"_"+desired_chain[counter]+","
        mat_transform=write_mat(root_name,desired_chain[counter],t,prefix)
        mat_write+=mat_transform+"\n"

        counter+=1

    mat_footer=mat_footer[:-1]+"};"

    fmat.write(mat_header+mat_joints+mat_write+mat_footer)
    fcpp.write(cpp_write)

    fcpp.close()
    fmat.close()




def write_jacobians(desired_chain,joint_chain,jacobians):
    root_name=desired_chain[0]
    tip_name=desired_chain[-1]
    file_name="Jacobians_"+root_name+"_"+tip_name

    fcpp = open(file_name+".cpp", "w+")
    fmat = open(file_name+".m", "w+")

    mat_header="function J="+file_name+"(q) \n \n"
    mat_footer="J={"

    mat_joints=""
    cpp_joints=""
    cpp_write=""
    mat_write=""
    prefix="J_"
    # in matlab since we're not doing real time stuff we can group all transforms together
    for i in range(len(joint_chain)):
        mat_joints+=joint_chain[i]+"=q("+str(i+1)+");\n"
        cpp_joints+="double "+joint_chain[i]+"=q["+str(i)+"];\n"

    print "mat_joints=",mat_joints

    counter=1
    for j in jacobians: # cycle through transforms
        cpp_header="void jacobian"+root_name+desired_chain[counter]+"(vpColVector q,vpMatrix& J) \n { \n"

        cpp_jacobian=write_cpp(root_name,desired_chain[counter],j,"vpMatrix ","J")
        cpp_write+=cpp_header+cpp_joints+cpp_jacobian+"}\n\n\n"

        mat_footer+=prefix+root_name+"_"+desired_chain[counter]+","
        mat_jacobian=write_mat(root_name,desired_chain[counter],j,prefix)
        mat_write+=mat_jacobian+"\n"

        counter+=1

    mat_footer=mat_footer[:-1]+"};"

    fmat.write(mat_header+mat_joints+mat_write+mat_footer)
    fcpp.write(cpp_write)

    fcpp.close()
    fmat.close()


def main(argv):

    root_name,tip_name=readArguments(argv)
    robot = URDF.from_parameter_server()


    desired_joint_chain=robot.get_chain(root=root_name,tip=tip_name,joints=True,links=False)
    desired_link_chain=robot.get_chain(root=root_name,tip=tip_name,joints=False,links=True)
    print desired_joint_chain
    print desired_link_chain
    q=sympy.symbols(desired_joint_chain)

    axes,transforms_dh,sigma=getKinematicInformation(robot,root_name,tip_name)

    transforms_sym=directGeometricModel(transforms_dh,q,axes,sigma)
    # Need deepcopy as transforms_sym is modified by function
    jacobians_sym=allDirectKinematicModels(deepcopy(transforms_sym),deepcopy(axes),deepcopy(sigma))


    write_transforms(desired_link_chain,desired_joint_chain,transforms_sym)# write transforms to file
    write_jacobians(desired_link_chain,desired_joint_chain,jacobians_sym)# write transforms to file


    for i,j in zip(transforms_sym,jacobians_sym):
        print "T=",sympy.pprint(i)
        print "J=",sympy.pprint(j)


if __name__ == '__main__':
    main(sys.argv[1:])
