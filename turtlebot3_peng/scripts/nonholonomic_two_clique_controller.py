#!/usr/bin/env python
import rospy
import numpy
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Twist

def relposition_sub():

    rospy.init_node('nonholonomic_two_clique_controller',anonymous=True)

    for i in range(4):
        rospy.Subscriber("tb3_%i/rel_polar_vector" % i, PoseArray, callback,callback_args=i)
    rospy.spin()


def callback(data,tb3_number):
    n = 3
    d = 2#dimension of the space
    global Y, Z, x_star, y_star
    # # desired position(triangle)
    # x_star = 0.8*numpy.array([0,-1,1,0])# x axis
    # y_star = 0.8*numpy.array([1, 0,0,0])# y axis

    ##desired position(rectangle)
    x_star = 0.8*numpy.array([-1,1,1,-1])# x axis
    y_star = 0.8*numpy.array([1,1,-1,-1])# y axis

    Y = numpy.zeros(shape=(d,n))
    #print(Y)
    Z = numpy.zeros(shape=(d,n))

    u = 0 #initialize the control input for velocity
    s = numpy.zeros(shape=(2,2)) #initialize the control input for angular velocity
    print('Information from %s' % tb3_number)

    # #agent 1 or 3
    if tb3_number==0 or tb3_number==2:
    #clique1---------------------------------start
        clique = numpy.array([0,1,2])#clique 1
        Y,Z = computation_of_Y_and_Z(data, clique) #compute matrix Y and Z
        ave_y=numpy.sum(Y, axis=1)/n #compute the average of matrix Y
        ave_y=ave_y.reshape(-1,1) #transpose of ave_y
        ave_z=numpy.sum(Z, axis=1)/n
        ave_z=ave_z.reshape(-1,1)
        #print(ave_y)
        ZZ=numpy.mat(Z-ave_z*numpy.array([1,1,1]))
        YY=numpy.mat(Y-ave_y*numpy.array([1,1,1]))
        A=ZZ.dot(numpy.transpose(YY))

        U,sigma,V_t=numpy.linalg.svd(A)
        # print(U)
        # print(V_t)

        V=numpy.transpose(V_t)
        detuv=numpy.linalg.det(U.dot(V))
        #print(detuv)
        D=numpy.diag([1,detuv])
        #print(D)
        R=V.dot(D).dot(numpy.transpose(U))
        phi1=ave_y+R.dot((numpy.array([[x_star[tb3_number]],[y_star[tb3_number]]])-ave_z))
        #print(phi1)
    #clique1---------------------------------end
    #clique2---------------------------------start
        clique = numpy.array([0,2,3])#clique 1
        Y,Z = computation_of_Y_and_Z(data, clique) #compute matrix Y and Z
        ave_y=numpy.sum(Y, axis=1)/n #compute the average of matrix Y
        ave_y=ave_y.reshape(-1,1) #transpose of ave_y
        ave_z=numpy.sum(Z, axis=1)/n
        ave_z=ave_z.reshape(-1,1)
        #print(ave_y)
        ZZ=numpy.mat(Z-ave_z*numpy.array([1,1,1]))
        YY=numpy.mat(Y-ave_y*numpy.array([1,1,1]))
        A=ZZ.dot(numpy.transpose(YY))

        U,sigma,V_t=numpy.linalg.svd(A)#svd
        V=numpy.transpose(V_t)
        detuv=numpy.linalg.det(U.dot(V))
        #print(detuv)
        D=numpy.diag([1,detuv])
        #print(D)
        R=V.dot(D).dot(numpy.transpose(U))#rotation matrix
        phi2=ave_y+R.dot((numpy.array([[x_star[tb3_number]],[y_star[tb3_number]]])-ave_z))
        #print(phi2)
        phi=phi1+phi2
    #clique2---------------------------------end
        ss=(numpy.identity(2)-numpy.array([[1],[0]])*numpy.array([1,0])).dot(phi)*(numpy.array([1,0]))
        s=ss-numpy.transpose(ss)
        v1=numpy.array([1,0])#constant vector b
        phi=numpy.ravel(phi)#transpose of vector phi
        u=v1.dot(phi)#input of velocity
        s=numpy.ravel(s)#in order to get the value of the matrix s, here change the dimension of the matrix s to an array
        s_input=s[1]#input of angular velocity


    if tb3_number==1: #agent 2
        clique = numpy.array([0,1,2])#clique 1
        Y,Z = computation_of_Y_and_Z(data, clique) #compute matrix Y and Z
        ave_y=numpy.sum(Y, axis=1)/n #compute the average of matrix Y
        ave_y=ave_y.reshape(-1,1) #transpose of ave_y
        ave_z=numpy.sum(Z, axis=1)/n
        ave_z=ave_z.reshape(-1,1)
        #print(ave_y)
        ZZ=numpy.mat(Z-ave_z*numpy.array([1,1,1]))
        YY=numpy.mat(Y-ave_y*numpy.array([1,1,1]))
        A=ZZ.dot(numpy.transpose(YY))

        U,sigma,V_t=numpy.linalg.svd(A)
        # print(U)
        # print(V_t)

        V=numpy.transpose(V_t)
        detuv=numpy.linalg.det(U.dot(V))
        #print(detuv)
        D=numpy.diag([1,detuv])
        #print(D)
        R=V.dot(D).dot(numpy.transpose(U))
        phi=ave_y+R.dot((numpy.array([[x_star[tb3_number]],[y_star[tb3_number]]])-ave_z))
        #compute the control input------------------
        ss=(numpy.identity(2)-numpy.array([[1],[0]])*numpy.array([1,0])).dot(phi)*(numpy.array([1,0]))
        s=ss-numpy.transpose(ss)
        v1=numpy.array([1,0])#constant vector b
        phi=numpy.ravel(phi)#transpose of vector phi
        u=v1.dot(phi)#input of velocity
        s=numpy.ravel(s)#in order to get the value of the matrix s, here change the dimension of the matrix s to an array
        s_input=s[1]#input of angular velocity

    if tb3_number==3: #agent 4
        clique = numpy.array([0,2,3])#clique 2
        Y,Z = computation_of_Y_and_Z(data, clique) #compute matrix Y and Z
        ave_y=numpy.sum(Y, axis=1)/n #compute the average of matrix Y
        ave_y=ave_y.reshape(-1,1) #transpose of ave_y
        ave_z=numpy.sum(Z, axis=1)/n
        ave_z=ave_z.reshape(-1,1)
        #print(ave_y)
        ZZ=numpy.mat(Z-ave_z*numpy.array([1,1,1]))
        YY=numpy.mat(Y-ave_y*numpy.array([1,1,1]))
        A=ZZ.dot(numpy.transpose(YY))

        U,sigma,V_t=numpy.linalg.svd(A)
        V=numpy.transpose(V_t)
        detuv=numpy.linalg.det(U.dot(V))
        #print(detuv)
        D=numpy.diag([1,detuv])
        #print(D)
        R=V.dot(D).dot(numpy.transpose(U))
        phi=ave_y+R.dot((numpy.array([[x_star[tb3_number]],[y_star[tb3_number]]])-ave_z))
        #compute the control input------------------
        ss=(numpy.identity(2)-numpy.array([[1],[0]])*numpy.array([1,0])).dot(phi)*(numpy.array([1,0]))
        s=ss-numpy.transpose(ss)
        v1=numpy.array([1,0])#constant vector b
        phi=numpy.ravel(phi)#transpose of vector phi
        u=v1.dot(phi)#input of velocity
        s=numpy.ravel(s)#in order to get the value of the matrix s, here change the dimension of the matrix s to an array
        s_input=s[1]#input of angular velocity

    twist = Twist()
    twist.linear.x = u
    twist.linear.y = 0.0
    twist.linear.z = 0.0
    twist.angular.x = 0.0
    twist.angular.y = 0.0
    twist.angular.z = s_input

    #vel_pub = rospy.Publisher('tb3_%i/destination_component' % tb3_number, Twist, queue_size=10)
    vel_pub = rospy.Publisher('tb3_%i/cmd_vel' % tb3_number, Twist, queue_size=10)
    print(twist)

    vel_pub.publish(twist)

def computation_of_Y_and_Z(data, clique):
    k=0
    for j in clique:
        xrel = data.poses[j].position.x
        yrel = data.poses[j].position.y
        Y[0][k] = xrel
        Y[1][k] = yrel
        Z[0][k] = x_star[j]
        Z[1][k] = y_star[j]
        k=k+1
    return Y, Z

if __name__ == '__main__':
    try:
        relposition_sub()
    except rospy.ROSInterruptException: pass
