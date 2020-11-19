#!/usr/bin/env python
import rospy
import numpy
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Twist

def relposition_sub():
    #global d_star

    # number of units
    #n = 4

    # # goal position (right isosceles triangle)
    # x_star = [0,1,2,1]
    # y_star = [0,0,0,1]

    # # goal position (square)
    x_star = numpy.array([-1,1,1])# desired position
    y_star = numpy.array([1,1,-1])

    # goal position (senbun)
    # x_star = [0,1.5]
    #y_star = [0,0]


    # # goal position (triangle)
    # x_star = [0,1,1/2]
    # y_star = [0,0,numpy.sqrt(3)]


    # calculate relevant distance
    # d_star = []
    # for p in range(len(x_star)):
    #     rd = []
    #     for q in range(len(x_star)):
    #         rd.append(numpy.sqrt((x_star[q]-x_star[p])**2 + (y_star[q]-y_star[p])**2))
    #     d_star.append(rd)
    # print(d_star)


    rospy.init_node('nonholonomic_one_clique_controller',anonymous=True)

    for i in range(3):
        rospy.Subscriber("tb3_%i/rel_polar_vector" % i, PoseArray, callback,callback_args=i)
    rospy.spin()


def callback(data,tb3_number):
    n = 3
    d = 2
    # x_star = numpy.array([-1,1,1,-1])# desired position
    # y_star = numpy.array([1,1,-1,-1])
    x_star = numpy.array([-1,1,1])# desired position
    y_star = numpy.array([1,1,-1])

    Y = numpy.zeros(shape=(d,n))
    #print(Y)
    Z = numpy.zeros(shape=(d,n))

    u = 0 #initialize the control input for velocity
    s = numpy.zeros(shape=(2,2)) #initialize the control input for angular velocity
    print('Information from %s' % tb3_number)


    clique = numpy.array([0,1,2])#clique 1
    #compute matrix Y and Z------------------
    k=0
    for j in clique:
        xrel = data.poses[j].position.x
        yrel = data.poses[j].position.y
        Y[0][k] = xrel
        Y[1][k] = yrel
        Z[0][k] = x_star[j]
        Z[1][k] = y_star[j]
        k=k+1
    #compute matrix Y and Z------------------
    #print(Y)
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
    #print(phi)
    ss=(numpy.identity(2)-numpy.array([[1],[0]])*numpy.array([1,0])).dot(phi)*(numpy.array([1,0]))
    s=ss-numpy.transpose(ss)

    v1=numpy.array([1,0])
    phi=numpy.ravel(phi)#transpose of vector phi
    u=v1.dot(phi)#input of velocity
    s=numpy.ravel(s)#in order to get the value of the matrix s, here change the dimension of the matrix s to an array
    s_input=s[1]#input of angular velocity

    # #agent 1 or 3
    # if tb3_number==0 or tb3_number==2:
    # #clique1---------------------------------
    #     clique = numpy.array([0,1,2])
    #     #compute matrix Y and Z------------------
    #     k=0
    #     for j in clique:
    #         xrel = data.poses[j].position.x
    #         yrel = data.poses[j].position.y
    #         Y[0][k] = xrel
    #         Y[1][k] = yrel
    #         Z[0][k] = x_star[j]
    #         Z[1][k] = y_star[j]
    #         k=k+1
    #     #compute matrix Y and Z------------------
    #     print(Y,Z)
    #     ave_y=numpy.sum(Y, axis=1)/n #compute the average of matrix Y
    #     ave_y=ave_y.reshape(-1,1) #transpose of ave_y
    #     ave_z=numpy.sum(Z, axis=1)/n
    #     ave_z=ave_z.reshape(-1,1)
    #     #print(ave_y)
    #     ZZ=numpy.mat(Z-ave_z*numpy.array([[1,1,1]]))
    #     YY=numpy.mat(Y-ave_y*numpy.array([[1,1,1]]))
    #     A=(ZZ)*numpy.transpose(YY)

    #     U,sigma,V_t=numpy.linalg.svd(A) #svd of matrix A
    #     # print(V_t)
    #     V=numpy.transpose(V_t)
    #     #print(U,V)
    #     detuv=numpy.linalg.det(U*V)
    #     #print(detuv)
    #     D=numpy.diag([1,detuv])
    #     #print(D)
    #     R=V*D*numpy.transpose(U) #construction of R_hat
    #     phi1=ave_y+R*(numpy.array([[x_star[tb3_number]],[y_star[tb3_number]]])-ave_z)
    #     #print(phi)

    # #clique2---------------------------------
    #     clique = numpy.array([0,2,3])
    #     #compute matrix Y and Z------------------
    #     k=0
    #     for j in clique:
    #         xrel = data.poses[j].position.x
    #         yrel = data.poses[j].position.y
    #         Y[0][k] = xrel
    #         Y[1][k] = yrel
    #         Z[0][k] = x_star[j]
    #         Z[1][k] = y_star[j]
    #         k=k+1
    #     #compute matrix Y and Z------------------
    #     #print(Y)
    #     ave_y=numpy.sum(Y, axis=1)/n #compute the average of matrix Y
    #     ave_y=ave_y.reshape(-1,1) #transpose of ave_y
    #     ave_z=numpy.sum(Z, axis=1)/n
    #     ave_z=ave_z.reshape(-1,1)
    #     #print(ave_y)
    #     ZZ=numpy.mat(Z-ave_z*numpy.array([1,1,1]))
    #     YY=numpy.mat(Y-ave_y*numpy.array([1,1,1]))
    #     A=(ZZ)*numpy.transpose(YY)
    #     U,sigma,V_t=numpy.linalg.svd(A)
    #     # print(U)
    #     # print(V_t)
    #     V=numpy.transpose(V_t)
    #     detuv=numpy.linalg.det(U*V)
    #     #print(detuv)
    #     D=numpy.diag([1,detuv])
    #     #print(D)
    #     R=V*D*numpy.transpose(U)
    #     phi2=ave_y+R*(numpy.array([[x_star[tb3_number]],[y_star[tb3_number]]])-ave_z)
    #     phi=phi1+phi2
    #     #print(phi)
    #     ss=(numpy.identity(2)-numpy.array([[1],[0]])*numpy.array([1,0]))*phi*numpy.array([1,0])
    #     s=ss-numpy.transpose(ss)
    #     u=numpy.array([1,0])*phi

    # if tb3_number==1: #agent 2
    #     clique = numpy.array([0,1,2])#clique 1
    #     #compute matrix Y and Z------------------
    #     k=0
    #     for j in clique:
    #         xrel = data.poses[j].position.x
    #         yrel = data.poses[j].position.y
    #         Y[0][k] = xrel
    #         Y[1][k] = yrel
    #         Z[0][k] = x_star[j]
    #         Z[1][k] = y_star[j]
    #         k=k+1
    #     #compute matrix Y and Z------------------
    #     #print(Y)
    #     ave_y=numpy.sum(Y, axis=1)/n #compute the average of matrix Y
    #     ave_y=ave_y.reshape(-1,1) #transpose of ave_y
    #     ave_z=numpy.sum(Z, axis=1)/n
    #     ave_z=ave_z.reshape(-1,1)
    #     #print(ave_y)
    #     ZZ=numpy.mat(Z-ave_z*numpy.array([1,1,1]))
    #     YY=numpy.mat(Y-ave_y*numpy.array([1,1,1]))
    #     A=(ZZ)*numpy.transpose(YY)

    #     U,sigma,V_t=numpy.linalg.svd(A)
    #     # print(U)
    #     # print(V_t)

    #     V=numpy.transpose(V_t)
    #     detuv=numpy.linalg.det(U*V)
    #     #print(detuv)
    #     D=numpy.diag([1,detuv])
    #     #print(D)
    #     R=V*D*numpy.transpose(U)
    #     phi=ave_y+R*(numpy.array([[x_star[tb3_number]],[y_star[tb3_number]]])-ave_z)
    #     #print(phi)
    #     ss=(numpy.identity(2)-numpy.array([[1],[0]])*numpy.array([1,0]))*phi*numpy.array([1,0])
    #     s=ss-numpy.transpose(ss)
    #     u=numpy.array([1,0])*phi

    # if tb3_number==3: #agent 4
    #     clique = numpy.array([0,2,3])
    #     #compute matrix Y and Z------------------
    #     k=0
    #     for j in clique:
    #         xrel = data.poses[j].position.x
    #         yrel = data.poses[j].position.y
    #         Y[0][k] = xrel
    #         Y[1][k] = yrel
    #         Z[0][k] = x_star[j]
    #         Z[1][k] = y_star[j]
    #         k=k+1
    #     #compute matrix Y and Z------------------

    #     ave_y=numpy.sum(Y, axis=1)/n #compute the average of matrix Y
    #     ave_y=ave_y.reshape(-1,1) #transpose of ave_y
    #     ave_z=numpy.sum(Z, axis=1)/n
    #     ave_z=ave_z.reshape(-1,1)
    #     #print(ave_y)
    #     ZZ=numpy.mat(Z-ave_z*numpy.array([1,1,1]))
    #     YY=numpy.mat(Y-ave_y*numpy.array([1,1,1]))
    #     A=(ZZ)*numpy.transpose(YY)

    #     U,sigma,V_t=numpy.linalg.svd(A)
    #     # print(U)
    #     # print(V_t)

    #     V=numpy.transpose(V_t)
    #     detuv=numpy.linalg.det(U*V)
    #     #print(detuv)
    #     D=numpy.diag([1,detuv])
    #     #print(D)
    #     R=V*D*numpy.transpose(U)
    #     phi=ave_y+R*(numpy.array([[x_star[tb3_number]],[y_star[tb3_number]]])-ave_z)
    #     #print(phi)
    #     ss=(numpy.identity(2)-numpy.array([[1],[0]])*numpy.array([1,0]))*phi*numpy.array([1,0])
    #     s=ss-numpy.transpose(ss)
    #     u=numpy.array([1,0])*phi

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

if __name__ == '__main__':
    try:
        relposition_sub()
    except rospy.ROSInterruptException: pass
