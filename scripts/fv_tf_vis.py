#!/usr/bin/env python
# import roslib;
# roslib.load_manifest('fv_sensor')
import numpy as np
import rospy
import tf
from math import pi



from sensor_msgs.msg import Range
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Vector3, Wrench, WrenchStamped

from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_input  as inputMsg

d_l = 0.12
d_r = -0.12
force_l = np.array([0.707,0,0.707])
force_r = np.array([0.707,0,0.707])

friction_angle_l = 0.0
friction_angle_r = 0.0

friction_angle = 43.0

def gripper_pos_callback(status):
    position = status.gPO
    
    #  compute distance from gripper position
    distance = (position*(-0.5767) + 124.2 + 50)/1000
    # print 'Current gripper position: ' + str(distance)
    # update the fv_l and fv_r frame
    # print distance
    global d_l, d_r
    d_l = distance/2
    d_r = - distance/2


def define_cone(frame_id):
    cone = Range()
    cone.header.frame_id = frame_id
    cone.field_of_view = pi * friction_angle*2 / 180
    cone.radiation_type = 1
    cone.min_range = 0.05
    cone.max_range = 0.1
    cone.range = 0.05
    
    return cone


def define_arrow(frame_id):
    marker = Marker()
    marker.action = Marker.ADD
    marker.header.frame_id = frame_id
    # marker.ns = 'Fv'
    
    marker.type = Marker.ARROW
    # marker.pose.orientation.y = 0
    # marker.pose.orientation.w = 1
    marker.points = [Point(0, 0, 0), Point(0.1, 0, 0.1)]
    
    marker.scale = Vector3(0.008, 0.016, 0.024)
    marker.color.r = 0.2
    marker.color.g = 0.6
    marker.color.b = 0.4
    marker.color.a = 1.0
    
    return marker


def wrench_update_l(data):
    wrench_l = data.wrench
    # wrench_l.force.x = - wrench_l.force.x
    # wrench_l.force.y = - wrench_l.force.y
    # wrench_l.torque.z = - wrench_l.torque.z

    force = np.array([wrench_l.force.x, wrench_l.force.y, wrench_l.force.z])
    f_norm = np.linalg.norm(force)

    global force_l, friction_angle_l
    friction_angle_l = angle(force, np.array([0, 0, 1]))
    # print force, friction_angle_l
    # print force
    force_l = force / f_norm
    
def wrench_update_r(data):
    # print 'data recieved'
    wrench_r = data.wrench
    # wrench_r.force.x = - wrench_r.force.x
    # wrench_r.force.y = - wrench_r.force.y
    # wrench_r.torque.z = - wrench_r.torque.z

    force = np.array([wrench_r.force.x, wrench_r.force.y, wrench_r.force.z])
    f_norm = np.linalg.norm(force)
    
    global force_r, friction_angle_r
    friction_angle_r = angle(force, np.array([0, 0, 1]))
    # print friction_angle_r
    # print force_r
    force_r = force/f_norm
    
    
def angle(v1, v2):
    length_1 = np.sqrt(np.dot(v1, v1))
    # print length_1
    length_2 = np.sqrt(np.dot(v2, v2))
    # print length_2
    # print np.dot(v1, v2)
    return np.arccos(np.clip(np.dot(v1, v2)/(length_1*length_2), -1.0, 1.0))
    
    
if __name__ == '__main__':
    rospy.init_node('fv_tf_vis')
    br = tf.TransformBroadcaster()
    
    cone_pub_l = rospy.Publisher('/Fv/cones_l', Range, queue_size=2)
    cone_pub_r = rospy.Publisher('/Fv/cones_r', Range, queue_size=2)
    arrow_pub_l = rospy.Publisher('Fv/wrench_vector_l', Marker, queue_size=2)
    arrow_pub_r = rospy.Publisher('Fv/wrench_vector_r', Marker, queue_size=2)

    rospy.Subscriber("Robotiq2FGripperRobotInput",
                     inputMsg.Robotiq2FGripper_robot_input,
                     gripper_pos_callback)
    
    rospy.Subscriber('/Fv/wrench_l', WrenchStamped, wrench_update_l)
    rospy.Subscriber('/Fv/wrench_r', WrenchStamped, wrench_update_r)

    rate = rospy.Rate(20.0)

    origin, xaxis, yaxis, zaxis = (0, 0, 0), (1, 0, 0), (0, 1, 0), (0, 0, 1)
    # Ry = tf.transformations.rotation_matrix(pi, yaxis)
    # Rx = tf.transformations.rotation_matrix(pi/2, xaxis)
    # R = tf.transformations.concatenate_matrices(Ry, Rx)
    # euler = tf.transformations.euler_from_matrix(R, 'sxyz')
    # quaternion = tf.transformations.quaternion_from_euler(euler[0], euler[1], euler[2], axes='sxyz')
    
    # static rotation trans
    quat_y = tf.transformations.quaternion_about_axis(pi, yaxis)
    quat_x_l = tf.transformations.quaternion_about_axis(pi / 2, xaxis)
    quat_x_r = tf.transformations.quaternion_about_axis(-pi / 2, xaxis)

    quaternion_l = tf.transformations.quaternion_multiply(quat_y, quat_x_l)
    quaternion_r = tf.transformations.quaternion_multiply(quat_y, quat_x_r)
    
    quat_cone_fv = tf.transformations.quaternion_about_axis(-pi/2, yaxis)

    cone_l = define_cone('cone_l')
    cone_r = define_cone('cone_r')
    
    arrow_l = define_arrow('fv_l')
    arrow_r = define_arrow('fv_r')
    # print 'before while'
    while not rospy.is_shutdown():
        # print d_l
        # print 'publish transformation'
        br.sendTransform((0.255, d_l, 0),
                         (quaternion_l[0], quaternion_l[1], quaternion_l[2], quaternion_l[3]),
                             rospy.Time.now(),
                             "fv_r",
                             "ee_link")
        br.sendTransform((0.255, d_r, 0),
                         (quaternion_r[0], quaternion_r[1], quaternion_r[2], quaternion_r[3]),
                             rospy.Time.now(),
                             "fv_l",
                             "ee_link")
        
        br.sendTransform((0.0, 0.0, 0.0),
                         (quat_cone_fv[0], quat_cone_fv[1], quat_cone_fv[2], quat_cone_fv[3]),
                         rospy.Time.now(),
                         "cone_l",
                         "fv_l")

        br.sendTransform((0.0, 0.0, 0.0),
                         (quat_cone_fv[0], quat_cone_fv[1], quat_cone_fv[2], quat_cone_fv[3]),
                         rospy.Time.now(),
                         "cone_r",
                         "fv_r")

        cone_l = define_cone('cone_l')
        cone_r = define_cone('cone_r')
        
        cone_l.header.stamp = rospy.Time.now()
        cone_r.header.stamp = rospy.Time.now()
        arrow_l.header.stamp = rospy.Time.now()
        arrow_l.header.stamp = rospy.Time.now()
        
        
        force_l_vis = 0.07 * force_l
        force_r_vis = 0.07 * force_r
        # print force_l
        # print arrow_l.points
        
        arrow_l.points[1] = Point(force_l_vis[0], force_l_vis[1], force_l_vis[2])
        arrow_r.points[1] = Point(force_r_vis[0], force_r_vis[1], force_r_vis[2])
        
        # print arrow_l.points
        
        # update arrow color to indicate slip occurrence rate
        # -- red - occurring
        # -- green - safe
        # print friction_angle_l*180/pi
        if friction_angle_l < pi * friction_angle / 180 and friction_angle_l > 0.0:
            arrow_l.color.r = 0.2
            arrow_l.color.g = 0.6
            arrow_l.color.b = 0.4
        else:
            arrow_l.color.r = 6.0
            arrow_l.color.g = 0.2
            arrow_l.color.b = 0.2
        
        if friction_angle_r < pi * friction_angle / 180 and friction_angle_r > 0.0:
            arrow_r.color.r = 0.2
            arrow_r.color.g = 0.6
            arrow_r.color.b = 0.4
        else:
            arrow_r.color.r = 6.0
            arrow_r.color.g = 0.2
            arrow_r.color.b = 0.2
        
        
        cone_pub_l.publish(cone_l)
        cone_pub_r.publish(cone_r)
        arrow_pub_l.publish(arrow_l)
        arrow_pub_r.publish(arrow_r)
        
        rate.sleep()