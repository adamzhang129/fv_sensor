#!/usr/bin/env python

# import roslib; roslib.load_manifest('fv_sensor')
import rospy
import numpy as np
import tf

from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_input  as inputMsg
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_output as outputMsg

from geometry_msgs.msg import WrenchStamped, Wrench

def gripper_move_command(degree):
    command = outputMsg.Robotiq2FGripper_robot_output();
    command.rACT = 1
    command.rGTO = 1
    command.rPR = degree
    command.rSP = 0
    command.rFR = 150
    # gripper_set_vel_pub.publish(command)
    return command


def angle(v1, v2):
    length_1 = np.sqrt(np.dot(v1, v1))
    # print length_1
    length_2 = np.sqrt(np.dot(v2, v2))
    # print length_2
    # print np.dot(v1, v2)
    return np.arccos(np.clip(np.dot(v1, v2)/(length_1*length_2), -1.0, 1.0))

class GripperFbController:
    def __init__(self):
        self.position = 0
        self.requested_pos = 0
        self.if_arrived = True
        self.wrench_l = Wrench()
        self.wrench_r = Wrench()
        self.mu = 1
        self.threhold = 0.10
        
        self.friction_angle_l = 0.0
        self.friction_angle_r = 0.0
        
        rospy.init_node('gripper_fb_controller')
    
        rospy.Subscriber('/Fv/wrench_l', WrenchStamped, self.wrench_update_l)
        rospy.Subscriber('/Fv/wrench_r', WrenchStamped, self.wrench_update_r)
        rospy.Subscriber("Robotiq2FGripperRobotInput",
                         inputMsg.Robotiq2FGripper_robot_input,
                         self.gripper_pos_callback)
        self.pub = rospy.Publisher('Robotiq2FGripperRobotOutput',
                                   outputMsg.Robotiq2FGripper_robot_output,
                                   queue_size=10)
        

    def gripper_pos_callback(self, status):
        self.position = status.gPO
        #  compute distance from gripper position
        distance = (self.position * (-0.6783) + 147.41) / 1000
    
        self.requested_pos = status.gPR
        self.if_arrived = (status.gOBJ == 3)
    
        print '[gripper_sub_cb] Current gripper position: ' + str(self.position) + ', equivalent distance:' + str(distance)
        print '[gripper_sub_cb] Requested position:' + str(self.requested_pos) + '(arrived?: {})'.format(self.if_arrived)
    
    
    def wrench_update_l(self, data):
        print 'updated left'
        self.wrench_l = data.wrench
        self.wrench_l.force.x = - self.wrench_l.force.x
        self.wrench_l.force.y = - self.wrench_l.force.y
        self.wrench_l.torque.z = - self.wrench_l.torque.z

        force = np.array([self.wrench_l.force.x, self.wrench_l.force.y, self.wrench_l.force.z])
        self.friction_angle_l = angle(force, np.array([0, 0, 1]))
    
    def wrench_update_r(self, data):
        print 'updated right'
        self.wrench_r = data.wrench
        self.wrench_r.force.x = - self.wrench_r.force.x
        self.wrench_r.force.y = - self.wrench_r.force.y
        self.wrench_r.torque.z = - self.wrench_r.torque.z

        force = np.array([self.wrench_r.force.x, self.wrench_r.force.y, self.wrench_r.force.z])
        self.friction_angle_r = angle(force, np.array([0, 0, 1]))
    
    def controller(self):
        # get current wrench and gripper position
        fnl = self.wrench_l.force.z
        fxl = self.wrench_l.force.x
        fyl = self.wrench_l.force.y
        taul = self.wrench_l.torque.z

        fnr = self.wrench_r.force.z
        fxr = self.wrench_r.force.x
        fyr = self.wrench_r.force.y
        taur = self.wrench_r.torque.z
        
        # determine if contacts have been made
        ftl = np.hypot(fxl, fyl)
        ftr = np.hypot(fxr, fyr)
        # print ftl, ftr
        preset_ratio = 0.85
        if ftl < 2 or ftr < 2:
            print "contacts haven't been made"
        
        else:
            if ftl/fnl < (preset_ratio - self.threhold)*self.mu or ftr/fnr < (preset_ratio - self.threhold)*self.mu:
                # decrease gripper force
                self.requested_pos = self.position - 1
                self.pub.publish(gripper_move_command(self.requested_pos))
            elif ftl/fnl > (preset_ratio + self.threhold)*self.mu or ftr/fnr > (preset_ratio + self.threhold)*self.mu:
                # increase gripper force
                self.requested_pos = self.position + 1
                self.pub.publish(gripper_move_command(self.requested_pos))
        
            
if __name__ == '__main__':
    
    gripper_controller = GripperFbController()
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        gripper_controller.controller()

        rospy.sleep(0.01)
    
