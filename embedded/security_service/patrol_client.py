
import numpy as np
import cv2
import os
import rclpy
import base64

from rclpy.node import Node
from geometry_msgs.msg import Twist,Point
from ssafy_msgs.msg import TurtlebotStatus
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float32,Int8MultiArray
from squaternion import Quaternion
from nav_msgs.msg import Odometry,Path
import geometry_msgs.msg
from math import pi,cos,sin,sqrt,atan2
import tf2_ros

global auto_switch
auto_switch=0

global m_control_cmd
m_control_cmd=0


class PatrolCtrlFromServer(Node):

    def __init__(self):
        super().__init__('Patrol_client')

        self.cmd_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.cmd_msg=Twist()

        self.subscription = self.create_subscription(Odometry,'/odom',self.odom_callback,10)
        self.path_sub = self.create_subscription(Path,'/global_path',self.path_callback,10)

        self.timer_period = 0.05
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.m_control_interval = 10
        self.m_control_iter = 0
        self.m_control_mode = 0

        self.lfd=0.1
        self.idx_wp = 0
        self.len_wp = None
        self.check_1_wp = False
        self.is_path = False
        self.is_odom = False

    def odom_callback(self, msg):
        self.is_odom=True
        self.odom_msg=msg
        q=Quaternion(msg.pose.pose.orientation.w,msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z)
        _,_,self.robot_yaw=q.to_euler()

    def path_callback(self, msg):
        self.is_path=True
        self.path_msg=msg

    def turtlebot_go(self) :
        self.cmd_msg.linear.x=0.2
        self.cmd_msg.angular.z=0.0

    def turtlebot_stop(self) :
        self.cmd_msg.linear.x=0.0
        self.cmd_msg.angular.z=0.0

    def turtlebot_cw_rot(self) :
        self.cmd_msg.linear.x=0.0
        self.cmd_msg.angular.z=0.06

    def turtlebot_cww_rot(self) :
        self.cmd_msg.linear.x=0.0
        self.cmd_msg.angular.z=-0.06

    def search_start_point(self) :

        self.len_wp = len(self.path_msg.poses)

        print("searching the start points ... ")

        dis_list = []

        for wp in self.path_msg.poses:

            robot_pose_x=self.odom_msg.pose.pose.position.x
            robot_pose_y=self.odom_msg.pose.pose.position.y

            self.current_point=wp.pose.position

            dx= self.current_point.x - robot_pose_x
            dy= self.current_point.y - robot_pose_y

            dis=sqrt(pow(dx,2)+pow(dy,2))

            dis_list.append(dis)

        self.idx_wp = np.argmin(dis_list)

        self.check_1_wp = True
        
    def timer_callback(self):
        

        if self.is_odom == True and self.is_path==True:

            if not self.check_1_wp: 
                
                self.search_start_point()

            rotated_point=Point()
            robot_pose_x=self.odom_msg.pose.pose.position.x
            robot_pose_y=self.odom_msg.pose.pose.position.y

            waypoint = self.path_msg.poses[self.idx_wp]

            self.current_point=waypoint.pose.position

            dx= self.current_point.x - robot_pose_x
            dy= self.current_point.y - robot_pose_y
            rotated_point.x=cos(self.robot_yaw)*dx + sin(self.robot_yaw)*dy
            rotated_point.y= -sin(self.robot_yaw)*dx + cos(self.robot_yaw)*dy

            theta=atan2(rotated_point.y,rotated_point.x)
            
            dis=sqrt(pow(rotated_point.x,2)+pow(rotated_point.y,2))

            if abs(theta) < pi/10:
                
                self.cmd_msg.linear.x=dis*2
                self.cmd_msg.angular.z=-theta*0.1
            
            else:
                
                self.cmd_msg.linear.x=0.0
                self.cmd_msg.angular.z=-theta*0.1

            if dis <= self.lfd and  self.idx_wp < self.len_wp-1:

                self.idx_wp += 1

            elif dis <= self.lfd and  self.idx_wp == self.len_wp-1:

                self.idx_wp = 0
        else:

            self.turtlebot_stop()

        self.cmd_publisher.publish(self.cmd_msg)



def main(args=None):
    
    rclpy.init(args=args)

    Patrol_client = PatrolCtrlFromServer()

    rclpy.spin(Patrol_client)
    
    rclpy.shutdown()

    sio.disconnect()


if __name__ == '__main__':
    main()

