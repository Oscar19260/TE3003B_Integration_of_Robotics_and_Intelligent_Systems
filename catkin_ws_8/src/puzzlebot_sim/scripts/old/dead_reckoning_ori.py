#!/usr/bin/env python  

import rospy  

#from geometry_msgs.msg import Twist  

from nav_msgs.msg import Odometry 

from std_msgs.msg import Float32 

from tf.transformations import quaternion_from_euler 

from visualization_msgs.msg import Marker   # EXTRA

import numpy as np 

 

#This class will do the following: 

#   ...

#   ... 

#   ... 

class LocalisationDeadReckoningClass():  

    def __init__(self):  

        ###******* INIT PUBLISHERS *******###  

        # Create the subscribers to wr and wl topics
        
        rospy.Subscriber("wr", Float32, self.wr_cb)
        
        rospy.Subscriber("wl", Float32, self.wl_cb)

        # Create ROS publishers 

        self.odom_pub = rospy.Publisher('odom', Odometry, queue_size=1) # Publisher to odom topic 
        
        self.marker_pub = rospy.Publisher("/odom_robot_marker", Marker, queue_size = 2)  # Publisher to odom_robot marker

        ############ ROBOT CONSTANTS ################  

        self.r = 0.065 #puzzlebot wheel radius [m] 

        self.L = 0.19 #puzzlebot wheel separation [m] 

        self.delta_t = 0.02 # Desired time to update the robot's pose [s] 

        ############ Variables ############### 
        
        self.wr, self.wl = 0.0, 0.0
        
        self.x_act, self.x_ant          = 0.0, 0.0
        self.y_act, self.y_ant          = 0.0, 0.0
        self.theta_act, self.theta_ant  = 0.0, 0.0

        #self.pose_odometry = Odometry()
        
        ############ Flags  ################# 

        #self.lock_cmd_vel = 0 # This flag will be used to avoid updating the robot's speed when we are computing the robot's pose.

        rate = rospy.Rate(int(1.0/self.delta_t)) # The rate of the while loop will be the inverse of the desired delta_t 

        
        while not rospy.is_shutdown(): 

            ######## Calculate V and W ################# 

            v = (self.r / 2) * (self.wl + self.wr)
            
            w = (self.r / self.L) * (self.wr - self.wl)
            
            ####### Get pose and stamp it ############# 

            [x, y, theta] = self.get_robot_pose(v, w) 

            pose_stamped = self.get_pose_stamped(x, y, theta, v, w)

            ######################################### 
            
            marker = self.fill_marker(pose_stamped)

            ######## Publish the data ################# 

            self.odom_pub.publish(pose_stamped)
            
            self.marker_pub.publish(marker) 

            rate.sleep() 

     
    ############ Methods  #################
     
    def wr_cb(self, msg): 

        self.wr = msg.data
        
        
    def wl_cb(self, msg): 

        self.wl = msg.data
    
    
    def cmd_vel_cb(self, msg): 

        self.v = msg.linear.x 

        self.w = msg.angular.z
        

    def get_pose_stamped(self, x, y, yaw, v, w): 

        # x, y and yaw are the robot's position (x,y) and orientation (yaw) 

        
        # Write the data as a ROS Odometry message 

        pose_odometry = Odometry() 
        
        
        pose_odometry.header.stamp          = rospy.Time.now()

        pose_odometry.header.frame_id       = "base_link"              # Head frame ODOM
        
        #pose_odometry.child_frame_id        = "base_link"    # Child frame BASE_LINK

        # Position 

        pose_odometry.pose.pose.position.x = x 

        pose_odometry.pose.pose.position.y = y 
        
        pose_odometry.pose.pose.position.z = self.r 

        # Rotation of the mobile base frame w.r.t. "map" frame as a quaternion 

        quat = quaternion_from_euler(0,0,yaw) 

        pose_odometry.pose.pose.orientation.x = quat[0] 

        pose_odometry.pose.pose.orientation.y = quat[1] 

        pose_odometry.pose.pose.orientation.z = quat[2] 

        pose_odometry.pose.pose.orientation.w = quat[3] 
        
        ################## COVARIANCE ##################
        
        pose_odometry.pose.covariance = [0]*36
        
        # Twist
        
        pose_odometry.twist.twist.linear.x  = v         #Linear velocity x
        
        pose_odometry.twist.twist.linear.y  = 0.0       #Linear velocity y
        
        pose_odometry.twist.twist.linear.z  = 0.0       #Linear velocity z
        
        pose_odometry.twist.twist.angular.x = 0.0       #Angular velocity around x axis (roll)
        
        pose_odometry.twist.twist.angular.y = 0.0       #Angular velocity around x axis (pitch)
        
        pose_odometry.twist.twist.angular.z = w         #Angular velocity around x axis (yaw)
        
        pose_odometry.twist.covariance      = [0]*36    #Velocity Covariance 6x6 matrix (empty for now)

        
        return pose_odometry 

         

    def get_robot_pose(self, v, w): 

        #This functions receives the robot speed v [m/s] and w [rad/s] 

        # and gets the robot's pose (x, y, theta) where (x,y) are in [m] and (theta) in [rad] 

        # is the orientation,
               

        ############ MODIFY THIS CODE   ################ 

        #rospy.logwarn("set_robot_pose: Make sure to modify this function") 

        x = self.x_ant + (v * np.cos(self.theta_ant) * self.delta_t)

        y = self.y_ant + (v * np.sin(self.theta_ant) * self.delta_t) 

        theta = self.theta_ant + (w * self.delta_t)
        
        
        self.x_ant, self.y_ant, self.theta_ant = x, y, theta
        

        return [x, y, theta]
        
        
    def fill_marker(self, pose_odometry = Odometry()): 

        # This function will fill the necessary data tu publish a marker to rviz.  

        # It receives pose_stamped which must be a [geometry_msgs/PoseStamped] message.  

        marker = Marker() 

        marker.header.frame_id = pose_odometry.header.frame_id 

        marker.header.stamp = rospy.Time.now() 
        
        #marker.ns = ""

 

        # set shape, Arrow: 0; Cube: 1 ; Sphere: 2 ; Cylinder: 3; mesh_resource: 10 

        marker.type = 10 

        marker.id = 0 
        
        marker.action = 0
        
        
        # Note: Must set mesh_resource to a valid URL for a model to appear
        #marker.mesh_resource = "/home/dassdinho/catkin_ws_8/src/puzzlebot_rviz_8/rviz/MCR2_1000_0_Puzzlebot.stl"
        marker.mesh_resource = "package://puzzlebot_rviz_8/rviz/MCR2_1000_0_Puzzlebot.stl"
        #marker.mesh_use_embedded_materials = true
        

        # Set the scale of the marker 

        marker.scale.x = 1.0

        marker.scale.y = 1.0 

        marker.scale.z = 1.0 

        # Set the color 

        marker.color.r = 1.0 

        marker.color.g = 0.0 

        marker.color.b = 0.0 

        marker.color.a = 1.0 

 

        # Set the pose of the marker 

        marker.pose.position.x = pose_odometry.pose.pose.position.x 

        marker.pose.position.y = pose_odometry.pose.pose.position.y 

        marker.pose.position.z = pose_odometry.pose.pose.position.z 

        marker.pose.orientation.x = pose_odometry.pose.pose.orientation.x 

        marker.pose.orientation.y = pose_odometry.pose.pose.orientation.y 

        marker.pose.orientation.z = pose_odometry.pose.pose.orientation.z 

        marker.pose.orientation.w = pose_odometry.pose.pose.orientation.w 

        
        return marker
        
        
############################### MAIN PROGRAM ####################################  

if __name__ == "__main__":  

    # first thing, init a node! 

    rospy.init_node('localisation')  

    LocalisationDeadReckoningClass()
    
