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

        self.r = 0.065 #puzzlebot wheel radius [m] or 0.05

        self.L = 0.19 #puzzlebot wheel separation [m] or 0.18

        self.delta_t = 0.02 # Desired time to update the robot's pose [s] 

        ############ Variables ############### 
        
        self.wr, self.wl = 0.0, 0.0
        
        self.x_act, self.x_ant          = 0.0, 0.0
        self.y_act, self.y_ant          = 0.0, 0.0
        self.theta_act, self.theta_ant  = 0.0, 0.0
        
        self.cov_mat_act, self.cov_mat_ant = np.zeros([3,3]), np.zeros([3,3])
        
        #self.cov_mat_final = np.zeros(36)
        
        self.kr, self.kl = 0.01, 0.01 # 0.1, 0.1

        #self.pose_odometry = Odometry()
        
        ############ Flags  ################# 

        #self.lock_cmd_vel = 0 # This flag will be used to avoid updating the robot's speed when we are computing the robot's pose.

        rate = rospy.Rate(int(1.0/self.delta_t)) # The rate of the while loop will be the inverse of the desired delta_t 

        
        while not rospy.is_shutdown(): 

            ######## Calculate V and W ################# 

            v = (self.r / 2) * (self.wl + self.wr)
            
            w = (self.r / self.L) * (self.wr - self.wl)
            
            ####### Get pose and stamp it ############# 

            [x, y, theta, cov_mat_final] = self.get_robot_pose(v, w, self.wr, self.wl) 

            pose_stamped = self.get_pose_stamped(x, y, theta, v, w, cov_mat_final)

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
        

    def get_pose_stamped(self, x, y, yaw, v, w, cov_mat_final): 

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
        
        pose_odometry.pose.covariance = cov_mat_final       # COVARIANCE POSE MATRIX FINAL
        
        # Twist
        
        pose_odometry.twist.twist.linear.x  = v         #Linear velocity x
        
        pose_odometry.twist.twist.linear.y  = 0.0       #Linear velocity y
        
        pose_odometry.twist.twist.linear.z  = 0.0       #Linear velocity z
        
        pose_odometry.twist.twist.angular.x = 0.0       #Angular velocity around x axis (roll)
        
        pose_odometry.twist.twist.angular.y = 0.0       #Angular velocity around x axis (pitch)
        
        pose_odometry.twist.twist.angular.z = w         #Angular velocity around x axis (yaw)
        
        pose_odometry.twist.covariance      = [0]*36    #Velocity Covariance 6x6 matrix (empty for now)

        
        return pose_odometry 

         

    def get_robot_pose(self, v, w, wr, wl): 

        #This functions receives the robot speed v [m/s] and w [rad/s] 

        # and gets the robot's pose (x, y, theta) where (x,y) are in [m] and (theta) in [rad] 

        # is the orientation,
               

        ############ ROBOT POSE   ################ 

        #rospy.logwarn("set_robot_pose: Make sure to modify this function") 

        x = self.x_ant + (v * np.cos(self.theta_ant) * self.delta_t)

        y = self.y_ant + (v * np.sin(self.theta_ant) * self.delta_t) 

        theta = self.theta_ant + (w * self.delta_t)
        theta = np.arctan2(np.sin(theta), np.cos(theta)) #Make theta from -pi to pi
        
        ############ COVARIANCE POSE MATRIX   ################
        
        Nabla_wk = 0.5 * self.r * self.delta_t * np.array([[np.cos(self.theta_ant), np.cos(self.theta_ant)], 
                                                           [np.sin(self.theta_ant), np.sin(self.theta_ant)], 
                                                           [2/self.L, -2/self.L]])
        #print("Nabla_wk:", np.shape(Nabla_wk))
        
        E_delta_k = np.array([[self.kr * abs(wr), 0], 
                              [0, self.kl * abs(wl)]])
        #print("E_delta_k:", np.shape(E_delta_k))
        
        Qk = Nabla_wk.dot(E_delta_k).dot(Nabla_wk.T)
        #print("Qk:", np.shape(Qk))
        
        Hk = np.array([[1, 0, -self.delta_t * v * np.sin(self.theta_ant)], 
                       [0, 1, self.delta_t * v * np.cos(self.theta_ant)], 
                       [0, 0, 1]])
        #print("Hk:", np.shape(Hk))
        
        
        Ek = Hk.dot(self.cov_mat_ant).dot(Hk.T) + Qk
        #print("\nEk:", np.shape(Ek))
        #print("\nEk:", Ek)
        
        ############ UPDATE VALUES   ################ 
        
        self.x_ant, self.y_ant, self.theta_ant = x, y, theta
        self.cov_mat_ant = Ek
        
        cov_mat_final = np.zeros(36)
        
        # Diagonal
        cov_mat_final[0]   = Ek[0][0]   # xx
        cov_mat_final[7]   = Ek[1][1]   # yy
        cov_mat_final[35]  = Ek[2][2]   # tt
        # Else
        cov_mat_final[1]   = Ek[0][1]  # xy
        cov_mat_final[5]   = Ek[0][2]  # xt
        cov_mat_final[6]   = Ek[1][0]  # yx
        cov_mat_final[11]  = Ek[1][2]  # yt
        cov_mat_final[30]  = Ek[2][0]  # tx
        cov_mat_final[31]  = Ek[2][1]  # ty
        
        #print(" cov_mat_final:", np.shape(cov_mat_final))
        
        cov_mat_final_list =  cov_mat_final.tolist()

        return [x, y, theta, cov_mat_final_list]
        
        
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
    