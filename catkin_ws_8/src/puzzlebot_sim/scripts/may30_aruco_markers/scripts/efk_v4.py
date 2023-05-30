#!/usr/bin/env python  

import rospy
import tf2_ros 
import numpy as np

from nav_msgs.msg import Odometry 
from std_msgs.msg import Float32
from visualization_msgs.msg import Marker 
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from geometry_msgs.msg import TransformStamped, Vector3
 

#This class will do the following: 
#   ...
#   ... 
#   ... 

class EKFClass():  
    def __init__(self):  
        ###******* INIT PUBLISHERS *******###  
        # Create the subscribers to wr and wl topics
        rospy.Subscriber("wr", Float32, self.wr_cb)
        rospy.Subscriber("wl", Float32, self.wl_cb)
        
        
        rospy.Subscriber("r_vect", Vector3, self.r_vect_cb)
        rospy.Subscriber("aruco_pos", Vector3, self.aruco_pos_cb)
        rospy.Subscriber("robot_aruco", Vector3, self.robot_aruco_cb)
        
        # Create ROS publishers 
        self.tf_br = tf2_ros.TransformBroadcaster()

        self.odom_dr_pub    = rospy.Publisher('odom_dr', Odometry, queue_size=1) # Publisher to odom_DR topic 
        self.odom_ekf_pub   = rospy.Publisher('odom_ekf', Odometry, queue_size=1) # Publisher to odom_EKF topic
        
        self.marker_ekf_pub = rospy.Publisher("/odom_robot_marker", Marker, queue_size=1)  # Publisher to odom_robot marker
        
        self.est_pose_robot_pub   = rospy.Publisher('est_pose_robot', Vector3, queue_size=1)

        ############ ROBOT CONSTANTS ################  
        self.r = 0.05 # puzzlebot wheel radius [m] or 0.05 or 0.065
        self.L = 0.19 # puzzlebot wheel separation [m] or 0.18 or 0.19
        self.delta_t = 0.0 # Desired time to update the robot's pose [s] 
        freq = 50 # Hz

        ############ Variables ############### 
        init_pose = [0.0000000000000000000001, 0.0000000000000000000001]              # INIT POSE
        
        self.r_vect         = Vector3()
        self.aruco_pos      = Vector3()
        self.robot_aruco    = Vector3()
        
        d_gtg               = Vector3() # dx, dy AND dtheta for GTG_control NONE
        
        odom_dr     = []
        odom_ekf    = []
        
        ###
        
        self.wr, self.wl = 0.0, 0.0
        
        self.x_act, self.x_ant          = 0.0, init_pose[0]
        self.y_act, self.y_ant          = 0.0, init_pose[1]
        self.theta_act, self.theta_ant  = 0.0, 0.0
        
        self.theta_r_act, self.theta_r_ant  = 0.0, 0.0
        self.theta_l_act, self.theta_l_ant  = 0.0, 0.0
        
        self.cov_mat_act, self.cov_mat_ant = np.zeros([3,3]), np.zeros([3,3])
        
        self.kr, self.kl = 6.0, 6.0 # 0.1, 0.1            ################################################## TUNE DR COV MATRIX ################################################## 
        
        
        self.time_ant, self.time_act = 0.0, 0.0
        

        rate = rospy.Rate(freq) # The rate of the while loop will be the inverse of the desired delta_t ---> 50

        
        while not rospy.is_shutdown(): 
            
            print("ENTRO")
        
            self.time_act = rospy.get_time()
            
            self.delta_t = self.time_act - self.time_ant
            
            delta_t = self.delta_t
            
            
            aruco_flag  = int( self.aruco_pos.x )                           # aruco_pos [flag, x, y] --> [.x]  
            Rk          = np.array([[ self.r_vect.x,              0], 
                                    [             0,  self.r_vect.y]])      # r_vect [r_d, r_a, 0] --> [.x, .y]
            m_detected  = [ self.aruco_pos.y, self.aruco_pos.z ]            # aruco_pos [flag, x, y] --> [.y, .z]
            zk          = [ self.robot_aruco.x, self.robot_aruco.y ]        # robot_aruco [dist, ang, 0] --> [.y, .z]

            ######## Calculate V and W ################# 

            v = (self.r / 2) * (self.wl + self.wr)
            
            w = (self.r / self.L) * (self.wr - self.wl)
            
            ####### Get pose and stamp it ############# 
            ####### KF --> predict AND correct #############
            
            # Always PREDICT/DR
            [x, y, theta, cov_mat_final, Ek] = self.kf_predict(v, w, self.wr, self.wl, delta_t) #predict KF
            odom_dr     = [x, y, theta, cov_mat_final]
            odom_ekf    = [x, y, theta, cov_mat_final]
            
            # If ARUCO DETECT
            if aruco_flag == 1:
                print("ARUCO DETECTED")
                print("M")
                print(m_detected)
                print("Rk")
                print(Rk)
                print("zk")
                print(zk)
                [x, y, theta, cov_mat_final] = self.kf_correct(x, y, theta, Ek, m_detected, Rk, zk) #correct KF
                odom_ekf = [x, y, theta, cov_mat_final]

            #print(odom_dr)
            #print(odom_ekf)
            
            pose_stamped_dr     = self.get_pose_stamped(odom_dr[0], odom_dr[1], odom_dr[2], v, w, odom_dr[3], "base_link_dr")
            pose_stamped_ekf    = self.get_pose_stamped(odom_ekf[0], odom_ekf[1], odom_ekf[2], v, w, odom_ekf[3], "base_link_efk")
            
            d_gtg = Vector3()
            d_gtg.x = odom_ekf[0]
            d_gtg.y = odom_ekf[1]
            d_gtg.z = odom_ekf[2]

            ######## Publish TFs to rviz ######### 

            self.send_base_link_tf(pose_stamped_dr, "base_link_dr")
            self.send_base_link_tf(pose_stamped_ekf, "base_link_efk")

            self.send_chassis_link_tf()
            
            self.send_right_wheel_link_tf(self.theta_r_act)
            self.send_left_wheel_link_tf(self.theta_l_act)
            
            ######## Publish marker and odom msgs to rviz ######### 

            marker_ekf      = self.fill_marker(pose_stamped_ekf)
            
            self.odom_dr_pub.publish(pose_stamped_dr)
            self.odom_ekf_pub.publish(pose_stamped_ekf)

            self.marker_ekf_pub.publish(marker_ekf)
            
            ######## Publish dx, dy AND dtheta to GTG_control #########
            
            self.est_pose_robot_pub.publish(d_gtg)
            
            #print(d_gtg)
            print("END")
            print(" ")

            
            rate.sleep()
            
     
    ############ Methods  #################
             
    def r_vect_cb(self, msg):
    
        self.r_vect = msg
        
    def aruco_pos_cb(self, msg):
    
        self.aruco_pos = msg
    
    def robot_aruco_cb(self, msg):
    
        self.robot_aruco = msg
        
    
    def wr_cb(self, msg):
    
        self.wr = msg.data
        
        
    def wl_cb(self, msg): 

        self.wl = msg.data
    
    
    def cmd_vel_cb(self, msg): 

        self.v = msg.linear.x 

        self.w = msg.angular.z
        
    def euclidian_distance(self, x2, y2, x1, y1):
        
        dis = np.sqrt( (x2 - x1)**2 + (y2 - y1)**2 )
        
        return dis
        
    def get_pose_stamped(self, x, y, yaw, v, w, cov_mat_final, str_cf_id): 

        # x, y and yaw are the robot's position (x,y) and orientation (yaw) 

        
        # Write the data as a ROS Odometry message 

        pose_odometry = Odometry() 
        
        
        pose_odometry.header.stamp          = rospy.Time.now()

        pose_odometry.header.frame_id       = "odom"            # Head frame ODOM
        
        pose_odometry.child_frame_id        = str_cf_id         # Child frame BASE_LINK

        # Position 

        pose_odometry.pose.pose.position.x = x 

        pose_odometry.pose.pose.position.y = y 
        
        pose_odometry.pose.pose.position.z = 0.0 # self.r 

        # Rotation of the mobile base frame w.r.t. "map" frame as a quaternion 

        quat = quaternion_from_euler(0,0,yaw) 

        pose_odometry.pose.pose.orientation.x = quat[0] 

        pose_odometry.pose.pose.orientation.y = quat[1] 

        pose_odometry.pose.pose.orientation.z = quat[2] 

        pose_odometry.pose.pose.orientation.w = quat[3] 
        
        ################## COVARIANCE ##################
        
        pose_odometry.pose.covariance = cov_mat_final       # COVARIANCE POSE MATRIX FINAL
        
        # Twist
        
        pose_odometry.twist.twist.linear.x  = 0.0         #Linear velocity x ------------------------------------------ v ################
        
        pose_odometry.twist.twist.linear.y  = 0.0       #Linear velocity y
        
        pose_odometry.twist.twist.linear.z  = 0.0       #Linear velocity z
        
        pose_odometry.twist.twist.angular.x = 0.0       #Angular velocity around x axis (roll)
        
        pose_odometry.twist.twist.angular.y = 0.0       #Angular velocity around x axis (pitch)
        
        pose_odometry.twist.twist.angular.z = 0.0         #Angular velocity around x axis (yaw) ----------------------- w ################
        
        pose_odometry.twist.covariance      = [0]*36    #Velocity Covariance 6x6 matrix (empty for now)

        
        return pose_odometry 

         

    def kf_predict(self, v, w, wr, wl, delta_t): 

        #This functions receives the robot speed v [m/s] and w [rad/s] 

        # and gets the robot's pose (x, y, theta) where (x,y) are in [m] and (theta) in [rad] 

        # is the orientation,
        
        
        ######## Calculate THETA_R and THETA_L #########
            
        self.theta_r_act = self.theta_r_ant + wr * (delta_t)
        self.theta_l_act = self.theta_l_ant + wl * (delta_t)
        
        ############ ROBOT POSE   ################ 

        #rospy.logwarn("set_robot_pose: Make sure to modify this function") 

        x = self.x_ant + (v * np.cos(self.theta_ant) * delta_t)

        y = self.y_ant + (v * np.sin(self.theta_ant) * delta_t) 

        theta = self.theta_ant + (w * delta_t)
        theta = np.arctan2(np.sin(theta), np.cos(theta)) #Make theta from -pi to pi
        
        ############ COVARIANCE POSE MATRIX   ################
        
        Nabla_wk = 0.5 * self.r * delta_t * np.array([[np.cos(self.theta_ant), np.cos(self.theta_ant)], 
                                                      [np.sin(self.theta_ant), np.sin(self.theta_ant)], 
                                                      [              2/self.L,              -2/self.L]])
        #print("Nabla_wk:", np.shape(Nabla_wk))
        
        E_delta_k = np.array([[self.kr * abs(wr),                 0], 
                              [                0, self.kl * abs(wl)]])
        #print("E_delta_k:", np.shape(E_delta_k))
        
        Qk = Nabla_wk.dot(E_delta_k).dot(Nabla_wk.T)
        #print("Qk:", np.shape(Qk))
        
        Hk = np.array([[1, 0, -delta_t * v * np.sin(self.theta_ant)], 
                       [0, 1,  delta_t * v * np.cos(self.theta_ant)], 
                       [0, 0,                                    1]])
        #print("Hk:", np.shape(Hk))
        
        
        Ek = Hk.dot(self.cov_mat_ant).dot(Hk.T) + Qk
        #print("\nEk:", np.shape(Ek))
        #print("\nEk:", Ek)
        
        ############ UPDATE VALUES   ################ 
        
        self.time_ant = self.time_act
        
        self.x_ant, self.y_ant, self.theta_ant = x, y, theta
        self.cov_mat_ant = Ek
        
        self.theta_r_ant = self.theta_r_act
        self.theta_l_ant = self.theta_l_act
        
                
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
        

        return [x, y, theta, cov_mat_final_list, Ek]
        
        
    def kf_correct(self, x, y, theta, Ek_hat, m_detected, Rk, zm):

        # Kalman Filter Corrrection Stage
        
        ## INITIAL ##
        
        s_hat   = np.array([[x, y, theta]]).T
        miu_hat = np.array([[x, y, theta]]).T
        m       = np.array([[m_detected[0], m_detected[1]]]).T  # value from Aruco Cam --> WAIT
        
        ######## Calculate Gk #########
        
        delta_x = m[0][0] - s_hat[0][0]
        delta_y = m[1][0] - s_hat[1][0]
        p       = delta_x**2 + delta_y**2
            
        Gk = np.array([[-delta_x / np.sqrt(p), -delta_y / np.sqrt(p),  0], 
                       [          delta_y / p,          -delta_x / p, -1]])
        
        ######## Calculate Zk #########
            
        Zk = Gk.dot(Ek_hat).dot(Gk.T) + Rk
        
        ######## Calculate Kk #########
            
        Kk = Ek_hat.dot(Gk.T).dot( np.linalg.inv(Zk) )
        
        ######## Calculate zk AND z_hat #########
        
        zk_p = zm[0]
        zk_a = zm[1]
        
        # x2, y2, x1, y1 = m[0][0], m[1][0], miu_hat[0][0], miu_hat[1][0]
        z_hat_p = self.euclidian_distance( m[0][0], m[1][0], miu_hat[0][0], miu_hat[1][0] )
        z_hat_a = np.arctan2( ( m[1][0] - miu_hat[1][0] ), ( m[0][0] - miu_hat[0][0] ) ) - miu_hat[2][0]
        z_hat_a = np.arctan2( np.sin(z_hat_a), np.cos(z_hat_a) )
            
        zk    = np.array([[zk_p, zk_a]]).T
        z_hat = np.array([[z_hat_p, z_hat_a]]).T
        
        # LAST ####### Calculate miu_K #########
            
        miu_K = miu_hat + Kk.dot(zk - z_hat)
        
        # LAST ####### Calculate Ek #########
        
        I = np.eye(3)   # 3x3
        
        Ek = (I - Kk.dot(Gk)).dot(Ek_hat)
        
        
        ############ UPDATE VALUES ################
        
        self.x_ant, self.y_ant, self.theta_ant = miu_K[0][0], miu_K[1][0], miu_K[2][0]
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
        

        return [miu_K[0][0], miu_K[1][0], miu_K[2][0], cov_mat_final_list]
        
    
    def send_base_link_tf(self, pose_stamped, str_cf_id): 

        # This receives the robot's pose and broadcast a transformation. 

         

        t = TransformStamped() 

 

        t.header.stamp = rospy.Time.now() 

        t.header.frame_id   = "odom" 

        t.child_frame_id    = str_cf_id 

        #Copy data from the received pose to the tf  

        t.transform.translation.x = pose_stamped.pose.pose.position.x 

        t.transform.translation.y = pose_stamped.pose.pose.position.y 

        t.transform.translation.z = pose_stamped.pose.pose.position.z 

         

        t.transform.rotation.x = pose_stamped.pose.pose.orientation.x 

        t.transform.rotation.y = pose_stamped.pose.pose.orientation.y 

        t.transform.rotation.z = pose_stamped.pose.pose.orientation.z 

        t.transform.rotation.w = pose_stamped.pose.pose.orientation.w 

        # Send the transformation 

        self.tf_br.sendTransform(t) 

 

    def send_chassis_link_tf(self): 

        t = TransformStamped() 

 

        t.header.stamp = rospy.Time.now() 

        t.header.frame_id = "base_link_efk"

        t.child_frame_id = "chassis_rviz" 

        #Copy data from the received pose to the tf  

        t.transform.translation.x = 0.0 

        t.transform.translation.y = 0.0 

        t.transform.translation.z = 0.0 

        quat = quaternion_from_euler(np.pi/2.0, 0.0, np.pi/2.0) 
        #quat = quaternion_from_euler(0.0, 0.0, 0.0) 

        t.transform.rotation.x = quat[0] 

        t.transform.rotation.y = quat[1] 

        t.transform.rotation.z = quat[2] 

        t.transform.rotation.w = quat[3] 

        # Send the transformation 

        self.tf_br.sendTransform(t)
        
        
        
    def send_right_wheel_link_tf(self, theta_r_act): 

        t = TransformStamped() 

 

        t.header.stamp = rospy.Time.now() 

        t.header.frame_id = "base_link_efk"

        t.child_frame_id = "right_wheel_rviz" 

        #Copy data from the received pose to the tf  

        t.transform.translation.x = 0.05 

        t.transform.translation.y = -0.09 

        t.transform.translation.z = 0.0 

        #quat = quaternion_from_euler(np.pi/2.0, 0.0, np.pi/2.0) 
        quat = quaternion_from_euler(0.0, theta_r_act, 0.0) 

        t.transform.rotation.x = quat[0] 

        t.transform.rotation.y = quat[1] 

        t.transform.rotation.z = quat[2] 

        t.transform.rotation.w = quat[3] 

        # Send the transformation 

        self.tf_br.sendTransform(t)
        
      
        
    def send_left_wheel_link_tf(self, theta_l_act): 

        t = TransformStamped() 

 

        t.header.stamp = rospy.Time.now() 

        t.header.frame_id = "base_link_efk"

        t.child_frame_id = "left_wheel_rviz" 

        #Copy data from the received pose to the tf  

        t.transform.translation.x = 0.05 

        t.transform.translation.y = 0.09 

        t.transform.translation.z = 0.0 

        #quat = quaternion_from_euler(np.pi/2.0, 0.0, np.pi/2.0) 
        quat = quaternion_from_euler(0.0, theta_l_act, 0.0) 

        t.transform.rotation.x = quat[0] 

        t.transform.rotation.y = quat[1] 

        t.transform.rotation.z = quat[2] 

        t.transform.rotation.w = quat[3] 

        # Send the transformation 

        self.tf_br.sendTransform(t)
        
        
        
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
        marker.mesh_resource = "package://aruco_markers/rviz/MCR2_1000_0_Puzzlebot.stl"
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

    rospy.init_node('localisation_ekf')  

    EKFClass()
    
