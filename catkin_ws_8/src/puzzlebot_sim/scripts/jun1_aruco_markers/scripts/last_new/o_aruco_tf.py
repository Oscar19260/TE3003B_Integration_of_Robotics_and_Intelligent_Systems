#!/usr/bin/env python  
import rospy
import math
import tf_conversions
import tf2_ros
from geometry_msgs.msg import Vector3, TransformStamped
from std_msgs.msg import Float64MultiArray, Int16
from tf.transformations import euler_from_quaternion
from fiducial_msgs.msg import FiducialTransformArray

#This class will do the following: 
#   subscribe to the /fiducial_transforms topic  
#   transform the aruco marker rotation info from quaternion to euler angles 
#   print said angles
#   make a transformation from the /camera frame to the marker frame  
class ArucoClass():
    def __init__(self):
        # ----- PUBLISHERS  -----
        self.id_pub = rospy.Publisher("/marker_id", Int16, queue_size=1)
        self.position_pub = rospy.Publisher("/marker_pos", Float64MultiArray, queue_size=1)
        self.rotation_pub = rospy.Publisher("/marker_rot", Float64MultiArray, queue_size=1)

        # [flag, x, y]
        self.aruco_pub = rospy.Publisher("/aruco_pos", Vector3, queue_size = 1)
        # [val1, val2, 0]
        self.r_vect_pub = rospy.Publisher("/r_vect", Vector3, queue_size = 1)
        # [dist, ang, 0]
        self.robot_aruco_pub = rospy.Publisher("/robot_aruco", Vector3, queue_size = 1)

        # ----- SUBSCRIBERS -----
        rospy.Subscriber("/fiducial_transforms", FiducialTransformArray, self.fid_tf_cb)

        # ----- CONSTANTS   -----
        self.delta_t = 0.02 
        #   a   b   c   d   e   f   g   h   i   j   k   l   m   n 
        # 703 704 705 706 707 708 709 710 711 712 713 714 715 716 
        #   o   p   q   r   s   t   u   v   q   x   y   z
        # 717 718 719 720 721 722 723 724 725 726 727 728
        self.aruco_dict = {
            "701": [0.48, 3.15],
            "702": [2.29, 2.85],
            "703": [1.04, 4.65],
            "704": [1.43, 2.45],
            "705": [1.20, 0.98]
        }

        # ----- VARIABLES   -----
        rate = rospy.Rate(int(1.0/self.delta_t))
        self.aruco = FiducialTransformArray()
        self.detected_id, self.position_array, self.rotation_array = Int16(), Float64MultiArray(), Float64MultiArray()

        self.aruco_info = Vector3()
        self.flag = 0.0
        self.aruco_x = 0.0
        self.aruco_y = 0.0

        self.aruco_r = Vector3()
        self.r_vect = [0.01, 0.02, 0.0]

        self.aruco_est = Vector3()
        self.aruco_dist = 0.0
        self.aruco_ang = 0.0
        # ----- MAIN LOOP   -----
        while not rospy.is_shutdown():
            try:

                # Aruco Detected!
                msg = self.aruco
                
                # Extract info from marker
                self.flag = 1.0
                id = msg.transforms[0].fiducial_id
                print(id)
                if id == 701:
                    self.aruco_x, self.aruco_y = self.aruco_dict["701"][0], self.aruco_dict["701"][1]
                elif id == 702:
                    self.aruco_x, self.aruco_y = self.aruco_dict["702"][0], self.aruco_dict["702"][1]
                elif id == 703:
                    self.aruco_x, self.aruco_y = self.aruco_dict["703"][0], self.aruco_dict["703"][1]
                elif id == 704:
                    self.aruco_x, self.aruco_y = self.aruco_dict["704"][0], self.aruco_dict["704"][1]
                elif id == 705:
                    self.aruco_x, self.aruco_y = self.aruco_dict["705"][0], self.aruco_dict["705"][1]

                position = [round(msg.transforms[0].transform.translation.x, 5),
                            round(msg.transforms[0].transform.translation.y, 5),
                            round(msg.transforms[0].transform.translation.z, 5)]

                self.aruco_ang = math.atan2(position[0], position[2])
                self.aruco_dist = math.sqrt(position[0]**2 + position[2]**2)
                quaternion = [round(msg.transforms[0].transform.rotation.x, 5),
                              round(msg.transforms[0].transform.rotation.y, 5),
                              round(msg.transforms[0].transform.rotation.z, 5),
                              round(msg.transforms[0].transform.rotation.w, 5)]
                
                # Transform Quat to Euler
                roll, pitch, yaw = euler_from_quaternion(quaternion)
                rotation = [round(roll, 5), round(pitch, 5), round(yaw, 5)]
                self.detected_id.data = id
                self.position_array.data = position
                self.rotation_array.data = rotation

                # Check ID
                # if id == self.aaron_id:
                print("\n========== INFO INIT ==========")
                print("ID: "+ str(id))
                print("Position [X, Y, Z]: " + str(position))
                print("Rotation [X, Y, Z]: " + str(rotation))
                print("========== INFO END  ==========")

                # Assemble vectors for EKF
                self.aruco_info.x, self.aruco_info.y, self.aruco_info.z = self.flag, self.aruco_x, self.aruco_y
                self.aruco_r.x, self.aruco_r.y, self.aruco_r.z = self.r_vect[0], self.r_vect[1], self.r_vect[2]
                self.aruco_est.x, self.aruco_est.y, self.aruco_est.z = self.aruco_dist, self.aruco_ang, 0.0
                
                # Publish info
                self.aruco_pub.publish(self.aruco_info)
                self.r_vect_pub.publish(self.aruco_r)
                self.robot_aruco_pub.publish(self.aruco_est)
                self.id_pub.publish(self.detected_id)
                self.position_pub.publish(self.position_array)
                self.rotation_pub.publish(self.rotation_array)
                self.transform("marker_" + str(id), position, rotation)
            except Exception as error:
                self.flag = 0.0
                self.aruco_x = 0.0
                self.aruco_y = 0.0
                self.aruco_info.x, self.aruco_info.y, self.aruco_info.z = self.flag, self.aruco_x, self.aruco_y
                print(error) 
            rate.sleep()
    
    def fid_tf_cb(self, msg):
        self.aruco = msg
    
    def transform(self, marker_id, position, rotation):
        br = tf2_ros.TransformBroadcaster()
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()

        t.header.frame_id = "camera"
        t.child_frame_id = marker_id
        t.transform.translation.x = position[0]
        t.transform.translation.y = position[1]
        t.transform.translation.z = position[2]
        q = tf_conversions.transformations.quaternion_from_euler(rotation[0], rotation[1], rotation[2])
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        br.sendTransform(t)

# ----- MAIN PROGRAM -----
if __name__ == "__main__":  
    rospy.init_node("aruco_tf")  
    ArucoClass()  
