#!/usr/bin/env python  

import rospy  

from geometry_msgs.msg import PoseStamped 

from visualization_msgs.msg import Marker 

from tf.transformations import quaternion_from_euler 

# Because of transformations 

from geometry_msgs.msg import TransformStamped 

import tf2_ros 

 

class PuzzlebotTfClass():  

    def __init__(self):  

        rospy.Subscriber("pose_sim", PoseStamped, self.pose_sim_cb) 

        self.marker_pub = rospy.Publisher("puzzlebot_marker", Marker, queue_size = 1) 

        self.tf_br = tf2_ros.TransformBroadcaster() 

        self.robot_pose = PoseStamped() 

        self.robot_pose.pose.orientation.w = 1.0 #this is necessary to avoid errors with the quaternion. 

        rate = rospy.Rate(50) # The rate of the while loop 

        while not rospy.is_shutdown(): 

            self.send_base_link_tf(self.robot_pose) 

            ######## Publish a marker to rviz ######### 

            marker = self.fill_marker(self.robot_pose) 

            self.marker_pub.publish(marker) 

            rate.sleep() 

 

    def pose_sim_cb(self, msg): 

        self.robot_pose = msg 

 

    def send_base_link_tf(self, pose_stamped=PoseStamped()): 

        # This receives the robot's pose and broadcast a transformation. 

         

        t = TransformStamped() 

 

        t.header.stamp = rospy.Time.now() 

        t.header.frame_id = "odom" 

        t.child_frame_id = "base_link" 

        #Copy data from the received pose to the tf  

        t.transform.translation.x = pose_stamped.pose.position.x 

        t.transform.translation.y = pose_stamped.pose.position.y 

        t.transform.translation.z = pose_stamped.pose.position.z 

         

        t.transform.rotation.x = pose_stamped.pose.orientation.x 

        t.transform.rotation.y = pose_stamped.pose.orientation.y 

        t.transform.rotation.z = pose_stamped.pose.orientation.z 

        t.transform.rotation.w = pose_stamped.pose.orientation.w 

        # Send the transformation 

        self.tf_br.sendTransform(t) 

         

   

    def fill_marker(self, pose_stamped=PoseStamped()): 

        marker = Marker() 

        marker.header.frame_id = "base_link" 

        marker.header.stamp = rospy.Time.now() 

        # set shape, Arrow: 0; Cube: 1 ; Sphere: 2 ; Cylinder: 3; Mesh: 10 

        marker.type = 10 

        marker.id = 0 

 

        # Use the stl file MCR2_1000_13_Chassis.stl,  

        marker.mesh_resource = "package://puzzlebot_sim/meshes/MCR2_1000_0_Puzzlebot.stl" 

 

        # Set the scale of the marker 

        marker.scale.x = 1 

        marker.scale.y = 1 

        marker.scale.z = 1 

 

        # Set the color 

        marker.color.r = 1.0 

        marker.color.g = 0.0 

        marker.color.b = 0.0 

        marker.color.a = 1.0 

 

        # Set the pose of the marker 

        marker.pose.position.x = 0.0 

        marker.pose.position.y = 0.0 

        marker.pose.position.z = 0.0 

        marker.pose.orientation.x = 0.0 

        marker.pose.orientation.y = 0.0 

        marker.pose.orientation.z = 0.0 

        marker.pose.orientation.w = 1.0 

        return marker 

 

 

############################### MAIN PROGRAM ####################################  

if __name__ == "__main__":  

    # first thing, init a node! 

    rospy.init_node('puzzlebot_tf_broadcaster')  

    PuzzlebotTfClass()
    
