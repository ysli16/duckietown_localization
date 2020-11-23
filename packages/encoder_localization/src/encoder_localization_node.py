#!/usr/bin/env python3
import numpy as np
import yaml
import math
import rospy
import tf
import tf2_ros
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import WheelEncoderStamped
from geometry_msgs.msg import TransformStamped
from encoder_localization.srv import FrameCalibration
#,FrameCalibrationResponse
class EncoderLocalizationNode(DTROS):

    def __init__(self, node_name):
        """Encoder Localization Node
        State estimation for duckiebot using wheel encoder data
        """

        # Initialize the DTROS parent class
        super(EncoderLocalizationNode, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)
        self.veh_name = rospy.get_namespace().strip("/")

        # Load static parameters
        self.load_static_param()
        self.total_tick=135
        rospy.loginfo('got static parameters')
        
        self.tick_left_last=0
        self.tick_right_last=0
        self.tick_left_current=0
        self.tick_right_current=0
        self.last_time=rospy.get_rostime()
        self.position=np.array([0.15,0,0])#initial position of duckiebot
        self.angle=math.pi #initial angular
        
        # Subscribing to the wheel encoders
        self.sub_encoder_ticks_left = rospy.Subscriber(f'/{self.veh_name}/left_wheel_encoder_node/tick',WheelEncoderStamped,self.cb_encoder_data,"left")
        self.sub_encoder_ticks_right = rospy.Subscriber(f'/{self.veh_name}/right_wheel_encoder_node/tick',WheelEncoderStamped,self.cb_encoder_data,"right")
        
        # Publishers
        self.pub_encoder_localization = rospy.Publisher('encoder_localization_node/transform',TransformStamped,queue_size=1)
        
        #server for calibration
        self.server = rospy.Service('encoder_localization_node/frame_calibration', FrameCalibration, self.frame_calibration)
        
        # tf broadcaster
        self.br = tf2_ros.TransformBroadcaster()
        
        #initial TF
        Transform=self.pose_to_msg()
        self.pub_encoder_localization.publish(Transform)
        self.br.sendTransform(Transform)
        
        #initialize timer 
        self.timer=rospy.Timer(rospy.Duration(1/30), self.encoder_localization)
        
        rospy.loginfo("Initialized")
        
        

    def cb_encoder_data(self, msg,arg):
        """ Record latest tick data of each wheel.
        """
        self.last_time=msg.header.stamp
        if arg=="left":
            
            self.tick_left_current=msg.data
            
            if self.tick_left_last==0:
                self.tick_left_last=self.tick_left_current
        else:
            
            self.tick_right_current=msg.data
            
            if self.tick_right_last==0:
                self.tick_right_last=self.tick_right_current


    
    def encoder_localization(self,event):
        """ Compute and publish a transformstamped massage.
        """
        
        if not self.tick_left_last==self.tick_left_current or self.tick_right_last==self.tick_right_current:#if robot moved, update pose
            
            d_left=2*math.pi*self.radius*(self.tick_left_current-self.tick_left_last)/self.total_tick
            d_right=2*math.pi*self.radius*(self.tick_right_current-self.tick_right_last)/self.total_tick
            distance=(d_left+d_right)/2
            theta=(d_right-d_left)/self.baseline
            self.angle+=theta
            self.position[0]=self.position[0]+distance*math.cos(self.angle)
            self.position[1]=self.position[1]+distance*math.sin(self.angle)
            
        Transform=self.pose_to_msg()
        
        self.pub_encoder_localization.publish(Transform)
        self.br.sendTransform(Transform)
        
        self.tick_left_last=self.tick_left_current
        self.tick_right_last=self.tick_right_current
        
    def pose_to_msg(self):
        Transform = TransformStamped()
        Transform.header.stamp=self.last_time
        Transform.header.frame_id = 'map'
        Transform.child_frame_id = 'encoder_baselink'
        
        Transform.transform.translation.x = self.position[0]
        Transform.transform.translation.y = self.position[1] 
        Transform.transform.translation.z = self.position[2]
        
        quaternion=tf.transformations.quaternion_from_euler(0, 0, self.angle,'rxyz')
        
        Transform.transform.rotation.x = quaternion[0]
        Transform.transform.rotation.y = quaternion[1]
        Transform.transform.rotation.z = quaternion[2]
        Transform.transform.rotation.w = quaternion[3]
        
        return Transform
    
    def msg_to_pose(self,Transform):
        self.position[0]=Transform.transform.translation.x 
        self.position[1]=Transform.transform.translation.y 
        self.position[2]=Transform.transform.translation.z
        
        qx=Transform.transform.rotation.x 
        qy=Transform.transform.rotation.y 
        qz=Transform.transform.rotation.z 
        qw=Transform.transform.rotation.w
        
        Euler=tf.transformations.euler_from_quaternion([qx,qy,qz,qw])
        
        self.angle=Euler[2]
        
    def load_static_param(self):
        """ Load static parameters radius and baseline from kinematics calibration file
        """
        cali_file_folder = '/data/config/calibrations/kinematics/'
        cali_file = cali_file_folder + rospy.get_namespace().strip("/") + ".yaml"
        calib_data=self.readYamlFile(cali_file)
        self.radius=calib_data['radius']
        self.baseline=calib_data['baseline']
        
    def readYamlFile(self,fname):
        """
            Reads the 'fname' yaml file and returns a dictionary with its input.

            You will find the calibration files you need in:
            `/data/config/calibrations/`
        """
        with open(fname, 'r') as in_file:
            try:
                yaml_dict = yaml.load(in_file)
                return yaml_dict
            except yaml.YAMLError as exc:
                self.log("YAML syntax error. File: %s fname. Exc: %s"
                         %(fname, exc), type='fatal')
                rospy.signal_shutdown()
                return
            
    def frame_calibration(self,req):
        transform_at=req.transform_at
        self.msg_to_pose(transform_at)
        new_transform=self.pose_to_msg()
        self.pub_encoder_localization.publish(new_transform)
        self.br.sendTransform(new_transform)
        return True
                
if __name__ == '__main__':
    node = EncoderLocalizationNode(node_name='encoder_localization_node')
    # Keep it spinning to keep the node alive
    rospy.spin()
