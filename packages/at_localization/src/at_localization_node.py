#!/usr/bin/env python3
import numpy as np
import yaml
import math
import rospy
import cv2
import tf
import tf2_ros
#from numba import jit, cuda
from duckietown.dtros import DTROS, NodeType
from dt_apriltags import Detector
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import CompressedImage,CameraInfo
from cv_bridge import CvBridge, CvBridgeError
class ATLocalizationNode(DTROS):

    def __init__(self, node_name):
        """Encoder Localization Node
        State estimation for April tag, assuming duckiebot at origin
        """

        # Initialize the DTROS parent class
        super(ATLocalizationNode, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)
        self.veh_name = rospy.get_namespace().strip("/")

        self.camerainfo=self.load_intrinsics()
        
        #initialize transformation matrixs
        pi=math.pi
        at_height=0.105
        T_BC=np.eye(4)#transformation matrix from camera frame to baselink frame
        T_BC[0:3,3]=[0.0582,0,0.1072]
        T_BC[0:3,0:3]=[[math.cos(15/180*pi),0,math.sin(15/180*pi)],
                       [0,1,0],
                       [-math.sin(15/180*pi),0,math.cos(15/180*pi)]]
        self.T_CB=np.linalg.inv(T_BC)
        
        T_CC1=np.eye(4)#transformation matrix from camera frame for at detection to camera frame
        T_CC1[0:3,0:3]=[[0,0,1],
                        [-1,0,0],
                        [0,-1,0]]
        self.T_C1C=np.linalg.inv(T_CC1)
        
        self.T_C1A1=np.eye(4)#transformation matrix from april tag frame for at detection to camera frame for at detection, given by at_detector
        
        T_A1A=np.eye(4)#transformation matrix from at frame to at frame for at detection
        T_A1A[0:3,0:3]=[[0,1,0],
                        [0,0,-1],
                        [-1,0,0]]
        self.T_AA1=np.linalg.inv(T_A1A)
        
        T_AM=np.eye(4)#transformation matrix from map frame  to at frame
        T_AM[0:3,3]=[0,0,-at_height]
        self.T_MA=np.linalg.inv(T_AM)
        
        #april tag detector
        self.at_detector = Detector(families='tag36h11',
                       nthreads=4,
                       quad_decimate=4.0,
                       quad_sigma=0.0,
                       refine_edges=1,
                       decode_sharpening=0.25,
                       debug=0)
        
        self.bridge=CvBridge()       
        
        # tf broadcaster
        self.br = tf2_ros.TransformBroadcaster()
        self.static_br = tf2_ros.StaticTransformBroadcaster()
                
        self.initialized=False
        # Publishers
        self.pub_at_localization = rospy.Publisher('at_localization_node/transform',TransformStamped,queue_size=1)
        
        self.sub_img=rospy.Subscriber('camera_node/image/compressed', CompressedImage, self.cb_at_localization)
        
        rospy.loginfo("Initialized")
        
        
    
    def cb_at_localization(self,msg):
        """ Compute and publish a transformstamped massage.
        """
        #detect april tag
        time=msg.header.stamp
        img=self.readImage(msg)
        #img=self.rectify(img)
        img_gray=cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        
        cameraMatrix = np.array(self.camerainfo.K).reshape((3,3))
        camera_params = (cameraMatrix[0,0],cameraMatrix[1,1],cameraMatrix[0,2],cameraMatrix[1,2])

        tag = self.at_detector.detect(img_gray, True , camera_params, 0.065)
        if tag:
            tag=tag[0]
            #rospy.loginfo(tag)
            if not self.initialized:
                # broadcast static transform
                self.br_static()
                self.initialized=True
            self.T_C1A1[0:3,3]=(tag.pose_t).reshape((3,))
            self.T_C1A1[0:3,0:3]=tag.pose_R
            T_A1C1=np.linalg.inv(self.T_C1A1)
            
            T_AC=np.linalg.multi_dot([self.T_AA1,T_A1C1,self.T_C1C])
            transform_msg=self.matrix_to_msg(T_AC,'apriltag','camera',time)
            #rospy.loginfo(transform_msg)
            self.br.sendTransform(transform_msg)
            
            T_MB=np.linalg.multi_dot([self.T_MA,T_AC,self.T_CB])
            transform_msg=self.matrix_to_msg(T_MB,'map','at_baselink',time)
            self.pub_at_localization.publish(transform_msg)
            #self.br.sendTransformMessage(transform_msg)
        
    def br_static(self):
        #broadcast static TF: map-apriltag        
        transform_msg1=self.matrix_to_msg(self.T_MA,'map','apriltag')

        #broadcast static TF: camera-at_baselink  
        transform_msg2=self.matrix_to_msg(self.T_CB,'camera','at_baselink')
        
        self.static_br.sendTransform([transform_msg1,transform_msg2])
        
    def matrix_to_msg(self,T,frame_id,child_frame_id,time=rospy.Time()):
        #convert 4*4 transform matrix to transformstamped message
        Transform = TransformStamped()
        Transform.header.stamp=time
        Transform.header.frame_id = frame_id
        Transform.child_frame_id = child_frame_id
        
        Transform.transform.translation.x = T[0,3]
        Transform.transform.translation.y = T[1,3]
        Transform.transform.translation.z = T[2,3]
        
        quaternion=tf.transformations.quaternion_from_matrix(T)
        
        Transform.transform.rotation.x = quaternion[0]
        Transform.transform.rotation.y = quaternion[1]
        Transform.transform.rotation.z = quaternion[2]
        Transform.transform.rotation.w = quaternion[3]
        
        return Transform
    
    def load_intrinsics(self):
        # load intrinsic calibration
        cali_file_folder = '/data/config/calibrations/camera_intrinsic/'
        cali_file = cali_file_folder + rospy.get_namespace().strip("/") + ".yaml"
        calib_data=self.readYamlFile(cali_file)
        cam_info = CameraInfo()
        cam_info.width = calib_data['image_width']
        cam_info.height = calib_data['image_height']
        cam_info.K = calib_data['camera_matrix']['data']
        cam_info.D = calib_data['distortion_coefficients']['data']
        cam_info.R = calib_data['rectification_matrix']['data']
        cam_info.P = calib_data['projection_matrix']['data']
        cam_info.distortion_model = calib_data['distortion_model']
        return cam_info
        
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
        
    def readImage(self, msg_image):
        """
            Convert images to OpenCV images
            Args:
                msg_image (:obj:`CompressedImage`) the image from the camera node
            Returns:
                OpenCV image
        """
        try:
            cv_image = self.bridge.compressed_imgmsg_to_cv2(msg_image)
            return cv_image
        except CvBridgeError as e:
            self.log(e)
            return []
        
#    @jit(target ="cuda") 
    def rectify(self,img):#undistort image using calibration files
        W = self.camerainfo.width
        H = self.camerainfo.height
        mapx = np.ndarray(shape=(H, W, 1), dtype='float32')
        mapy = np.ndarray(shape=(H, W, 1), dtype='float32')
#        newCameraMatrix = cv2.getOptimalNewCameraMatrix(np.array(self.camerainfo.K).reshape((3,3)), 
#                                                np.array(self.camerainfo.D), 
#                                                (W, H), 
#                                                1.0)
        mapx, mapy = cv2.initUndistortRectifyMap(np.array(self.camerainfo.K).reshape((3,3)), 
                                                 np.array(self.camerainfo.D), 
                                                 np.array(self.camerainfo.R).reshape((3,3)),
                                                 None, 
                                                 (W, H),
                                                 cv2.CV_32FC1, mapx, mapy)
        #cv_image_rectified = np.empty_like(img)
        res = cv2.remap(img, mapx, mapy, cv2.INTER_LINEAR)
        return res    
    
         
if __name__ == '__main__':
    node = ATLocalizationNode(node_name='at_localization_node')
    # Keep it spinning to keep the node alive
    rospy.spin()