#!/usr/bin/env python3


from sensor_msgs.msg import Image
from std_msgs.msg import Float64
from std_srvs.srv import Empty, EmptyResponse
from cv_bridge import CvBridge, CvBridgeError
import message_filters
import cv2
import rospy
import numpy as np
import PIL

NODE_NAME = "lane_detection_realsense_node"
SUB_TOPIC_color = "camera/color/image_raw"
SUB_TOPIC_depth = "camera/aligned_depth_to_color/image_raw"
PUB_TOPIC = "debug_image"
PUB_SETPOINT_TOPIC = "lane_curvature"
PUB_STATE_TOPIC = "lane_offset"
RESET_SERVICE = "reset"
QUEUE_SIZE = 1


class LaneDetectionNode:
    def __init__(self, node_name, sub_topic_color, sub_topic_depth, pub_topic, pub_setpoint_topic, pub_state_topic, reset_service):
        self.bridge = CvBridge()

        # Publishers
        self.image_pub = rospy.Publisher(pub_topic, Image, queue_size=QUEUE_SIZE)
        self.setpoint_pub = rospy.Publisher(pub_setpoint_topic, Float64, queue_size=QUEUE_SIZE)
        self.state_pub = rospy.Publisher(pub_state_topic, Float64, queue_size=QUEUE_SIZE)
        self.send_timer = rospy.Timer(rospy.Duration(0.1), self.sendCallback)

        # reset camera service
        self.reset_srv = rospy.Service(reset_service, Empty, self.reset_callback)
        self.reset_tracking = False


        # subscribe to RGB channel and depth (aligned to rgb) channel
        self.color_sub = rospy.Subscriber(sub_topic_color, Image,self.colorCallback) # subscriber for color topic
        self.depth_sub = rospy.Subscriber(sub_topic_depth, Image,self.depthCallback) # subscriber for depth topic


        # Crop Parameters
        self.above_value = rospy.get_param("/autonomous_driving/lane_detection_node/above", 0.58)
        self.below_value = rospy.get_param("/autonomous_driving/lane_detection_node/below", 0.1)
        self.side_value = rospy.get_param("/autonomous_driving/lane_detection_node/side", 0.3)

        # Lane Tracking Parameters
        self.deviation = rospy.get_param("/autonomous_driving/lane_detection_node/deviation", 5)
        self.border = rospy.get_param("/autonomous_driving/lane_detection_node/border", 0)

        # Canny Parameters
        self.threshold_low = rospy.get_param("/autonomous_driving/lane_detection_node/threshold_low", 50)
        self.threshold_high = rospy.get_param("/autonomous_driving/lane_detection_node/threshold_high", 150)
        self.aperture = rospy.get_param("/autonomous_driving/lane_detection_node/aperture", 3)

        self.cv_image_col = []
        self.cv_image_depth = []
        self.track_lane_width = 1.067 # meters, width of a lane on the track
        self.look_ahead_dist = 0.75 # meters, distance ahead from which lane is measured
        self.K = np.array([[304.5852355957031, 0.0, 213.36288452148438],[0.0, 304.6020202636719, 120.16206359863281],[0.0, 0.0, 1.0]])
        self.Kinv = np.linalg.inv(self.K)
        rospy.spin()


    def colorCallback(self, color_image):
        try:
            self.cv_image_col = self.bridge.imgmsg_to_cv2(color_image, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)

    def depthCallback(self,depth_image):
        try:
            self.cv_image_depth = self.bridge.imgmsg_to_cv2(depth_image, "16UC1")
        except CvBridgeError as e:
            rospy.logerr(e)

    def sendCallback(self,msg):

        # grayscale
        hsvd = cv2.cvtColor(self.cv_image_col, cv2.COLOR_BGR2HSV)
        h, s, gray = cv2.split(hsvd)

        # blur
        #blurred = cv2.GaussianBlur(gray, (self.deviation, self.deviation), self.border)
        #blurred = self.img_prep.blur(gray, , self.border)

        # canny
        canny = cv2.Canny(gray, self.threshold_low, self.threshold_high, self.aperture)

        # map into 3D coordinates
        #print(self.cv_image_depth[3][8])
        #u, v = np.mgrid[:gray.shape[1], :gray.shape[0]]
        #x_coords = self.cv_image_depth / self.K[0][0] * (self.K[0][2] - u)
        #y_coords = self.cv_image_depth / self.K[1][1] * (self.K[1][2] - v)


        # crop image
        above = 0.6
        below = 0.25
        side = 0.0
        height, width = canny.shape
        cropped = canny[int((height*above)):height - int((height*below)), int((width*side)):width - int((width*side))]
        #
        # # histogram = np.sum(canny[canny.shape[0]//2:,:], axis=0)
        # histogram = np.sum(cropped, axis=0)
        #
        # # find peaks of left and right halves
        # midpoint = int(histogram.shape[0]/2)
        # leftx_base = np.argmax(histogram[:midpoint])
        # rightx_base = np.argmax(histogram[midpoint:]) + midpoint
        #
        #
        # Lane Detection
        canny = cv2.cvtColor(cropped, cv2.COLOR_GRAY2BGR)

        lines = cv2.HoughLines(cropped,1,np.pi/180,35)
        slope = []
        if lines is not None:
            print(len(lines))
            for i in range(0,len(lines)):
                rho = lines[i][0][0]
                print(rho)
                theta = lines[i][0][1]
                a = np.cos(theta)
                b = np.sin(theta)
                x0 = a*rho
                y0 = b*rho
                x1 = int(x0 + 1000*(-b))
                y1 = int(y0 + 1000*(a))
                x2 = int(x0 - 1000*(-b))
                y2 = int(y0 - 1000*(a))
                slope.append((y2-y1)/(x2-x1))
                if slope[i]<0.39 and slope[i]>0.24:
                    cv2.line(canny,(x1,y1),(x2,y2),(255,0,255),2)
                if slope[i]<-0.24 and slope[i]>-0.39:
                    cv2.line(canny,(x1,y1),(x2,y2),(0,255,255),2)
        #print(slope)
        # print(min(slope))


        # calculate lane_curvature
        left_curv = 0.3
        right_curv = 0.28
        avg_curv = (left_curv+right_curv)/2.0

        # calculate bias from center
        deviation = 0.2


        if self.reset_tracking is True:
            self.reset_tracking = False

        # publish to pid
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(canny, "bgr8"))
            self.state_pub.publish(deviation)
            self.setpoint_pub.publish(avg_curv)
        except CvBridgeError as e:
            rospy.logerr(e)


    def reset_callback(self, req):
        rospy.loginfo("Reset Lanetracking")
        self.reset_tracking = True
        return EmptyResponse()


def main():
    # init node
    rospy.init_node(NODE_NAME, anonymous=True)
    try:
        LaneDetectionNode(NODE_NAME, SUB_TOPIC_color, SUB_TOPIC_depth, PUB_TOPIC, PUB_SETPOINT_TOPIC, PUB_STATE_TOPIC, RESET_SERVICE)
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down node %s", NODE_NAME)

if __name__ == '__main__':
    main()
