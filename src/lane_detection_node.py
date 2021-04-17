#!/usr/bin/env python3


from sensor_msgs.msg import Image
from std_msgs.msg import Float64
from std_srvs.srv import Empty, EmptyResponse
from cv_bridge import CvBridge, CvBridgeError
import cv2
import rospy
import numpy as np
import PIL

NODE_NAME = "lane_detection_node"
SUB_TOPIC = "camera/color/image_raw"
SUB_BASE_THROTTLE_TOPIC = "baseThrottle"
PUB_TOPIC = "debug_image"
PUB_SETPOINT_TOPIC = "lane_curvature"
PUB_STATE_TOPIC = "lane_offset"
RESET_SERVICE = "reset"
QUEUE_SIZE = 1

DEFAULT_LANE_WIDTH = 20
DEFAULT_SEGMENT_START = 6
DEFAULT_SEGMENT_AMOUNT = 1


class LaneDetectionNode:
    def __init__(self, node_name, sub_topic, sub_base_throttle_topic, pub_topic, pub_setpoint_topic, pub_state_topic, pub_throttle_topic, reset_service):
        self.bridge = CvBridge()

        # Publisher
        self.image_pub = rospy.Publisher(pub_topic, Image, queue_size=QUEUE_SIZE)
        self.setpoint_pub = rospy.Publisher(pub_setpoint_topic, Float64, queue_size=QUEUE_SIZE)
        self.state_pub = rospy.Publisher(pub_state_topic, Float64, queue_size=QUEUE_SIZE)

        self.reset_srv = rospy.Service(reset_service, Empty, self.reset_callback)
        self.reset_tracking = False

        rospy.init_node(node_name, anonymous=True)

        self.image_sub = rospy.Subscriber(sub_topic, Image, self.callback)


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
        self.track_lane_width = 1 # Parameters
        rospy.spin()

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)

        # ---------------------------- Perspective Transform --------------------------
        # 4 points from image that we know to be rectangular in birds eye view (i.e. top and bottom of l-r lanes)
        # see https://www.pyimagesearch.com/2014/08/25/4-point-opencv-getperspective-transform-example/
        # src = [450, 450], [800, 450], [cv_image.shape[1]-50, cv_image.shape[0]],  [150, cv_image.shape[0]]
        #line_dst_offset = 200
        #src = [595, 500], [685, 500],[1110,img_b.shape[0]],[220, img_b.shape[0]]
        #dst = [src[3][0] + line_dst_offset, 0], \
        #      [src[2][0] - line_dst_offset, 0], \
    #          [src[2][0] - line_dst_offset, src[2][1]], \
    #          [src[3][0] + line_dst_offset, src[3][1]]
    #    src = np.float32([src])
    #    dst = np.float32([dst])
    #    warped = cv2.warpPerspective(cv_image, cv2.getPerspectiveTransform(src, dst),dsize=cv_image.shape[0:2][::-1], flags=cv2.INTER_LINEAR)


        # grayscale
        hsvd = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        h, s, gray = cv2.split(hsvd)

        # blur
        blurred = cv2.GaussianBlur(gray, (self.deviation, self.deviation), self.border)
        #blurred = self.img_prep.blur(gray, , self.border)

        # canny
        canny = cv2.Canny(blurred, self.threshold_low, self.threshold_high, self.aperture)

        heigth, width = canny.shape
        if width == 63:  # TODO dirty hack
            cv2.line(canny, (0, 4/2), (18/2, heigth), (0, 0, 0), 2)
            cv2.line(canny, (width, 4/2), (width - 18/2, heigth), (0, 0, 0), 2)
        else:
            cv2.line(canny, (0, 4), (18, heigth), (0, 0, 0), 2)
            cv2.line(canny, (width, 4), (width - 18, heigth), (0, 0, 0), 2)

        # crop image
        above = 0.0
        below = 0.0
        side = 0.0
        height, width = canny.shape
        cropped = canny[int((height*above)):height - int((height*below)), int((width*side)):width - int((width*side))]





        # Lane Detection
        canny = cv2.cvtColor(cropped, cv2.COLOR_GRAY2BGR)

        nwindows=9 # number of windows to divide frame into
        margin=150 # horizontal margin that lane must fall within
        minpix = 1 # minimum number of pixels within a window
        left_fit_= np.empty(3)
        right_fit_ = np.empty(3)
        histogram = np.sum(canny[canny.shape[0]//2:,:], axis=0)

        # find peaks of left and right halves
        midpoint = int(histogram.shape[0]/2)
        leftx_base = np.argmax(histogram[:midpoint])
        rightx_base = np.argmax(histogram[midpoint:]) + midpoint

        # Set height of windows
        window_height = np.int(canny.shape[0]/nwindows)

        # Identify the x and y positions of all nonzero pixels in the image
        nonzero = canny.nonzero()
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])

        # Current positions to be updated for each window
        leftx_current = leftx_base
        rightx_current = rightx_base

        # Create empty lists to receive left and right lane pixel indices
        left_lane_inds = []
        right_lane_inds = []

        left_a, left_b, left_c = [],[],[]
        right_a, right_b, right_c = [],[],[]

        # Step through the windows one by one
        for window in range(nwindows):
            # Identify window boundaries in x and y (and right and left)
            win_y_low = canny.shape[0] - (window+1)*window_height
            win_y_high = canny.shape[0] - window*window_height
            win_xleft_low = leftx_current - margin
            win_xleft_high = leftx_current + margin
            win_xright_low = rightx_current - margin
            win_xright_high = rightx_current + margin
            # # Draw the windows on the visualization image
            # if draw_windows == True:
            #     cv2.rectangle(out_img,(win_xleft_low,win_y_low),(win_xleft_high,win_y_high),
            #     (100,255,255), 3)
            #     cv2.rectangle(out_img,(win_xright_low,win_y_low),(win_xright_high,win_y_high),
            #     (100,255,255), 3)

            # Identify the nonzero pixels in x and y within the window
            good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
            (nonzerox >= win_xleft_low) &  (nonzerox < win_xleft_high)).nonzero()[0]
            good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
            (nonzerox >= win_xright_low) &  (nonzerox < win_xright_high)).nonzero()[0]
            # Append these indices to the lists
            left_lane_inds.append(good_left_inds)
            right_lane_inds.append(good_right_inds)
            # If you found > minpix pixels, recenter next window on their mean position
            if len(good_left_inds) > minpix:
                leftx_current = np.int(np.mean(nonzerox[good_left_inds]))
            if len(good_right_inds) > minpix:
                rightx_current = np.int(np.mean(nonzerox[good_right_inds]))


        # Concatenate the arrays of indices
        left_lane_inds = np.concatenate(left_lane_inds)
        right_lane_inds = np.concatenate(right_lane_inds)

        # Extract left and right line pixel positions
        leftx = nonzerox[left_lane_inds]
        lefty = nonzeroy[left_lane_inds]
        rightx = nonzerox[right_lane_inds]
        righty = nonzeroy[right_lane_inds]

        # Fit a second order polynomial to each lane
        left_fit = np.polyfit(lefty, leftx, 2)
        right_fit = np.polyfit(righty, rightx, 2)

        # calculate lane_curvature
        left_curv = 0.3
        right_curv = 0.28
        avg_curv = (left_curv+right_curv)/2.0

        # calculate bias from center
        deviation = 0.2


        if self.reset_tracking is True:
            self.init_lanemodel()
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
    try:
        LaneDetectionNode(NODE_NAME, SUB_TOPIC, SUB_BASE_THROTTLE_TOPIC, PUB_TOPIC, PUB_SETPOINT_TOPIC, PUB_STATE_TOPIC, PUB_THROTTLE_TOPIC, RESET_SERVICE)
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down node %s", NODE_NAME)

if __name__ == '__main__':
    main()
