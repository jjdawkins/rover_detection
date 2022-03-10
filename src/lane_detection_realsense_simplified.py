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
        self.look_ahead_dists = np.array([0.75,1.5,3]) # meters, distance ahead from which lane is measured
        self.K = np.array([[304.5852355957031, 0.0, 213.36288452148438],[0.0, 304.6020202636719, 120.16206359863281],[0.0, 0.0, 1.0]])
        self.Kinv = np.linalg.inv(self.K)
        self.slope_cutoffs = ((0.6,1.4),(1.7,2.2)) # ((left lane),(right lane))
        rospy.spin()


    def colorCallback(self, color_image):
        try:
            self.cv_image_col = self.bridge.imgmsg_to_cv2(color_image, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)


    def depthCallback(self,depth_image):
        # depth is published in 16 bit 1mm resolution, i.e. a value of 1000 is 1 meter
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
        crp_ind_top_v = int((height*above))
        crp_ind_bottom_v = height - int((height*below))
        cropped = canny[int((height*above)):height - int((height*below)), int((width*side)):width - int((width*side))]

        sidetst = cv2.cvtColor(canny, cv2.COLOR_GRAY2BGR)
        tst = cv2.rectangle(sidetst,(0,crp_ind_top_v),(width,crp_ind_bottom_v),(255,0,0),3)
        #

        # Lane Detection
        canny = cv2.cvtColor(cropped, cv2.COLOR_GRAY2BGR)

        lines = cv2.HoughLines(cropped,1,np.pi/180,30)
        slope = []
        slope_int_l = []
        slope_int_r = []
        if lines is not None:
            print(len(lines))
            for i in range(0,len(lines)):
                rho = lines[i][0][0]
                theta = lines[i][0][1]
                a = np.cos(theta)
                b = np.sin(theta)
                x0 = a*rho
                y0 = b*rho

                y1 = crp_ind_top_v
                x1 = int(-np.tan(theta)*(0) + rho/np.cos(theta) )
                y2 = crp_ind_bottom_v
                x2 = int(-np.tan(theta)*(crp_ind_bottom_v-crp_ind_top_v) + rho/np.cos(theta) )

                #x1 = int(x0 + 500*(-b))
                #y1 = int(y0 + 500*(a))+crp_ind_top_v
                #x2 = int(x0 - 500*(-b))
                #y2 = int(y0 - 500*(a))+crp_ind_top_v
                slope.append(theta)

                if slope[i]<self.slope_cutoffs[1][1] and slope[i]>self.slope_cutoffs[1][0]: # right lane
                    cv2.line(tst,(x1,y1),(x2,y2),(255,0,255),2)
                    #print("right",theta,rho)
                    slope_int_r.append((-np.tan(theta),rho/np.cos(theta)))
                if slope[i]<self.slope_cutoffs[0][1] and slope[i]>self.slope_cutoffs[0][0]: # left lane
                    cv2.line(tst,(x1,y1),(x2,y2),(0,255,255),2)
                    #print("left",theta,rho)
                    slope_int_l.append((-np.tan(theta),rho/np.cos(theta)))

            if len(slope_int_l)!=0: # parse data if we catch the left lane
                mean_ln_l = np.mean(slope_int_l,0)
                y1_mean_l = crp_ind_top_v
                x1_mean_l = int(mean_ln_l[0]*(0) + mean_ln_l[1] )
                y2_mean_l = crp_ind_bottom_v
                x2_mean_l = int(mean_ln_l[0]*(crp_ind_bottom_v-crp_ind_top_v) + mean_ln_l[1] )
                cv2.line(tst,(x1_mean_l,y1_mean_l),(x2_mean_l,y2_mean_l),(0,255,0),2)

            if len(slope_int_r)!=0: # parse data if we catch the right lane
                mean_ln_r = np.mean(slope_int_r,0)
                y1_mean_r = crp_ind_top_v
                x1_mean_r = int(mean_ln_r[0]*(0) + mean_ln_r[1] )
                y2_mean_r = crp_ind_bottom_v
                x2_mean_r = int(mean_ln_r[0]*(crp_ind_bottom_v-crp_ind_top_v) + mean_ln_r[1] )
                cv2.line(tst,(x1_mean_r,y1_mean_r),(x2_mean_r,y2_mean_r),(0,255,0),2)

        cv2.line(tst,(int(width/2),crp_ind_top_v),(int(width/2),crp_ind_bottom_v),(0,0,255),2)
        #print("Depth at u=%d,v=%d is %f meters",int(width/2),crp_ind_top_v,self.cv_image_depth[crp_ind_top_v][int(width/2)]/1000.0)




        vval = crp_ind_top_v # they all have the same v pixel coordinate

        dpth = 1 # assuming a flat surface and level car, taking points at same v value means all points at same depth
        #dpth = self.cv_image_depth[vval][uval]/1000.0


        # calculate 3D coordinates of middle of camera frame (should be 0,Y,depth)
        midFrame = int(width/2)
        X_midFrame = (midFrame-self.K[0][2])*dpth/self.K[0][0]
        Y_midFrame = (midFrame-self.K[1][2])*dpth/self.K[1][1]
        # #Z = dpth
        cv2.circle(tst,(midFrame,vval),4,(0,255,255),1)

        # calculate bias from center
        if 'x1_mean_r' in locals(): # need to convert to meters
            # calculate 3D coordinate of lane lines and middle of camera frame at top of crop
            uval_r = int(x1_mean_r) # pixel coordinate of left lane
            X_r = (uval_r-self.K[0][2])*dpth/self.K[0][0]
            #Y_r = (vval-self.K[1][2])*dpth/self.K[1][1]
            deviation_r = self.track_lane_width/2 - (X_r - X_midFrame)
        else:
            deviation_r = 'nan'


        if 'x1_mean_l' in locals(): # need to convert to meters!
            # calculate 3D coordinate of lane lines and middle of camera frame at top of crop
            uval_l = int(x1_mean_l) # pixel coordinate of left lane
            X_l = (uval_l-self.K[0][2])*dpth/self.K[0][0]
            #Y_l = (vval-self.K[1][2])*dpth/self.K[1][1]
            deviation_l = -self.track_lane_width/2 - (X_l - X_midFrame)
        else:
            deviation_l = 'nan'

        print((deviation_l,deviation_r))

        if self.reset_tracking is True:
            self.reset_tracking = False

        # publish to pid
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(tst, "bgr8"))
            self.state_pub.publish(deviation_r)
            #self.setpoint_pub.publish(avg_curv)
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
