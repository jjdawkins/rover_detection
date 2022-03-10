#!/usr/bin/env python3

from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64
from geometry_msgs.msg import PolygonStamped, Point32
import rospy
import numpy as np
import math
import pcl


NODE_NAME = "obstacle_detection_LIDAR_node"
SUB_TOPIC = "scan"
PUB_TOPIC = "obstacles"
QUEUE_SIZE = 1


class ObstacleDetectionNode:
    def __init__(self, node_name, sub_topic_lidar, pub_topic):

        # Publishers
        self.obs_pub = rospy.Publisher(pub_topic, PolygonStamped, queue_size=QUEUE_SIZE)
        self.scan_sub = rospy.Subscriber('scan',LaserScan,self.scanCallback)
        self.range_threshold = rospy.get_param('detection_distance_threshold',3) # default to 2 m
        self.fov_threshold = rospy.get_param('FOV',math.pi/2)
        self.fov_publisher = rospy.Publisher('fovscan',LaserScan,queue_size=QUEUE_SIZE)

    def scanCallback(self,msg):
        #print("scan received")

        # raw laserscan data
        ranges = np.asarray(msg.ranges)
        # array of angles corresponding to ranges
        angles = np.arange(msg.angle_min,msg.angle_max,msg.angle_increment)

        # narrow angles and ranges to field of view
        fov_angles = angles[(angles>-self.fov_threshold) & (angles<self.fov_threshold)]
        fov_ranges = ranges[(angles>-self.fov_threshold) & (angles<self.fov_threshold)]
        # eliminate samples that are out of range
        fov_ranges[fov_ranges>self.range_threshold] = 100

        ranges_imp = fov_ranges[fov_ranges<self.range_threshold]
        angles_imp = fov_angles[fov_ranges<self.range_threshold]

        # # # # # Perform obstacle detection here!!!

        xvals_b = ranges_imp*np.cos(angles_imp) # x position of detected point
        yvals_b = ranges_imp*np.sin(angles_imp) # y position of detected point
        pts = np.zeros([len(xvals_b),3],dtype=np.float32)
        for i in range(len(xvals_b)):
                pts[i,:] = [xvals_b[i],yvals_b[i],0]

        p = pcl.PointCloud() # add points into this
        p.from_array(pts)
        # Creating the KdTree object for the search method of the extraction
        tree = p.make_kdtree()

        ec = p.make_EuclideanClusterExtraction()
        ec.set_ClusterTolerance (0.4)
        ec.set_MinClusterSize (20)
        ec.set_MaxClusterSize (1000)
        ec.set_SearchMethod (tree)
        cluster_indices = ec.Extract()

        centroid = np.zeros([len(cluster_indices),3])
        for j, indices in enumerate(cluster_indices):
            # cloudsize = indices
            #print('indices = ' + str(len(indices)))
            # cloudsize = len(indices)
            points = np.zeros((len(indices), 3), dtype=np.float32)
            # points = np.zeros((cloudsize, 3), dtype=np.float32)

            # for indice in range(len(indices
            sumx = 0
            sumy = 0
            sumz = 0
            for i, indice in enumerate(indices):
                # print('dataNum = ' + str(i) + ', data point[x y z]: ' + str(cloud_filtered[indice][0]) + ' ' + str(cloud_filtered[indice][1]) + ' ' + str(cloud_filtered[indice][2]))
                # print('PointCloud representing the Cluster: ' + str(cloud_cluster.size) + " data points.")
                #print(indices)
                #print(indice)
                points[i][0] = p[indice][0]
                points[i][1] = p[indice][1]
                points[i][2] = p[indice][2]
                sumx += points[i][0]/len(indices)
                sumy += points[i][1]/len(indices)
                sumz += points[i][2]/len(indices)
            centroid[j,:] = np.array([sumx,sumy,sumz])

        #print(centroid)

        # package subsampled laserscan data into laserscan message
        fovmsg = LaserScan()
        fovmsg.header.frame_id = msg.header.frame_id
        fovmsg.header.stamp = rospy.Time.now()
        fovmsg.angle_min = -self.fov_threshold
        fovmsg.angle_max = self.fov_threshold
        fovmsg.range_min = msg.range_min
        fovmsg.range_max = self.range_threshold
        fovmsg.angle_increment = msg.angle_increment
        fovmsg.ranges = tuple(fov_ranges)
        self.fov_publisher.publish(fovmsg)

        # package obstacles and publish as Polygon

        obsmsg = PolygonStamped()
        obsmsg.header.stamp = rospy.Time.now()
        obsmsg.header.frame_id = msg.header.frame_id
        for ii in range(len(centroid)):
            obsmsg.polygon.points.append(Point32(centroid[ii,0],centroid[ii,1],0))
        self.obs_pub.publish(obsmsg)



def main():
    # init node
    rospy.init_node(NODE_NAME, anonymous=True)
    try:
        ObstacleDetectionNode(NODE_NAME, SUB_TOPIC, PUB_TOPIC)
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down node %s", NODE_NAME)
    rospy.spin()

if __name__ == '__main__':
    main()
