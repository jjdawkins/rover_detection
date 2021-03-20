
#include <ros/ros.h>
// PCL specific includes
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>

#include <pcl/common/impl/angles.hpp>
#include <pcl/common/centroid.h>

#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/sac_model_perpendicular_plane.h>

#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/kdtree/io.h>


#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>


ros::Publisher pub;
ros::Publisher obs_pub;


/*void
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{

  // Container for original & filtered data
    pcl::PCLPointCloud2* cloud_2 = new pcl::PCLPointCloud2; 
    //pcl::PCLPointCloud2ConstPtr cloudPtr(cloud_2);
    pcl::PCLPointCloud2 cloud_filtered;

      // Convert to PCL data type
    pcl_conversions::toPCL(*cloud_msg, *cloud_2);

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);

    pcl::PointCloud<pcl::PointXYZ> cloud_1;
    pcl::PCLPointCloudConstPtr cloudPtr(cloud_1);
    
    pcl::fromPCLPointCloud2(*cloud_2, cloud_1);
    
    //pcl::toPCLPointCloud2(point_cloud, point_cloud2);
    
    // Perform the actual filtering
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    
   // sor.setInputCloud(cloudPtr);
   // sor.setLeafSize (0.1, 0.1, 0.1);
   // sor.filter(cloudPtr);    
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr (new pcl::PointCloud<pcl::PointXYZ> ());
    // Creating the KdTree object for the search method of the extraction
    tree->setInputCloud (cloud_ptr);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance (0.02); // 2cm
    ec.setMinClusterSize (100);
    ec.setMaxClusterSize (25000);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloudPtr);
    ec.extract (cluster_indices);
    
    
    
    // Convert to ROS data type
    sensor_msgs::PointCloud2 output;
    pcl_conversions::fromPCL(cloud_filtered, output);
    
    // Publish the data.
    pub.publish (output);
}*/

void cloud_cb(const sensor_msgs::PointCloud2& input_cloud){
  sensor_msgs::PointCloud2 output;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
  // elaboration on cloud_filtered ...

  pcl::fromROSMsg(input_cloud,*cloud_filtered);

  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
  tree->setInputCloud (cloud_filtered);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
  ec.setClusterTolerance (0.02); // 2cm
  ec.setMinClusterSize (100);
  ec.setMaxClusterSize (25000);
  ec.setSearchMethod (tree);

  ec.setInputCloud (cloud_filtered);
  ec.extract (cluster_indices);

  int j = 0;
  std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> sourceClouds;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it){

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
    //std::cout << it << std::endl;
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit){
      cloud_cluster->points.push_back (cloud_filtered->points[*pit]);
    }

    cloud_cluster->width = cloud_cluster->points.size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;
    sourceClouds.push_back(cloud_cluster);
        j++;
  }

    pcl::PointXYZRGB centroid;
    std::vector<pcl::PointXYZRGB> centroids;
        for (size_t i = 0; i < sourceClouds.size(); i++){

      pcl::computeCentroid( *(sourceClouds[i]), centroid);
      centroids.push_back(centroid);
      std::cout<<"center of pipe #"<<i+1<<" in ("<<(centroids[i]).x<<", "<<(centroids[i]).y<<", "<<(centroids[i]).z<<")"<<std::endl;
      geometry_msgs::PointStamped point_msg;
      point_msg.header.frame_id = "camera_depth_frame";
      point_msg.point.x = (centroids[i]).x;
      point_msg.point.y = (centroids[i]).y;
      point_msg.point.z = (centroids[i]).z;
      obs_pub.publish(point_msg);
      
    }

//pcl::toROSMsg ( *cloud_plane, output);
//output.header.frame_id = input.header.frame_id;
//pcl_pub.publish( output);
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("/extract_plane_indices/output", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("processed/output", 1);
  obs_pub = nh.advertise<geometry_msgs::PointStamped> ("obstacles",1);

  // Spin
  ros::spin ();
}
