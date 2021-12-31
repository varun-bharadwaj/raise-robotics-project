#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include<bits/stdc++.h>
#include<cmath>
#include<math.h>
#include<algorithm>
#include <vector>

float standardDeviation(std::vector<float> v);
float findDistanceToPoint(pcl::PointXYZ ref1, pcl::PointXYZ ref2, pcl::PointXYZ point);
float distanceBetweenPoins (pcl::PointXYZ pt1, pcl::PointXYZ pt2);
float getRectangleIndex(pcl::PointCloud<pcl::PointXYZ>::Ptr CloudHull);

int
main ()
{
  //Create Necessary Cloud Objects
  pcl::PCLPointCloud2::Ptr cloud_blob (new pcl::PCLPointCloud2), cloud_filtered_blob (new pcl::PCLPointCloud2);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>), cloud_p (new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>), cloud_projected (new pcl::PointCloud<pcl::PointXYZ>), cloud_hull (new pcl::PointCloud<pcl::PointXYZ>),
  cloud_filtered_outliers (new pcl::PointCloud<pcl::PointXYZ>), table_cloud (new pcl::PointCloud<pcl::PointXYZ>), not_table_cloud (new pcl::PointCloud<pcl::PointXYZ>);

  // Fill in the cloud data
  pcl::PCDReader reader;
  reader.read ("data/data8.pcd", *cloud_blob);

  // Create the filtering object: downsample the dataset using a leaf size of 1cm
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (cloud_blob);
  sor.setLeafSize (0.01f, 0.01f, 0.01f);
  sor.filter (*cloud_filtered_blob);

  // Convert to the templated PointCloud
  pcl::fromPCLPointCloud2 (*cloud_filtered_blob, *cloud_filtered);

  //Create Segmentation Objects
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (1000);
  seg.setDistanceThreshold (0.01);

  // Create the filtering object
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  pcl::copyPointCloud(*cloud_filtered, *not_table_cloud);
  int counter = 0;
  float minRunningSum = HUGE_VALF;

  //Loop through 10 largest planar components and check which one is most similar to table
   while (counter < 10) //Sample 10 largest planar components
   {
    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (cloud_filtered);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
      std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
      break;
    }

    // Extract the inliers
    extract.setInputCloud (cloud_filtered);
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter (*cloud_p);

    //Remove outliers for more accurate Convex Hull
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor2;
    sor2.setInputCloud (cloud_p);
    sor2.setMeanK (500);
    sor2.setStddevMulThresh (0.01);
    sor2.filter (*cloud_filtered_outliers);

    pcl::ProjectInliers<pcl::PointXYZ> proj;
    proj.setModelType (pcl::SACMODEL_PLANE);
    proj.setInputCloud (cloud_filtered_outliers);
    proj.setModelCoefficients (coefficients);
    proj.filter (*cloud_projected); 

    //Generate Convex Hull
    pcl::ConvexHull<pcl::PointXYZ> chull;
    chull.setInputCloud (cloud_projected);
    chull.reconstruct (*cloud_hull);

    // Create the filtering object
    extract.setNegative (true);
    extract.filter (*cloud_f);
    cloud_filtered.swap (cloud_f);
    
    //Returns how likely current segment is the table
    float currIndex = getRectangleIndex(cloud_hull);

    if (currIndex < minRunningSum){
      minRunningSum = currIndex;
      pcl::copyPointCloud(*cloud_filtered_outliers, *table_cloud);
    }

    counter++;
  }

  //Visualization
  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red(table_cloud, 255, 0, 0);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> light_blue(not_table_cloud, 173, 216, 230);
  viewer->addPointCloud<pcl::PointXYZ> (table_cloud, red, "sample cloud");
  viewer->addPointCloud<pcl::PointXYZ> (not_table_cloud, light_blue, "nottable");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  while (!viewer->wasStopped ())
  {
    viewer->spinOnce (100);
    //viewer2->spinOnce(100);
  }
  return (0);
}



/* EXTRA FUNCTIONS USED TO DETERMINE IF CLOUD IS TABLE OR NOT */

//standard deviation
float standardDeviation(std::vector<float> v)
{
    float sum=0;
    for(float coord : v)
      sum+= coord;
    float ave = sum/static_cast<float>(v.size());
    float E=0;
    for(int i=0;i<v.size();i++)
            E+=(v[i] - ave)*(v[i] - ave);
    return sqrt(1/ static_cast<float>(v.size())*E);
}


float findDistanceToPoint(pcl::PointXYZ ref1, pcl::PointXYZ ref2, pcl::PointXYZ point){
  return std::abs((ref2.x - ref1.x) * (ref1.y - point.y) - (ref1.x - point.x) * (ref2.y - ref1.y)) / (sqrt((ref2.x - ref1.x) * (ref2.x - ref1.x) + (ref2.y - ref1.y)* (ref2.y - ref1.y)));
}


float distanceBetweenPoins (pcl::PointXYZ pt1, pcl::PointXYZ pt2){
  return sqrt((pt1.x - pt2.x) * (pt1.x - pt2.x) + (pt1.y - pt2.y) * (pt1.y - pt2.y));
}


/* Function that calculates likelihood of being the table*/

float getRectangleIndex(pcl::PointCloud<pcl::PointXYZ>::Ptr CloudHull){
    
    //Used to calculate how different cloud is from rectange
    float pi = 3.14159265;
    float runningSum = 0;
    float maxSum = 0;

    //Used to calculate orientation
    std::vector<float> yCoords;

    //Calculates how similar it is to a rectange 
    //(summing how close all points are to being either 90 or 180 removing 1 outlier in case of prev algorithm not working)
    for (int i = 1; i < CloudHull->width - 1; i++){
      yCoords.push_back(CloudHull->points[i].y);

      float currDistance = findDistanceToPoint(CloudHull->points[i - 1], CloudHull->points[i + 1], CloudHull->points[i]);
      float x = distanceBetweenPoins(CloudHull->points[i-1], CloudHull->points[i]);
      float z = distanceBetweenPoins(CloudHull->points[i+1], CloudHull->points[i]);

      float angle = (acos(currDistance/x) + acos(currDistance/z)) * 180.0 / pi;
      float currSum  =  std::min(abs(angle - 90), abs(angle - 180));
      if (currSum > maxSum){
        float temp = maxSum;
        maxSum = currSum;
        currSum = maxSum;
      }
      runningSum += currSum;
    }

  //Orientation is calculated by standard deviation with respect to y axis
  float yDev = standardDeviation(yCoords) * 10;

  return runningSum + yDev;
}

