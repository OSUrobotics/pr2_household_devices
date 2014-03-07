#include <iostream>

// ROS
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

// PCL
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

// STL
#include <vector>

// My package
#include <find_devices/GetKnobParams.h>

using namespace std;

// prototype function
template <typename T> void printVector (const vector<T> &vec);

bool get_knob_params (find_devices::GetKnobParams::Request &request, find_devices::GetKnobParams::Response &response)
{
  // Unpack point cloud message into PCL data type
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  fromROSMsg(request.cloud, *cloud);

  // Display size of point cloud
  cout << "KNOB_PARAMS> Number of points in cloud is: " << (*cloud).size() << endl;

  // Iterator for PCL point cloud data structure
  pcl::PointCloud<pcl::PointXYZ>::iterator it;

  // User-instantiated containers for point components
  vector <float> x, y, z;

  // Parse point cloud into x-, y-, and z-components; store in vectors
  for ( it = (*cloud).begin(); it != (*cloud).end(); ++it )
    {
      x.push_back(it -> x);  // copy x-component of point to x-vector
      y.push_back(it -> y);  // " " for y
      z.push_back(it -> z);  // " " for z
    }

  // Max/min values along each axis
  float extents_x[2] = {*min_element(x.begin(), x.end()), *max_element(x.begin(), x.end())};
  float extents_y[2] = {*min_element(y.begin(), y.end()), *max_element(y.begin(), y.end())};

  // Radius is average deviation from center
  float diameter = ((extents_x[1] - extents_x[0])/2 + (extents_y[1] - extents_y[0])/2); 
  //float diameter = (extents_x[1] - extents_x[0]); 
  cout << "Diameter is: " << diameter << endl;

  // Center is average of extents
  float center[2] = {(extents_x[0] + extents_x[1])/2, (extents_y[0] + extents_y[1])/2};
  //float center[2] = {(extents_x[0] + extents_x[1])/2, extents_y[1] - diameter/2};
  cout << "Center (x, y) is: (" << center[0] << ", " << center[1] << ")" << endl;

  // Knob front/top is largest Z-coordinate
  float top = *max_element(z.begin(), z.end());
  cout << "Maximum z-value is: " << top << endl;

  response.center.push_back( center[0] );
  response.center.push_back( center[1] );
  response.diameter = diameter;
  response.top = top;

  return true;
}

// function template for outputting vector elements (from textbook)
template <typename T> void printVector (const vector<T> &vec)
{
  typename vector<T>::const_iterator constIt; // const_iterator
  
  // Display vector elements using const_iterator
  for (constIt = vec.begin(); constIt != vec.end(); ++constIt)
    {
    cout << *constIt << endl;
    }
}


int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "get_knob_params");
  ros::NodeHandle nh;

  ros::ServiceServer service = nh.advertiseService("get_knob_params", get_knob_params);

  ros::spin();
}
