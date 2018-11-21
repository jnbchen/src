#include <iostream>
#include <fstream>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

#include <ibeo_had/Point2D.h>
#include <ibeo_had/Color.h>
#include <ibeo_had/Object.h>
#include <ibeo_had/ObjectArray.h>

#include <dirent.h>
#include <json/json.h>

ros::Publisher pub_points;
ros::Publisher pub_objects;


//==================================================================================
void LoadPcd(std::string *pcdfile, sensor_msgs::PointCloud2& output)
{
  // initialize container
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  // load pcd file to pcl::pointcloud
  pcl::io::loadPCDFile<pcl::PointXYZ>(*pcdfile, *cloud);
  for(size_t i = 0; i < cloud->points.size(); ++i)
  {
    cloud->points[i].x = -cloud->points[i].x;
    cloud->points[i].y = -cloud->points[i].y;
  }
  // convert to ros msg
  pcl::toROSMsg(*cloud, output);
  // header
  output.header.frame_id = "/visualization";
  output.header.stamp = ros::Time::now();
  // return output;
}

//==================================================================================
void LoadJson(std::string *jsonfile, ibeo_had::ObjectArray& object_array)
{
  //object_array.objects.clear();
  ibeo_had::Point2D contour_points;
  // open json file
  std::ifstream is;
  is.open(*jsonfile, std::ios::binary);
  if(!is.is_open())
    std::cerr << "Error opening file!" << std::endl;
  std::string line, jsonStr;
  while (std::getline(is, line))
    jsonStr.append(line);

  Json::CharReaderBuilder b;
  Json::CharReader* reader(b.newCharReader()); 
  Json::Value root;
  JSONCPP_STRING error;
  if(reader->parse(jsonStr.c_str(), jsonStr.c_str()+jsonStr.size(), &root, &error))
  {
    std::cout << "begin parse json..." << std::endl;
    // loop of objects in json file
    for(Json::Value::const_iterator outer = root.begin(); outer!= root.end(); outer++)
    {
      Json::Value subroot = *outer;
      ibeo_had::Object object;
      object.id = subroot["id"].asInt();
      object.age = subroot["age"].asInt();
      object.classification = subroot["classification"].asInt();
      object.classification_certainty = subroot["class certainty"].asInt();
      object.classification_age = subroot["class age"].asInt();

      object.object_box_center.x = subroot["object box center"]["x"].asFloat();
      object.object_box_center.y = subroot["object box center"]["y"].asFloat();

      object.object_box_size.x = subroot["object box size"]["x"].asFloat();
      object.object_box_size.y = subroot["object box size"]["y"].asFloat();

      object.yaw_angle = subroot["object box orientation"].asFloat();

      object.absolute_velocity.x = subroot["absoluted velocity"]["x"].asFloat();
      object.absolute_velocity.y = subroot["absoluted velocity"]["y"].asFloat();

      object.relative_velocity.x = subroot["related velocity"]["x"].asFloat();
      object.relative_velocity.y = subroot["related velocity"]["y"].asFloat();

      int contour_points_number = subroot["number of contour point"].asInt();
      for(int i = 0; i < contour_points_number; i++)
      {
        contour_points.x = subroot["contour point " + std::to_string(i)]["x"].asFloat();
        contour_points.y = subroot["contour point " + std::to_string(i)]["y"].asFloat();
        object.contour.push_back(contour_points);
      }
      switch(object.classification)
      {
        case object.UNCLASSIFIED:
        object.text = "UNCLASSIFIED";
        object.color.r = 1.0;
        object.color.g = 1.0;
        object.color.b = 1.0;
        object.color.a = 1.0;
        break;
        case object.UNKNOWN_SMALL:
        object.text = "UNKNOWN_SMALL";
        object.color.r = 1.0;
        object.color.g = 1.0;
        object.color.b = 1.0;
        object.color.a = 1.0;
        break;
        case object.UNKNOWN_BIG:
        object.text = "UNKNOWN_BIG";
        object.color.r = 1.0;
        object.color.g = 1.0;
        object.color.b = 1.0;
        object.color.a = 1.0;
        break;
        case object.PEDESTRIAN:
        object.text = "PEDESTRIAN";
        object.color.r = 1.0;
        object.color.g = 0.0;
        object.color.b = 0.0;
        object.color.a = 1.0;
        break;
        case object.BIKE:
        object.text = "BIKE";
        object.color.r = 1.0;
        object.color.g = 1.0;
        object.color.b = 0.0;
        object.color.a = 1.0;
        break;
        case object.CAR:
        object.text = "CAR";
        object.color.r = 0.0;
        object.color.g = 0.0;
        object.color.b = 1.0;
        object.color.a = 1.0;
        break;
        case object.TRUCK:
        object.text = "TRUCK";
        object.color.r = 0.0;
        object.color.g = 1.0;
        object.color.b = 0.0;
        object.color.a = 1.0;
        break;
      }
      object_array.objects.push_back(object);
      //object.clear();
    }
    is.close();
    delete reader;
    object_array.header.frame_id = "/ObjectList";
    object_array.header.stamp = ros::Time::now();
  }
  else
    std::cerr << "parse error!" << std::endl; 
}

//==================================================================================
int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "ibeo_parser");
  ros::NodeHandle nh("~");

  // Initialize offline data path
  std::string path_pcd;
  std::string path_json;
  int pub_rate;
  nh.param<std::string>("pcd_path", path_pcd, "/home/jinbo/UISEE/velo_grid/");
  nh.param<std::string>("json_path", path_json, "/home/jinbo/UISEE/ibeo_json/");
  nh.param<int>("pub_rate", pub_rate, 1);
  const char *pcd = path_pcd.c_str();
  const char *json = path_json.c_str();

  pub_points = nh.advertise<sensor_msgs::PointCloud2> ("scan_points", 100);
  pub_objects = nh.advertise<ibeo_had::ObjectArray> ("object_list", 100);
  ros::Rate loop_rate(pub_rate);

  // Prepare directory for dirent (just copy)
  DIR *dirp_pcd;
  DIR *dirp_json;
  struct dirent *directory_pcd, *directory_json;
  dirp_pcd = opendir(pcd);
  dirp_json = opendir(json);
  std::cout << "cd the path: " << path_pcd << " and " << path_json << std::endl;

  // Load pcd to sensor_msgs
  if(dirp_pcd && dirp_json)
  {
      while((directory_pcd = readdir(dirp_pcd))!= NULL &&
            (directory_json = readdir(dirp_json))!= NULL && ros::ok())
      {
        std::string file_pcd(directory_pcd->d_name);
        std::string file_json(directory_json->d_name);
        std::size_t found_pcd = file_pcd.find(".pcd");
        std::size_t found_json = file_json.find(".json");
        std::cout << "opening .pcd file: " << file_pcd << std::endl;
        std::cout << "opening .json file: " << file_json << std::endl;

        if((found_pcd != std::string::npos) && (found_json != std::string::npos))
        {
          sensor_msgs::PointCloud2 scan_points;
          ibeo_had::ObjectArray object_array;

          std::string pcdfile = path_pcd + file_pcd;
          std::string jsonfile = path_json + file_json;
          
          LoadPcd(&pcdfile, scan_points);
          std::cout << "load pcd" << std::endl;
          LoadJson(&jsonfile, object_array);
          std::cout << "load json" << std::endl;

          pub_points.publish(scan_points);
          std::cout << "i have published a sensor_msg." << std::endl;
          pub_objects.publish(object_array);
          std::cout << "i have published a obejct_array msg." << std::endl;

          ros::spinOnce();
          loop_rate.sleep();
          //delete[] object_list;
        }
        else
          std::cerr << "no .pcd file has been found." << std::endl;
      }
      closedir(dirp_pcd);
      closedir(dirp_json);
  }
  else
  {
    std::cerr << "no directory has been found." << std::endl;
    return 1;
  }

  // Create a ROS subscriber for the input point cloud
  //sub = nh.subscribe ("input", 1, cloud_cb);
 // ros::spin();

  return 0;
}
