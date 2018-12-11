#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>

#include <ibeo_had/ObjectArray.h>
#include <math.h>

//================================================================================
class Visualizer
{
  ros::NodeHandle node_;
  ros::Publisher pub_class_;
  ros::Publisher pub_box_;
  ibeo_had::Object object_;
  visualization_msgs::Marker marker_;
  visualization_msgs::Marker line_strip_;
  visualization_msgs::MarkerArray ibeo_class_;
  //visualization_msgs::MarkerArray ibeo_box_;

  public:
  Visualizer(ros::NodeHandle nh, float text_size, float box_size): node_(nh)
  {
    pub_class_ = nh.advertise <visualization_msgs::MarkerArray> ("ibeo_class", 10);

    marker_.header.frame_id = "ibeo";
    marker_.ns = "class";
    marker_.action = visualization_msgs::Marker::ADD;
    marker_.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker_.scale.z = text_size;
    marker_.pose.orientation.w = 1.0;
    
    line_strip_.header.frame_id = "ibeo";
    line_strip_.ns = "box";
    line_strip_.action = visualization_msgs::Marker::ADD;
    line_strip_.type = visualization_msgs::Marker::LINE_STRIP;
    line_strip_.scale.x = box_size;
    line_strip_.scale.y = box_size;

    marker_.lifetime = line_strip_.lifetime = ros::Duration(0.2);
    //std::cout << "visualizer is built." << std::endl;
  }
  
  ~Visualizer(){}

  void ObjectClass(std::vector<ibeo_had::Object>::const_iterator msg)
  {
    marker_.header.stamp = ros::Time::now();
    marker_.id = msg->id;
    marker_.pose.position.x = msg->object_pose.x;
    marker_.pose.position.y = msg->object_pose.y;
    //int certainty = msg->classification_certainty;
    //int class_age = msg->classification_age;
    //std::string txt = msg->text;
    marker_.text = msg->text; //txt+" cr:"+std::to_string(certainty)+" age:"+std::to_string(class_age);
    marker_.color = msg->color;
  }
  
  void ObjectBox(std::vector<ibeo_had::Object>::const_iterator msg)//ibeo_had::Object* msg)
  {
    line_strip_.header.stamp = ros::Time::now();
    line_strip_.id = msg->id+1000;

    geometry_msgs::Point p1, p2, p3, p4;
    float w = 0.5*(msg->object_box_x);
    float l = 0.5*(msg->object_box_y);
    p1.x = -w; p1.y = -l;
    p2.x = -w; p2.y = l;
    p3.x = w; p3.y = l;
    p4.x = w; p4.y = -l;
    line_strip_.points.clear();
    line_strip_.points.push_back(p1);
    line_strip_.points.push_back(p2);
    line_strip_.points.push_back(p3);
    line_strip_.points.push_back(p4);
    line_strip_.points.push_back(p1);
    line_strip_.pose.position.x = msg->object_pose.x;
    line_strip_.pose.position.y = msg->object_pose.y;

    line_strip_.color = msg->color;

    line_strip_.pose.orientation.z = sin(0.5*(msg->object_pose.theta));
    line_strip_.pose.orientation.w = cos(0.5*(msg->object_pose.theta));
  }

  void Callback(const ibeo_had::ObjectArray::ConstPtr& msg)
  {
    ibeo_class_.markers.clear();
    for(auto iter = msg->objects.begin(); iter!=msg->objects.end(); iter++)
    {
      ObjectClass(iter);
      ObjectBox(iter);
      ibeo_class_.markers.push_back(marker_);
      ibeo_class_.markers.push_back(line_strip_);
    }
    //ibeo_class_.header.stamp = ros::Time::now();
    //ibeo_class_.header.frame_id = "ibeo";
    pub_class_.publish(ibeo_class_);
    //std::cout << "i have subscribed a object text msg." << std::endl;
      
    //pub_box_.publish(line_strip_);
    //std::cout << "i have published a object box msg." << std::endl;
  }
};//Visualizer
//================================================================================

int
main (int argc, char** argv)
{
  ros::init (argc, argv, "ibeo_visualizer");
  ros::NodeHandle nh("~");

  std::string sub_topic;
  float text_size, box_size;
  int rate;
  nh.param<std::string>("sub_topic", sub_topic, "/idc_reader/ibeo_objects");
  nh.param<float>("text_size", text_size, 1);
  nh.param<float>("box_size", box_size, 0.1);
  nh.param<int>("rate", rate, 30); 

  Visualizer visualizer(nh, text_size, box_size);
  ros::Subscriber sub = nh.subscribe(sub_topic, 10, &Visualizer::Callback, &visualizer);
  ros::Rate loop_rate(rate);
  
  //ros::spin();
  loop_rate.sleep();
  ros::spin();

  return 0;
}
