#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <ibeosdk/ecu.hpp>
#include "ibeosdk/listener/DataListener.hpp"

#include <ibeo_had/ObjectArray.h>
#include "PointXYZIRL.h"

#include <boost/shared_ptr.hpp>
#include <pcl/io/pcd_io.h>

//======================================================================
using namespace ibeosdk;

class AllListener : public ibeosdk::DataListener<ibeosdk::ScanEcu>,
                       public ibeosdk::DataListener<ObjectListEcuEt>
{
  public:
    virtual ~AllListener() {}

  private:
    ros::NodeHandle node_;
    ros::Publisher pub_points_;
    ros::Publisher pub_objects_;
    ibeo_had::Object object_;
    ibeo_had::ObjectArray object_array_;
    boost::shared_ptr<pcl::PointCloud<PointXYZIRL>> scan_xyzirl_;
    int count_;

  public:
    //========================================
    AllListener(ros::NodeHandle nh):
        node_(nh),
        scan_xyzirl_ (new pcl::PointCloud<PointXYZIRL>())
    {
        count_ = 0;
        pub_points_ = nh.advertise<sensor_msgs::PointCloud2>("ibeo_points", 1);
        pub_objects_ = nh.advertise<ibeo_had::ObjectArray>("ibeo_objects", 1);
    }

    //========================================
    virtual void onData(const ScanEcu* const);
	virtual void onData(const ObjectListEcuEt* const);
    void SetColor(int);

}; //AllListener
//======================================================================