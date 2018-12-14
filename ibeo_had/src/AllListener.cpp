#include <vector>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

#include <AllListener.h>

//======================================================================
using namespace ibeosdk;

//======================================================================
TimeConversion tc;
boost::posix_time::ptime utc_epoch(boost::gregorian::date(1970,1,1));
typedef unsigned short uint16;

//======================================================================
void AllListener::onData(const ScanEcu* const scan)
{
    std::vector<ScanPointEcu> points;
    points = scan->getScanPoints();
    count_++;

    for (size_t index = 0; index < points.size(); index++)
    {
        unsigned int flag = points[index].getFlags();
        unsigned int time_stamp = points[index].getTimeOffset();
        if((flag&0x0007)==0)
        {
            PointXYZIRL xyzirl;
            xyzirl.x = points[index].getPositionX();
            xyzirl.y = points[index].getPositionY();
            xyzirl.z = points[index].getPositionZ();
            xyzirl.intensity = points[index].getEchoPulseWidth();
            xyzirl.ring = points[index].getLayer();
            xyzirl.label = flag;
            //logInfo << flag << std::endl;
            scan_xyzirl_->push_back(xyzirl);
        }
    }

    if (count_&1 == 1)
    {
        sensor_msgs::PointCloud2 scan_points;
        pcl::toROSMsg(*scan_xyzirl_, scan_points);
        scan_points.header.frame_id = "ibeo";

        //double time_stamp = (scan->getStartTimestamp().toPtime()-utc_epoch).total_microseconds()/(double)(1000*1000);
        scan_points.header.stamp = ros::Time::now();
        pub_points_.publish(scan_points);

        scan_xyzirl_.reset (new pcl::PointCloud<PointXYZIRL>);
    }
}
//========================================

void AllListener::onData(const ObjectListEcuEt* const objectList)
{
    // logInfo << "ObjectListEcuEt " << std::endl;
    object_array_.objects.clear();
    for (auto iter=objectList->getObjects().begin(); iter!=objectList->getObjects().end(); iter++)
    {
        unsigned int flag = iter->getFlags();
        if(flag>>8 == 1)
        {
        object_.id = iter->getObjectId();
        object_.age = iter->getObjectAge();
        object_.classification = iter->getClassification();
        SetColor(object_.classification);

        object_.classification_certainty = iter->getClassCertainty();
        object_.classification_age = iter->getClassAge();
        object_.flag = flag;

        object_.object_pose.x = iter->getObjBoxCenter().getX();
        object_.object_pose.y = iter->getObjBoxCenter().getY();
        object_.object_pose.theta = iter->getObjBoxOrientation();

        object_.object_box_x = iter->getObjBoxSize().getX();
        object_.object_box_y = iter->getObjBoxSize().getY();

        object_.absolute_velocity_x = iter->getAbsVelocity().getX();
        object_.absolute_velocity_y = iter->getAbsVelocity().getY();

        object_.relative_velocity_x = iter->getRelVelocity().getX();
        object_.relative_velocity_y = iter->getRelVelocity().getY();

        /* if (iter->getNbOfContourPoints() > 0)
        {
            for (auto it=iter->getContourPoints().begin(); it!=iter->getContourPoints().end(); it++)
            {
                contour_points_.x = it->getX();
                contour_points_.y = it->getY();
                object_.contour.push_back(contour_points_);
            }
        } */
        //loop_rate_.sleep();
        //ros::spin();

        object_array_.objects.push_back(object_);
        }
    }
    object_array_.header.frame_id = "ibeo";
    //double time_stamp = (objectList->getTimestamp().toPtime()-utc_epoch).total_microseconds()/(double)(1000*1000);
    object_array_.header.stamp = ros::Time::now();
    pub_objects_.publish(object_array_);
    //std::cout << "i have published a object array msg." << std::endl;
}
//==========================================================

void AllListener::onData(const VehicleStateBasicEcu2808* const vsb)
{
    // Vehicle State here, you can do it better!
}

void AllListener::SetColor(int classification)
{
    switch(classification)
    {
        case object_.UNCLASSIFIED:
        object_.text = "UNCLASSIFIED";
        object_.color.r = 1.0;
        object_.color.g = 1.0;
        object_.color.b = 1.0;
        object_.color.a = 1.0;
        break;
        case object_.UNKNOWN_SMALL:
        object_.text = "UNKNOWN_SMALL";
        object_.color.r = 1.0;
        object_.color.g = 1.0;
        object_.color.b = 1.0;
        object_.color.a = 1.0;
        break;
        case object_.UNKNOWN_BIG:
        object_.text = "UNKNOWN_BIG";
        object_.color.r = 1.0;
        object_.color.g = 1.0;
        object_.color.b = 1.0;
        object_.color.a = 1.0;
        break;
        case object_.PEDESTRIAN:
        object_.text = "PEDESTRIAN";
        object_.color.r = 1.0;
        object_.color.g = 0.0;
        object_.color.b = 0.0;
        object_.color.a = 1.0;
        break;
        case object_.BIKE:
        object_.text = "BIKE";
        object_.color.r = 1.0;
        object_.color.g = 1.0;
        object_.color.b = 0.0;
        object_.color.a = 1.0;
        break;
        case object_.CAR:
        object_.text = "CAR";
        object_.color.r = 0.0;
        object_.color.g = 0.0;
        object_.color.b = 1.0;
        object_.color.a = 1.0;
        break;
        case object_.TRUCK:
        object_.text = "TRUCK";
        object_.color.r = 0.0;
        object_.color.g = 1.0;
        object_.color.b = 0.0;
        object_.color.a = 1.0;
        break;
    }
}
//======================================================================
