#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <ibeosdk/ecu.hpp>

#include <ibeosdk/devices/IdcFile.hpp>
//#include <ibeosdk/datablocks/PointCloudPlane7510.hpp>
//#include <ibeosdk/datablocks/commands/CommandEcuAppBaseStatus.hpp>
//#include <ibeosdk/datablocks/commands/ReplyEcuAppBaseStatus.hpp>
//#include <ibeosdk/datablocks/commands/CommandEcuAppBaseCtrl.hpp>
//#include <ibeosdk/datablocks/commands/EmptyCommandReply.hpp>

#include <ibeo_had/ObjectArray.h>

#include <string>
#include <vector>
#include <boost/shared_ptr.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

//======================================================================
using namespace ibeosdk;

//======================================================================
const ibeosdk::Version::MajorVersion majorVersion(5);
const ibeosdk::Version::MinorVersion minorVersion(2);
const ibeosdk::Version::Revision revision(2);
const ibeosdk::Version::PatchLevel patchLevel;
const ibeosdk::Version::Build build;
const std::string info("IbeoHADNode");

ibeosdk::Version appVersion(majorVersion, minorVersion, revision, patchLevel, build, info);
IbeoSDK ibeoSDK;

//======================================================================

void file_demo(const std::string& filename);

//======================================================================
TimeConversion tc;
boost::posix_time::ptime utc_epoch(boost::gregorian::date(1970,1,1));
typedef unsigned short uint16;

//======================================================================
class AllListener : public ibeosdk::DataListener<ScanEcu>,
                    public ibeosdk::DataListener<ObjectListEcuEt>
{
public:
	virtual ~AllListener() {}

private:
    ros::NodeHandle node_;
    ros::Publisher pub_points_;
    ros::Publisher pub_objects_;
    ros::Rate loop_rate_;
    ibeo_had::Object object_;
    ibeo_had::ObjectArray object_array_;
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZI>> scan_xyzi_;

public:
    //========================================
    AllListener(ros::NodeHandle nh, int rate):
        node_(nh),
        loop_rate_(rate),
        scan_xyzi_ (new pcl::PointCloud<pcl::PointXYZI>())
    {
        pub_points_ = nh.advertise<sensor_msgs::PointCloud2>("ibeo_points", 10);
        pub_objects_ = nh.advertise<ibeo_had::ObjectArray>("ibeo_objects", 10);
    }
	//========================================

    void onData(const ScanEcu* const scan)
    {
        std::vector<ScanPointEcu> points;
        points = scan->getScanPoints();

        for (size_t index = 0; index < points.size(); index++)
        {
            unsigned int flag = points[index].getFlags();
	    	unsigned int time_stamp = points[index].getTimeOffset();
            if((flag&0x0007)==0)
            {
                pcl::PointXYZI xyzi;
                xyzi.x = points[index].getPositionX();
                xyzi.y = points[index].getPositionY();
                xyzi.z = points[index].getPositionZ();
                xyzi.intensity = points[index].getEchoPulseWidth();
                scan_xyzi_->push_back(xyzi);
            }
        }

        sensor_msgs::PointCloud2 scan_points;
        pcl::toROSMsg(*scan_xyzi_, scan_points);
        scan_points.header.frame_id = "ibeo";

		double time_stamp = (scan->getStartTimestamp().toPtime()-utc_epoch).total_microseconds()/(double)(1000*1000);
        scan_points.header.stamp = ros::Time(time_stamp);
        pub_points_.publish(scan_points);
		loop_rate_.sleep();

        scan_xyzi_.reset (new pcl::PointCloud<pcl::PointXYZI>);
    }

    void SetColor(int classification)
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

	//========================================
	void onData(const ObjectListEcuEt* const objectList)
    {
        object_array_.objects.clear();
        for(auto iter=objectList->getObjects().begin(); iter!=objectList->getObjects().end(); iter++)
        {
		  unsigned int flag = iter->getFlags();
		  if(flag>>7&1 == 1)
		  {
            object_.id = iter->getObjectId();
            object_.age = iter->getObjectAge();
            object_.classification = iter->getClassification();
            SetColor(object_.classification);

            object_.classification_certainty = iter->getClassCertainty();
            object_.classification_age = iter->getClassAge();

            object_.object_box_center.x = iter->getObjBoxCenter().getX();
            object_.object_box_center.y = iter->getObjBoxCenter().getY();

            object_.object_box_size.x = iter->getObjBoxSize().getX();
            object_.object_box_size.y = iter->getObjBoxSize().getY();

            object_.yaw_angle = iter->getObjBoxOrientation();

            object_.absolute_velocity.x = iter->getAbsVelocity().getX();
            object_.absolute_velocity.y = iter->getAbsVelocity().getY();

            //object_.relative_velocity.x = iter->getRelVelocity().getX();
            //object_.relative_velocity.y = iter->getRelVelocity().getY();

            /* if (iter->getNbOfContourPoints() > 0)
            {
                for (auto it=iter->getContourPoints().begin(); it!=iter->getContourPoints().end(); it++)
                {
                    contour_points_.x = it->getX();
                    contour_points_.y = it->getY();
                    object_.contour.push_back(contour_points_);
                }
            } */
            object_array_.objects.push_back(object_);
		  }
        }
        object_array_.header.frame_id = "ibeo";
		//double time_stamp = (iter->getTimestamp().toPtime()-utc_epoch).total_microseconds()/(double)(1000*1000);
        object_array_.header.stamp = ros::Time::now();//(time_stamp);
		pub_objects_.publish(object_array_);
		//std::cout << "i have published a object array msg." << std::endl;
		loop_rate_.sleep();
    }

}; //AllEcuListener

//======================================================================
class CustomLogStreamCallbackExample : public CustomLogStreamCallback {
public:
	virtual ~CustomLogStreamCallbackExample() {}
public:
	virtual void onLineEnd(const char* const s, const int)
	{
		std::cerr << s << std::endl;
	}
}; // CustomLogStreamCallback
//======================================================================

int checkArguments(int argc, char** argv, bool& hasLogFile)
{
    const int minNbOfNeededArguments = 1;
    const int maxNbOfNeededArguments = 3;

    bool wrongNbOfArguments = false;
    if (argc < minNbOfNeededArguments) {
        std::cerr << "Missing argument" << std::endl;
        wrongNbOfArguments = true;
    }
    else if (argc > maxNbOfNeededArguments) {
        std::cerr << "Too many argument" << std::endl;
        wrongNbOfArguments = true;
    }

    if (wrongNbOfArguments) {
        std::cerr << argv[0] << " " << " IP [LOGFILE]" << std::endl;
        std::cerr << "\tIP is the ip address of the Ibeo Ecu, e.g. 192.168.0.1."
                  << std::endl;
        std::cerr << "\tLOGFILE name of the log file. If ommitted, the log output "
                  << "will be performed to stderr." << std::endl;
        return 1;
    }

    hasLogFile = (argc == maxNbOfNeededArguments);
    return 0;
}
//======================================================================

int main(int argc, char** argv)
{
    ros::init(argc, argv, "idc_reader");
    ros::NodeHandle nh("~");

	std::cerr << argv[0] << " Version " << appVersion.toString();
	std::cerr << "  using IbeoSDK " << ibeoSDK.getVersion().toString() << std::endl;

	bool hasLogFile;
	const int checkResult = checkArguments(argc, argv, hasLogFile);
	if (checkResult != 0)
		exit(checkResult);
	int currArg = 1;

	std::string filename; //= argv[currArg++];
	int rate;
	nh.param<std::string>("filename", filename, "/media/jinbo/Work/Data_set/idc/20181030-170255.idc");
	nh.param<int>("rate", rate, 30);

	const off_t maxLogFileSize = 1000000;

	LogFileManager logFileManager;
	ibeosdk::LogFile::setTargetFileSize(maxLogFileSize);

	if (hasLogFile) {
		ibeosdk::LogFile::setLogFileBaseName(argv[currArg++]);
	}
	const ibeosdk::LogLevel ll = ibeosdk::logLevelFromString("Debug");
	ibeosdk::LogFile::setLogLevel(ll);

	static CustomLogStreamCallbackExample clsce;

	if (!hasLogFile)
		LogFile::setCustomLogStreamCallback(&clsce);

	logFileManager.start();

	if (hasLogFile) {
		logInfo << argv[0] << " Version " << appVersion.toString()
		        << "  using IbeoSDK " << ibeoSDK.getVersion().toString() << std::endl;
	}

    IdcFile file;
    file.open(filename);
    if (file.isOpen()) {
		AllListener allListener(nh, rate);

		file.registerListener(&allListener);

		const DataBlock* db = NULL;
		unsigned short nbMessages = 0; // # of messages we parsed

		while (file.isGood()) {
			db = file.getNextDataBlock();
			if (db == NULL) {
				continue; // might be eof or unknown file type
			}
			file.notifyListeners(db);
			++nbMessages;
		}

		logDebug << "EOF reached. " << nbMessages << " known blocks found." << std::endl;
	}
	else {
		logError << "File not readable." << std::endl;
	}

	exit(0);
}
//======================================================================

