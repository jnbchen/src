#include <ros/ros.h>
#include <string>

#include <AllListener.h>

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
    ros::init(argc, argv, "ibeo_listener");
    ros::NodeHandle nh("~");

    std::cerr << argv[0] << " Version " << appVersion.toString();
    std::cerr << " using IbeoSDK " << ibeoSDK.getVersion().toString() << std::endl;

    bool hasLogFile;
    const int checkResult = checkArguments(argc, argv, hasLogFile);
    /*if (checkResult != 0)
        exit(checkResult);*/
    
    int currArg = 1;

    const off_t maxLogFileSize = 1000000;

    LogFileManager logFileManager;
    ibeosdk::LogFile::setTargetFileSize(maxLogFileSize);

    if (!checkResult && hasLogFile)
    {
        ibeosdk::LogFile::setLogFileBaseName(argv[currArg++]);
    }

    const ibeosdk::LogLevel ll = ibeosdk::logLevelFromString("Debug");
    //const ibeosdk::LogLevel ll = ibeosdk::logLevelFromString("Warning");
    ibeosdk::LogFile::setLogLevel(ll);

    logFileManager.start();

    if (hasLogFile) {
        logInfo << argv[0] << " Version " << appVersion.toString()
                << "  using IbeoSDK " << ibeoSDK.getVersion().toString() << std::endl;
    }

    int port, rate;
    std::string ip;
    nh.param<int>("ibeo_had_port", port, 12002);
    nh.param<std::string>("ibeo_had_ip", ip, std::string("192.168.100.211"));
    nh.param<int>("rate", rate, 20);

    AllListener allEcuListener(nh);
    IbeoEcu ecu(ip, port);
    ecu.setLogFileManager(&logFileManager);

    ecu.registerListener(&allEcuListener);
    ecu.getConnected();

    //sleep(1);

    // Just to keep the program alive
    ros::Rate r(rate);

    while (true)
    {
      ros::spinOnce();
      if (!ecu.isConnected())
      {
        logInfo << "Ecu is disconnected, try to connect now..." << std::endl;
        ecu.getConnected();
        //sleep(1);
        continue;
      }
      r.sleep();
    } // while

    exit(0);
}

//==========================================
