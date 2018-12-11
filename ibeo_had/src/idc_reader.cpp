#include <ros/ros.h>
#include <string>

#include <ibeosdk/devices/IdcFile.hpp>

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
		AllListener allListener(nh);

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

