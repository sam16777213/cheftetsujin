#include <thread>
#include <sstream>
#include <QString>
#include <iostream>
#include "DobotDll.h"
#include "DobotType.h"
#include "vterm_colors.h"
using namespace std;
using namespace lacus;
void die(string message){
	vt0::log(cout,vt0::ERR,"takoyaki",message);
};
void warn(string message){
	vt0::log(cout,vt0::WARN,"takoyaki",message);
};
void info(string message){
	vt0::log(cout,vt0::INFO,"takoyaki",message);
};

static void setPTP(float x,float y,float z, float r){
	PTPCmd d;
	d.ptpMode=PTPMOVJXYZMode;
	d.x=x;
	d.y=y;
	d.z=z;
	d.r=r;
	while(SetPTPCmd(&d,true,NULL)!=::DobotCommunicate_NoError) ;
	info("moveCommand: X="+to_string(x)+" Y="+to_string(y)+" Z="+to_string(z)+" R="+to_string(r));
};

static void setInitialPTP(){
	setPTP(170, 9, 27, 49);
}

void initialize(){
	//Command timeout
	SetCmdTimeout(3000);
	//clear old commands and set the queued command running
	SetQueuedCmdClear();
	SetQueuedCmdStartExec();

	char deviceSN[64];
	GetDeviceName(deviceSN, sizeof(deviceSN));

	info(QString::fromUtf8(deviceSN).toStdString());
	char deviceName[64];
	GetDeviceName(deviceName, sizeof(deviceName));
	info(deviceName);
	uint8_t majorVersion, minorVersion, revision;
	GetDeviceVersion(&majorVersion, &minorVersion, &revision);
	cout<<QString::number(majorVersion).toStdString()<<QString::number(minorVersion).toStdString()<<QString::number(revision).toStdString()<<endl;
	//set the end effector parameters

	EndEffectorParams endEffectorParams;
	memset(&endEffectorParams, 0, sizeof(endEffectorParams));
	endEffectorParams.xBias = 71.6f;
	SetEndEffectorParams(&endEffectorParams, false, NULL);

	JOGJointParams jogJointParams;
	for (int i = 0; i < 4; i++) {
		jogJointParams.velocity[i] = 100;
		jogJointParams.acceleration[i] = 100;
	}
	SetJOGJointParams(&jogJointParams, false, NULL);

	JOGCoordinateParams jogCoordinateParams;
	for (int i = 0; i < 4; i++) {
		jogCoordinateParams.velocity[i] = 100;
		jogCoordinateParams.acceleration[i] = 100;
	}
	SetJOGCoordinateParams(&jogCoordinateParams, false, NULL);

	JOGCommonParams jogCommonParams;
	jogCommonParams.velocityRatio = 50;
	jogCommonParams.accelerationRatio = 50;
	SetJOGCommonParams(&jogCommonParams, false, NULL);

	PTPJointParams ptpJointParams;
	for (int i = 0; i < 4; i++) {
		ptpJointParams.velocity[i] = 100;
		ptpJointParams.acceleration[i] = 100;
	}
	SetPTPJointParams(&ptpJointParams, false, NULL);

	PTPCoordinateParams ptpCoordinateParams;
	ptpCoordinateParams.xyzVelocity = 100;
	ptpCoordinateParams.xyzAcceleration = 100;
	ptpCoordinateParams.rVelocity = 100;
	ptpCoordinateParams.rAcceleration = 100;
	SetPTPCoordinateParams(&ptpCoordinateParams, false, NULL);

	PTPJumpParams ptpJumpParams;
	ptpJumpParams.jumpHeight = 20;
	ptpJumpParams.zLimit = 150;
	SetPTPJumpParams(&ptpJumpParams, false, NULL);

	setInitialPTP();
};

static void getPose(){
	Pose pose;
	GetPose(&pose);
	stringstream m;
	for(int i=0;i<4;i++){
		m<<'['<<i<<']'<<pose.jointAngle[i]<<' ';
	};
	info(m.str());
};

int main(int argc,const char **argv){
	//connect
	if(argc<2) {
		die("usage: takoyaki <serialPortPath>");
		return -1;
	}
	info(("connecting to "+string(argv[1])).c_str());
	if(ConnectDobot(argv[1],115200)!=::DobotConnect_NoError){
		die("cannot connect");
		return -1;
	};
	initialize();
	getPose();
	setPTP(179,-10,0.95,0);
	this_thread::sleep_for(chrono::seconds(3));
	setInitialPTP();
	getPose();
	info("disconnecting");
	DisconnectDobot();
	return 0;
};
