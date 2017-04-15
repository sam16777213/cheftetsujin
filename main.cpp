#include <iostream>
#include "DobotDll.h"
#include "DobotType.h"
#include "vterm_colors.h"
using namespace std;
using namespace lacus;
void die(const char* message){
	vt0::log(cout,vt0::ERR,"takoyaki",message);
};
void warn(const char *message){
	vt0::log(cout,vt0::WARN,"takoyaki",message);
};
void info(const char *message){
	vt0::log(cout,vt0::INFO,"takoyaki",message);
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
	info("disconnecting");
	DisconnectDobot();
	return 0;
};
