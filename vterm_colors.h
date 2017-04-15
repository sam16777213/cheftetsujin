#ifndef __VTCOLOR_H__
#define __VTCOLOR_H__

namespace lacus{
	namespace vt0{
		static const char *warning_message[]={
			"ERR!","WARN","INFO"," OK "
		};
		static const char 
			*defaultstyle="\e[0m", 	
			*bold="\e[1m",
			*underline="\e[2m",
			*hidden="\e[8m",

			*red="\e[91m",
			*green="\e[92m",
			*yellow="\e[33m",
			*brightyellow="\e[93m",
			*skyblue="\e[94m",
			*blue="\e[34m",
			*magenta="\e[95m",
			*cyan="\e[36m",
			*white="\e[97m";
		auto
			*ERR=&warning_message[0],
			*WARN=&warning_message[1],
			*INFO=&warning_message[2],
			*OK=&warning_message[3];
		template<typename OStream, typename String1, typename String2> void log(OStream& osm, const char ** type, const String1& source, const String2& message){
			using namespace lacus::vt0;
			const char *innerColor;
			switch(type-&warning_message[0]){
				case 0:
					innerColor=red;
					break;
				case 1:
					innerColor=brightyellow;
					break;
				case 2:
					innerColor=skyblue;
					break;
				default:
					innerColor=green;
			};
			osm<<white<< "[" << innerColor <<*type<<white<<  "] "<<magenta<<source<<red<<bold<<" : "<<defaultstyle<<message<<"\n" ;
		};
		
	};
};
#endif
