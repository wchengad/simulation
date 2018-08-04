#ifndef __UAV_LOGGER_H
#define __UAV_LOGGER_H

#include <sstream>
#include "ros/console.h"
#include "ros/assert.h"

#define USE_ROSLOG
// #define USE_GLOGLOG

#ifdef USE_ROSLOG
	#define LOG(...) 		printf(__VA_ARGS__)
	#define LOGPUTS(...)	puts(__VA_ARGS__)
	#define LOGS(...)		do{std::stringstream ss; ss << __VA_ARGS__ << std::endl;std::cout<< ss;}while(0)
	#define LOGDEBUG(...) 	ROS_DEBUG(__VA_ARGS__)
	#define LOGINFO(...) 	ROS_INFO(__VA_ARGS__)
	#define LOGWARN(...) 	ROS_WARN(__VA_ARGS__)
	#define LOGERR(...) 	/*ROS_ERROR(__VA_ARGS__)*/
	#define LOGERR_STREAM(...) ROS_ERROR_STREAM(__VA_ARGS__)
	#define LOGPRT(...)		ROS_ERROR(__VA_ARGS__)
	#define LOGPRTS(...)	do{std::stringstream ss; ss << __VA_ARGS__ << std::endl;LOGERR_STREAM(ss.str().c_str());}while(0)
	#define LOGBOTH(...)    do{LOG(__VA_ARGS__);LOG("\n");LOGPRT(__VA_ARGS__);}while(0)
#else
	#define LOG(...)		
	#define LOGPUTS(...)	
	#define LOGS(...)		
	#define LOGDEBUG(...) 	
	#define LOGINFO(...) 	
	#define LOGWARN(...) 	
	#define LOGERR(...) 
	#define LOGERR_STREAM(...)	
	#define LOGPRT(...)		do{ROS_ERROR(__VA_ARGS__);}while(0);
	#define LOGPRTS(...)	do{std::stringstream ss; ss << __VA_ARGS__ << std::endl;LOGERR_STREAM(ss.str().c_str());}while(0)
	#define LOGBOTH(...)    do{LOG(__VA_ARGS__);LOG("\n");LOGPRT(__VA_ARGS__);}while(0)
#endif 	

#ifdef NDEBUG
	#define fassert()
#else
	#define fassert(cond,...)	do{\
									if (!(cond)){\
										LOGPRT(__VA_ARGS__);\
										assert(cond);\
									}\
								}while(0)

#endif

namespace uav_utils
{
typedef std::stringstream DebugSS_t;
}

#endif
