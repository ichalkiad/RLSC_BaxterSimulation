// Copyright 2006-2013 Dr. Marc Andreas Freese. All rights reserved. 
// marc@coppeliarobotics.com
// www.coppeliarobotics.com
// 
// -------------------------------------------------------------------
// This file is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
// 
// You are free to use/modify/distribute this file for whatever purpose!
// -------------------------------------------------------------------
//
// This file was automatically created for V-REP release V3.0.4 on July 8th 2013

#ifndef __PORTING_H__
#define __PORTING_H__

#ifdef _WIN32
	#pragma message("-----------------------> Adding library: Winmm.lib") 
	#pragma comment(lib,"Winmm.lib")
#endif /* _WIN32 */

#if defined (__linux) || defined (__APPLE__)

#include <sys/types.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/un.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>

#endif /* __linux || __APPLE__ */

#include <string>

#if defined (__linux) || defined (__APPLE__)

#define LINUX_NAMED_PIPES_DIR "/tmp/"
#define INVALID_SOCKET        -1
#define Sleep(x) (usleep(x*1000))
#define _stricmp(x,y) strcasecmp(x,y)

typedef int                 _SOCKET;
typedef struct timeval      _timeval;
typedef socklen_t           _socklen;

typedef unsigned char       BYTE;
typedef unsigned short      WORD;
typedef unsigned long       DWORD;
typedef unsigned short      u_short;

#endif /* __linux || __APPLE__ */

DWORD getTimeInMs();
DWORD getTimeDiffInMs(DWORD lastTime);

#endif /* __PORTING_H__ */
