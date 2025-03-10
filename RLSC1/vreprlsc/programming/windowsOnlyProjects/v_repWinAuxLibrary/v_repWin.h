// This file is part of V-REP, the Virtual Robot Experimentation Platform.
// 
// Copyright 2006-2013 Dr. Marc Andreas Freese. All rights reserved. 
// marc@coppeliarobotics.com
// www.coppeliarobotics.com
// 
// V-REP is dual-licensed, under the terms of EITHER (at your option):
//   1. V-REP commercial license (contact us for details)
//   2. GNU GPL (see below)
// 
// GNU GPL license:
// -------------------------------------------------------------------
// V-REP is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// V-REP is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
// 
// You should have received a copy of the GNU General Public License
// along with V-REP.  If not, see <http://www.gnu.org/licenses/>.
// -------------------------------------------------------------------
//
// This file was automatically created for V-REP release V3.0.4 on July 8th 2013

#pragma once

typedef UINT( __cdecl* AFX_THREADPROC_TT )( LPVOID );

#define VREPWIN_DLLEXPORT extern "C" __declspec(dllexport)

#ifndef __AFXWIN_H__
	#error "include 'stdafx.h' before including this file for PCH"
#endif

VREPWIN_DLLEXPORT void vrepWin_showConsole(char show);
VREPWIN_DLLEXPORT DWORD_PTR vrepWin_launchThread(AFX_THREADPROC_TT startAddress,int priority,DWORD_PTR affinityMask);
VREPWIN_DLLEXPORT DWORD_PTR vrepWin_setThreadAffinity(DWORD_PTR affinity);
VREPWIN_DLLEXPORT DWORD vrepWin_getCurrentThreadId();
VREPWIN_DLLEXPORT void vrepWin_switchThread();
VREPWIN_DLLEXPORT void vrepWin_sleep(int ms);
VREPWIN_DLLEXPORT int vrepWin_serialOpen(char* portString,int baudrate);
VREPWIN_DLLEXPORT char vrepWin_serialClose(int portHandle);
VREPWIN_DLLEXPORT int vrepWin_serialWrite(int portHandle,const char* buffer,int size);
VREPWIN_DLLEXPORT int vrepWin_serialCheck(int portHandle);
VREPWIN_DLLEXPORT int vrepWin_serialRead(int portHandle,char* buffer,int maxSize);
VREPWIN_DLLEXPORT void vrepWin_cleanUp();

