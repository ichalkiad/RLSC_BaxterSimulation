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

#pragma once

#include "porting.h"

// Never change following data, data is serialized together with BubbleRob!
#define DEVELOPER_DATA_HEADER 0 // Keep always same for all modules created by you!!! If possible, use your V-REP copy's serial number for that header to avoid clashes
#define	BUBBLEROB_DATA_MAXVELOCITY	0 // defines the base of BubbleRob AND its max velocity
#define BUBBLEROB_DATA_LEFTMOTOR	1 // defines the left motor of BubbleRob
#define BUBBLEROB_DATA_RIGHTMOTOR	2 // defines the right motor of BubbleRob
#define BUBBLEROB_DATA_SENSOR		3 // defines the proximity sensor of BubbleRob
// Here you could add other entries, for another plugin, e.g.:
//#define	WALKINGROB_DATA_STEPSIZE	100 // Make sure we leave enough data IDs for BubbleRob for future extensions

#include <vector>
#include "bubbleRobContainer.h"
#ifdef _WIN32 // only the Windows version has a dialog!
	#include "bubbleRobDialog.h"
#endif // _WIN32

class CAccess  
{
public:
	CAccess();
	virtual ~CAccess();

	static void createNonGui();
	static void destroyNonGui();
	static void createGui();
	static void destroyGui();

	// Following functions are helper functions to insert/extract items to/from scene objects.
	// Following is how this developer stores data under his/her header:
	// data saved under the DEVELOPER_DATA_HEADER header: item1ID,item1DataLengthInBytes,item1Data,item2ID,item2DataLengthInBytes,item2Data, etc.
	static void insertSerializationData(std::vector<unsigned char>& buffer,int dataName,const std::vector<unsigned char>& data);
	static bool extractSerializationData(std::vector<unsigned char>& buffer,int dataName,std::vector<unsigned char>& data);
	static int getDataLocationAndSize(const std::vector<unsigned char>& buffer,int dataName,int& theSize);
	static void push_int(std::vector<unsigned char>& buffer,int data);
	static void push_float(std::vector<unsigned char>& buffer,float data);
	static int pop_int(std::vector<unsigned char>& buffer);
	static float pop_float(std::vector<unsigned char>& buffer);

	static CBubbleRobContainer* bubbleRobContainer; // this is the bubbleRobContainer

#ifdef _WIN32 // only the Windows version has a dialog!
	static CBubbleRobDialog* bubbleRobDialog; // This is the dialog for BubbleRob
#endif // _WIN32

};
