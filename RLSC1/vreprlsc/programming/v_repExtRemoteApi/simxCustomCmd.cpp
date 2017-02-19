// This file is part of the REMOTE API
// 
// Copyright 2006-2013 Dr. Marc Andreas Freese. All rights reserved. 
// marc@coppeliarobotics.com
// www.coppeliarobotics.com
// 
// The REMOTE API is licensed under the terms of GNU GPL:
// 
// -------------------------------------------------------------------
// The REMOTE API is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// The REMOTE API is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
// 
// You should have received a copy of the GNU General Public License
// along with the REMOTE API.  If not, see <http://www.gnu.org/licenses/>.
// -------------------------------------------------------------------
//
// This file was automatically created for V-REP release V3.0.5 on October 26th 2013

/* Your custom remote API functions. Following are examples: */

if (_rawCmdID==simx_customcmd_get_object_count)
{
	// Get the total number of objects in the scene (i.e. execute the command):
	int index=0;
	while (simGetObjects(index,sim_handle_all)!=-1)
		index++;

	// We now need to return that number (index):
	retCmd->setDataReply_1int(index,true,otherSideIsBigEndian); // Endianness of the client is detected and automatically adjusted (for simple data replies, otherwise you'll have to do that yourself)
}

if (_rawCmdID==simx_customcmd_get_object_type)
{
	// First retrieve the data that is part of the command (i.e. the handle of the object). The server is in charge of handling little endian to big endian conversions:
	int handle=littleEndianIntConversion(((int*)(_cmdData+0))[0],otherSideIsBigEndian);

	// Execute the command:
	int objType=simGetObjectType(handle);
	bool success=(objType!=-1);

	// We now need to return the type:
	retCmd->setDataReply_1int(objType,success,otherSideIsBigEndian); // Endianness of the client is detected and automatically adjusted (for simple data replies, otherwise you'll have to do that yourself)
}

if (_rawCmdID==simx_customcmd_set_object_name)
{
	// First retrieve the data that is part of the command (i.e. the handle of the object). The server is in charge of handling little endian to big endian conversions:
	int handle=littleEndianIntConversion(((int*)(_cmdData+0))[0],otherSideIsBigEndian);

	// The new object name is pure data (i.e. not part of the command). The pure data is located in _data
	
	// Execute the command:
	bool success=(simSetObjectName(handle,_pureData)!=-1);

	// We now need to return an empty command reply, just indicating the success of the command:
	retCmd->setDataReply_nothing(success);
}

if (_rawCmdID==simx_customcmd_get_ui_button_label)
{
	// First retrieve the data that is part of the command (i.e. the handle of the UI and the ID of the button). The server is in charge of handling little endian to big endian conversions:
	int uiHandle=littleEndianIntConversion(((int*)(_cmdData+0))[0],otherSideIsBigEndian);
	int buttonID=littleEndianIntConversion(((int*)(_cmdData+0))[1],otherSideIsBigEndian);

	// Execute the command:
	char* label=simGetUIButtonLabel(uiHandle,buttonID);

	// We now need to return the label (if the function was successful):
	if (label!=NULL)
	{
		retCmd->setDataReply_custom_copyBuffer(label,int(strlen(label)+1),true); // we return the string (including terminal zero) as pure data
		simReleaseBuffer(label); // do not forget to release that buffer!
	}
	else
		retCmd->setDataReply_nothing(false); // we do not return anything, we just indicate that the function failed on the server side
}

if (_rawCmdID==simx_customcmd_get_script_handle)
{
	// The data that is part of the command (i.e. the name of the object) is stored in _cmdString

	// Execute the command now:
	int h=simGetObjectHandle(_cmdString.c_str());
	int scriptHandle=-1;
	if (h!=-1)
		scriptHandle=simGetScriptAssociatedWithObject(h);

	// We now need to return the script handle:
	retCmd->setDataReply_1int(scriptHandle,scriptHandle!=-1,otherSideIsBigEndian); // Endianness of the client is detected and automatically adjusted (for simple data replies, otherwise you'll have to do that yourself)
}

if (_rawCmdID==simx_cmd_get_object_quaternion)
{
	int handle=littleEndianIntConversion(((int*)(_cmdData+0))[0],otherSideIsBigEndian);
	int relativeToObject=littleEndianIntConversion(((int*)(_pureData+0))[0],otherSideIsBigEndian);
	float quat[4];
	bool success=(simGetObjectQuaternion(handle,relativeToObject,quat)!=-1);
	// Endian conversion on the client side!
	retCmd->setDataReply_custom_copyBuffer((char*)quat,4*4,success);
}

if (_rawCmdID==simx_cmd_set_object_orientation)
{
	int handle=littleEndianIntConversion(((int*)(_cmdData+0))[0],otherSideIsBigEndian);
	int relativeToObject=littleEndianIntConversion(((int*)(_pureData+0))[0],otherSideIsBigEndian);
	float quat[4];
	quat[0]=littleEndianFloatConversion(((float*)(_pureData+0))[1],otherSideIsBigEndian);
	quat[1]=littleEndianFloatConversion(((float*)(_pureData+0))[2],otherSideIsBigEndian);
	quat[2]=littleEndianFloatConversion(((float*)(_pureData+0))[3],otherSideIsBigEndian);
	quat[3]=littleEndianFloatConversion(((float*)(_pureData+0))[4],otherSideIsBigEndian);
	bool success=(simSetObjectQuaternion(handle,relativeToObject,quat)!=-1);
	retCmd->setDataReply_nothing(success);
}

