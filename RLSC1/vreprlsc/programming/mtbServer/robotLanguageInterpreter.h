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
#include <string>
#include <vector>

struct SCompiledProgramLine
{
	std::string correspondingUncompiledCode;
	int command; /*0=MOVE, 1=WAIT, 2=GOTO, 3=SETROTVEL, etc. */
	int intParameter[2]; /* intParameter[0]: If command==GOTO, then compiled prg line to jump to. if command==SETBIT or CLEARBIT, then the bit number to set/clear (1-32)  */
						 /* intParameter[1]: If command==IFBITGOTO or IFNBITGOTO, then compiled prg line to jump to  */
	float floatParameter[4]; /*if command==MOVE, then target joint x value (rad or meter), if command==SETLINVEL/SETROTVEL, then desired joint velocity (m/s / rad/s), if command==WAIT, then time to wait (s). */
	std::string tmpLabel;
};

struct SMtbRobotState
{
	float currentJointPosition[4]; // The joint values for the robot
	float jointVelocityWhenMoving[2]; // The joint velocity to use for robot movements (linear and angular)
	int currentProgramLine; // The current program counter (number is relative to the compiled program!)
	float timeAlreadySpentAtCurrentProgramLine; // Some instructions require longer execution times than the simulation time step
	unsigned char outputData[4]; // The data that the robot (or controller) can output (e.g. signals, alarms, etc.). 
};

std::string compileCode(const std::string& inputCode,float initialJointPosition[4],float initialJointVelocityWhenMoving[2]);
std::string runProgram(unsigned char inputData[4],float deltaTime);
void getJointsAndOutputData(float jointPosition[4],unsigned char outputData[4]);
