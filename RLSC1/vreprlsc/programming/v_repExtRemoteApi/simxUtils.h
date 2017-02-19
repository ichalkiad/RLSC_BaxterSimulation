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
// This file was automatically created for V-REP release V3.0.4 on July 8th 2013

#pragma once
#include "porting.h"

short littleEndianShortConversion(short v,bool otherSideIsBigEndian);
WORD littleEndianWordConversion(WORD v,bool otherSideIsBigEndian);
int littleEndianIntConversion(int v,bool otherSideIsBigEndian);
float littleEndianFloatConversion(float v,bool otherSideIsBigEndian);
double littleEndianDoubleConversion(double v,bool otherSideIsBigEndian);
WORD getCRC(const char* data,int length);
