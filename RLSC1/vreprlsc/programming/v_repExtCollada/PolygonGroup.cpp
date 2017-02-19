// This file is part of the COLLADA PLUGIN for V-REP
// 
// Copyright 2006-2013 Dr. Marc Andreas Freese. All rights reserved. 
// marc@coppeliarobotics.com
// www.coppeliarobotics.com
// 
// The COLLADA PLUGIN is licensed under the terms of GNU GPL:
// 
// -------------------------------------------------------------------
// The COLLADA PLUGIN is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// The COLLADA PLUGIN is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
// 
// You should have received a copy of the GNU General Public License
// along with the COLLADA PLUGIN.  If not, see <http://www.gnu.org/licenses/>.
// -------------------------------------------------------------------
//
// This file was automatically created for V-REP release V3.0.4 on July 8th 2013

// Written by Alex Doumanoglou on behalf of Dr. Marc Freese

#include "PolygonGroup.h"

std::string PolygonGroup::getMaterialSymbol() const
{
	return m_MaterialSymbol;
}

void PolygonGroup::setMaterialSymbol(const std::string& MaterialSymbol)
{
	m_MaterialSymbol = MaterialSymbol;
}

std::vector<vec3>& PolygonGroup::getNormals()
{
	return m_Normals;
}

const std::vector<vec3>& PolygonGroup::getNormals() const
{
	return m_Normals;
}
 
std::vector<PPolygon>& PolygonGroup::getPolygons()
{
	return m_Polygons;
}

const std::vector<PPolygon>& PolygonGroup::getPolygons() const
{
	return m_Polygons;
}
