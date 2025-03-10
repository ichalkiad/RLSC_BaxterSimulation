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

#pragma once

#include "Mesh.h"
#include <string>
#include <vector>
#include <boost/unordered_map.hpp>
#include "Material.h"
#include "SceneNode.h"
#include "VisualScene.h"
#include "mat4.h"
#include "tinyxml2.h"

class COLLADAExporter
{
private:
	
	int m_NextPosSourceID;
	int m_NextNormalSourceID;
	int m_NextGeometryID;
	int m_NextNodeID;

	std::vector<Mesh> m_Meshes;
	std::vector<std::pair<std::string,Material> > m_Materials;
	std::vector<VisualScene> m_VisualScenes;

	std::string m_PrimarySceneID;

	void SaveSceneNode(tinyxml2::XMLElement* parentNode,const SceneNode& scene);
	void SaveSceneNodes(tinyxml2::XMLElement* rootNode);
	void SaveMaterials(tinyxml2::XMLElement *rootNode);
	void SaveSource(tinyxml2::XMLElement* parentNode,const std::vector<vec3>& values,const std::string& sourceid,const std::string& sourcename);
	bool SaveGeometry(tinyxml2::XMLElement* geometriesNode,const Mesh& mesh);

public:	

	bool  SaveFile(const std::string& filename);
	const std::vector<Mesh>& getMeshes() const;	
	std::vector<Mesh>& getMeshes();

	const std::vector<std::pair<std::string,Material> >& getMaterials() const;
	std::vector<std::pair<std::string,Material> >& getMaterials();

	const std::vector<VisualScene>& getVisualScenes() const;
	std::vector<VisualScene>& getVisualScenes();

	const std::string& getPrimarySceneID() const;
	void setPrimarySceneID(const std::string& PrimarySceneID);
};
