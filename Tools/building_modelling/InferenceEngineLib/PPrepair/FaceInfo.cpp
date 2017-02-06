/*
 Copyright (c) 2009-2013,
 Ken Arroyo Ohori    g.a.k.arroyoohori@tudelft.nl
 Hugo Ledoux         h.ledoux@tudelft.nl
 Martijn Meijers     b.m.meijers@tudelft.nl
 All rights reserved.
 
 This file is part of pprepair: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.
 
 Licensees holding a valid commercial license may use this file in
 accordance with the commercial license agreement provided with
 the software.
 
 This file is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 */

#include "FaceInfo.h"
using namespace libPP;


FaceInfo::FaceInfo() {
	tag = NULL;
}

FaceInfo::~FaceInfo() 
{
	if (tag != NULL) {
		if (tag->isMultiPolygonHandle()) {
			delete tag;
		}
	}
}

/*
FaceInfo& FaceInfo::operator = (const FaceInfo& rInfo)
{
	if (this == &rInfo) return *this;

	if (rInfo.tag) {
		if (rInfo.tag->isMultiPolygonHandle())
			this->tag == new MultiPolygonHandle(rInfo.tag);
		else		
			this->tag == new PolygonHandle(*rInfo.tag);
	}
}*/

bool FaceInfo::operator == (const FaceInfo& rInfo) const
{
	if (this->numberOfTags()!=rInfo.numberOfTags()) return false;

	if (this->hasOneTag() && !this->hasTag(rInfo.getOneTag())) return false;
	if (rInfo.hasOneTag() && !rInfo.hasTag(this->getOneTag())) return false;
	
	return true;	
}

bool LessFieldPointer(Field* lhv, Field* rhv)
{
	return *lhv < *rhv;
}

bool FaceInfo::operator < (const FaceInfo& rInfo) const
{
	if (this->numberOfTags()!=rInfo.numberOfTags()) 
		return this->numberOfTags() < rInfo.numberOfTags();
	else {
		std::vector<Field*> vecFields0, vecFields1;
		for (unsigned int i=0; i<this->numberOfTags(); i++) {
			vecFields0.push_back(this->getOneTag()->getField(i));
			vecFields1.push_back(rInfo.getOneTag()->getField(i));
		}

		std::sort(vecFields0.begin(), vecFields0.end(), LessFieldPointer);
		std::sort(vecFields1.begin(), vecFields1.end(), LessFieldPointer);
		
		for (unsigned int i=0; i<this->numberOfTags(); i++) {
			if (LessFieldPointer(vecFields0[i], vecFields1[i]))
				return true;
		}
			
		return false;
	}
}

bool FaceInfo::hasTag(PolygonHandle *handle) const
{
	if (tag == NULL) return false;
	
	if (tag->isMultiPolygonHandle()) return static_cast<MultiPolygonHandle *>(tag)->hasHandle(handle);
	else if (tag == handle) return true;
	return false;
}

bool FaceInfo::hasNoTags() const 
{
	if (tag == NULL) return true;
	return false;
}

bool FaceInfo::hasOneTag() const 
{
	if (tag == NULL) return false;
	if (tag->isMultiPolygonHandle()) return false;
	return true;
}

unsigned int FaceInfo::numberOfTags() const 
{
	if (tag == NULL) return 0;
	if (tag->isMultiPolygonHandle()) return static_cast<MultiPolygonHandle *>(tag)->numberOfHandles();
	return 1;
}

void FaceInfo::addTag(PolygonHandle *handle) 
{
	if (!handle) return;
	
	if (tag == NULL) {
		if (!handle->isMultiPolygonHandle()) {
			tag = handle;
		}
		else {
			MultiPolygonHandle *multiTag = new MultiPolygonHandle(tag);
			multiTag->addHandle(handle);
			tag = multiTag;
		}
	}
	else if(!this->hasTag(handle)) {
		if (tag->isMultiPolygonHandle())
			static_cast<MultiPolygonHandle *>(tag)->addHandle(handle);
		else {
			//delete tag;
			MultiPolygonHandle *multiTag = new MultiPolygonHandle(tag);
			multiTag->addHandle(handle);
			tag = multiTag;
		}
	}
}

void FaceInfo::removeAllTags()
{
	if (tag == NULL) return;
	if (tag->isMultiPolygonHandle()) delete tag;
	tag = NULL;
}

void FaceInfo::substituteTagsWith(PolygonHandle *handle) 
{
	removeAllTags();
	addTag(handle);
}

PolygonHandle * FaceInfo::getOneTag() const 
{
    if (tag == NULL) return NULL;
	if (tag->isMultiPolygonHandle()) 
		return *static_cast<MultiPolygonHandle *>(tag)->getHandles()->begin();
	else
		return tag;
}

PolygonHandle * FaceInfo::getTags() const 
{
	return tag;
}

void FaceInfo::setTags(PolygonHandle *handle) 
{
	tag = handle;
}