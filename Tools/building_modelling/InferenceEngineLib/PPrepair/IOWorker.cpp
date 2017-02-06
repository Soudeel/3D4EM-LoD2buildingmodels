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

#include "IOWorker.h"
using namespace libPP;
using namespace std;


bool DebugVertex(Point v, double inX, double inY) 
{
	double dist = CGAL::to_double(CGAL::squared_distance(v, Point(inX, inY)));
	return dist<0.001;
}


bool DebugEdge(Point v1, Point v2, double inX1, double inY1, double inX2, double inY2)
{
	bool same11 = DebugVertex(v1, inX1, inY1); 
	bool same12 = DebugVertex(v1, inX2, inY2);
	bool same21 = DebugVertex(v2, inX1, inY1);
	bool same22 = DebugVertex(v2, inX2, inY2);
	return (same11&&same22)||(same12&&same21);
}

bool DebugTriangle(Triangulation::Face_handle f, double inX1, double inY1, double inX2, double inY2, double inX3, double inY3)
{
	bool same11 = DebugVertex(f->vertex(0)->point(), inX1, inY1);
	bool same12 = DebugVertex(f->vertex(0)->point(), inX2, inY2);
	bool same13 = DebugVertex(f->vertex(0)->point(), inX3, inY3);
	bool same21 = DebugVertex(f->vertex(1)->point(), inX1, inY1);
	bool same22 = DebugVertex(f->vertex(1)->point(), inX2, inY2);
	bool same23 = DebugVertex(f->vertex(1)->point(), inX3, inY3);
	bool same31 = DebugVertex(f->vertex(2)->point(), inX1, inY1);
	bool same32 = DebugVertex(f->vertex(2)->point(), inX2, inY2);
	bool same33 = DebugVertex(f->vertex(2)->point(), inX3, inY3);

	return (same11&&same22&&same33)||(same11&&same23&&same32)//1-1 2-2 3-3, 1-1 2-3 3-2
		||(same12&&same21&&same33)||(same12&&same23&&same31)//1-2 2-1 3-3, 1-2 2-3 3-1
		||(same13&&same21&&same32)||(same13&&same22&&same31);//1-3 2-1 3-2, 1-3 2-2 3-1
}


IOWorker::IOWorker() {
    startingSearchFace = Triangulation::Face_handle();
}

bool IOWorker::addToTriangulation(Triangulation &triangulation, TaggingVector &edgesToTag, const char *file, unsigned int schemaIndex) 
{
    // Open file
	OGRDataSource *dataSource = OGRSFDriverRegistrar::Open(file, false);
	if (dataSource == NULL) {
		std::cerr << "Error: Could not open file." << std::endl;
		return false;
	}
    
    char *name = new char[strlen(dataSource->GetName())+1];
	strcpy(name, dataSource->GetName());
	fileNames.push_back(name);
	//std::cout << "\tPath: " << name << std::endl;
	//std::cout << "\tType: " << dataSource->GetDriver()->GetName() << std::endl;
	int numberOfLayers = dataSource->GetLayerCount();
	//std::cout << "\tLayers: " << numberOfLayers << std::endl;
    
    // Read layer by layer
    for (int currentLayer = 0; currentLayer < numberOfLayers; currentLayer++) {
        OGRLayer *dataLayer = dataSource->GetLayer(currentLayer);
		dataLayer->ResetReading();
		
		unsigned int numberOfPolygons = dataLayer->GetFeatureCount(true);
		//std::cout << "\tReading layer #" << currentLayer+1 << " (" << numberOfPolygons << " polygons)...";
		polygons.reserve(polygons.size()+numberOfPolygons);
        
        // Check fields and the schema type
        OGRFeatureDefn *layerDefinition = dataLayer->GetLayerDefn();
        insertToStream(std::cout, layerDefinition, 1, schemaIndex);
        
        // If it's the first input file, assign the schema type of it
        if (triangulation.number_of_faces() == 0) {
            schemaFieldType = layerDefinition->GetFieldDefn(schemaIndex)->GetType();
        } // Otherwise, check if it matches the previous one
        else {
            if (layerDefinition->GetFieldDefn(schemaIndex)->GetType() != schemaFieldType) {
                std::cerr << "\tError: The schema field type in this layer is incompatible with the previous one. Skipped." << std::endl;
                continue;
            }
        }
        
        // Save the field names and types
		for (int currentField = 0; currentField < layerDefinition->GetFieldCount(); currentField++) {
			OGRFieldDefn *fieldDefinition = layerDefinition->GetFieldDefn(currentField);
			FieldDefinition *newField = new FieldDefinition(fieldDefinition->GetNameRef(), fieldDefinition->GetType(), fieldDefinition->GetJustify(), fieldDefinition->GetWidth(), fieldDefinition->GetPrecision());
			unsigned int currentCheck;
			for (currentCheck = 0; currentCheck < fields.size(); currentCheck++) {
				if (newField->matches(fields[currentCheck])) break;
			} if (currentCheck == (unsigned int)fields.size()) {
				// It's a new field
				fields.push_back(newField);
				fieldEquivalencies[FieldDescriptor(name, currentLayer, currentField)] = ((unsigned int)fields.size())-1;
			} else {
				// The field matches an older one, don't add
				delete newField;
				fieldEquivalencies[FieldDescriptor(name, currentLayer, currentField)] = currentCheck;
			}
		}
        
        // Reads all features in this layer
		OGRFeature *feature;
		while ((feature = dataLayer->GetNextFeature()) != NULL) {
			
			// STEP 1: Get polygons from input
			std::vector<std::list<Point> > outerRingsList;
			std::vector<std::list<Point> > innerRingsList;
			switch(feature->GetGeometryRef()->getGeometryType()) {
                    
                // Most typical case, receiving polygons
                case wkbPolygon: {
					OGRPolygon *geometry = static_cast<OGRPolygon *>(feature->GetGeometryRef());
					outerRingsList.push_back(std::list<Point>());
					
					// Get outer ring
					for (int currentPoint = 0; currentPoint < geometry->getExteriorRing()->getNumPoints(); currentPoint++)
						outerRingsList.back().push_back(Point(geometry->getExteriorRing()->getX(currentPoint), 
															  geometry->getExteriorRing()->getY(currentPoint)));
					
					// Get inner rings
					innerRingsList.reserve(geometry->getNumInteriorRings());
					for (int currentRing = 0; currentRing < geometry->getNumInteriorRings(); currentRing++) {
						innerRingsList.push_back(std::list<Point>());
						for (int currentPoint = 0; currentPoint < geometry->getInteriorRing(currentRing)->getNumPoints(); currentPoint++) {
							innerRingsList.back().push_back(Point(geometry->getInteriorRing(currentRing)->getX(currentPoint), 
																  geometry->getInteriorRing(currentRing)->getY(currentPoint)));
						}
					} break;
				}
					
                // Receiving multi polygons
				case wkbMultiPolygon: {
					OGRMultiPolygon *geometry = static_cast<OGRMultiPolygon *>(feature->GetGeometryRef());
					
					// Check each polygon
					for (int currentPolygon = 0; currentPolygon < geometry->getNumGeometries(); currentPolygon++) {
						OGRPolygon *thisGeometry = static_cast<OGRPolygon *>(geometry->getGeometryRef(currentPolygon));
						outerRingsList.push_back(std::list<Point>());
						
						// Get outer ring
						for (int currentPoint = 0; currentPoint < thisGeometry->getExteriorRing()->getNumPoints(); currentPoint++)
							outerRingsList.back().push_back(Point(thisGeometry->getExteriorRing()->getX(currentPoint), 
																  thisGeometry->getExteriorRing()->getY(currentPoint)));
						
						// Get inner rings
						innerRingsList.reserve(innerRingsList.size()+thisGeometry->getNumInteriorRings());
						for (int currentRing = 0; currentRing < thisGeometry->getNumInteriorRings(); currentRing++) {
							innerRingsList.push_back(std::list<Point>());
							for (int currentPoint = 0; currentPoint < thisGeometry->getInteriorRing(currentRing)->getNumPoints(); currentPoint++) {
								innerRingsList.back().push_back(Point(thisGeometry->getInteriorRing(currentRing)->getX(currentPoint),
																	  thisGeometry->getInteriorRing(currentRing)->getY(currentPoint)));
							}
						}
					} break;
				}
                    
				default:
					std::cerr << "\tFeature #" << feature->GetFID() << ": unsupported type (";
					insertToStream(std::cout, feature->GetGeometryRef()->getGeometryType());
					std::cerr << "). Skipped." << std::endl;
					continue;
					break;
                    
                // TODO: Implement other cases: points, lines, containers with multiple features, etc.
			}
			
			// STEP 2: Check validity of individual polygons
			//  it's more efficient doing this during creation, but this is more readable and maintainable (check SVN v59).
			//  After all, this is not the main focus here.
			
			std::vector<Polygon> polygonsVector;
			// CHECKS ON VERTICES
            
            // Remove repeated vertices. One per ring is considered normal, since some specifications allow or require it (first == last).
            for (unsigned int currentRing = 0; currentRing < outerRingsList.size(); ++currentRing) {
                if (removeDuplicateVertices(outerRingsList[currentRing]) > 1)
                    std::cout << "\tFeature #" << feature->GetFID() << ": duplicate vertices in outer boundary #" << currentRing << ". Removed duplicates." << std::endl;
            } for (unsigned int currentRing = 0; currentRing < innerRingsList.size(); ++currentRing) {
                if (removeDuplicateVertices(innerRingsList[currentRing]) > 1)
                    std::cout << "\tFeature #" << feature->GetFID() << ": duplicate vertices in inner boundary #" << currentRing << ". Removed duplicates." << std::endl;
            }
            
            // Trivial check for rings with less than 3 vertices. The ones with 3 or more vertices will be done in the triangulation
            for (int currentRing = 0; currentRing < (int)outerRingsList.size(); ++currentRing) {
                if (outerRingsList[currentRing].size() < 3) {
                    std::cout << "\tFeature #" << feature->GetFID() << ": less than 3 vertices in outer boundary #" << currentRing << ". Removed." << std::endl;
                    outerRingsList.erase(outerRingsList.begin()+currentRing);
                    --currentRing;
                }
            } for (int currentRing = 0; currentRing < (int)innerRingsList.size(); ++currentRing) {
                if (innerRingsList[currentRing].size() < 3) {
                    std::cout << "\tFeature #" << feature->GetFID() << ": less than 3 vertices in inner boundary #" << currentRing << ". Removed." << std::endl;
                    innerRingsList.erase(innerRingsList.begin()+currentRing);
                    --currentRing;
                }
            }
            
            // CHECKS ON RINGS
            
            // Let's move on to the CGAL data structures for Rings
            std::vector<Ring> outerRings;
            std::vector<Ring> innerRings;
            outerRings.reserve(outerRingsList.size());
            innerRings.reserve(innerRingsList.size());
            for (unsigned int currentRing = 0; currentRing < outerRingsList.size(); currentRing++) {
                outerRings.push_back(Ring(outerRingsList[currentRing].begin(), outerRingsList[currentRing].end()));
                outerRingsList[currentRing].clear();
            } for (unsigned int currentRing = 0; currentRing < innerRingsList.size(); currentRing++) {
                innerRings.push_back(Ring(innerRingsList[currentRing].begin(), innerRingsList[currentRing].end()));
                innerRingsList[currentRing].clear();
            }
            
            // Split self touching rings and correct winding
            std::vector<Ring *> outerRingsToBuild;
            std::vector<Ring *> innerRingsToClassify;
            std::vector<std::vector<Ring> > innerRingsToBuild;
            
            // Get outer rings
            for (unsigned int currentRings = 0; currentRings < outerRings.size(); currentRings++) {
                if (!outerRings[currentRings].is_simple()) {
                    std::cout << "\tFeature #" << feature->GetFID() << " (" << outerRings[currentRings].size() << " vertices): self intersecting outer boundary #" << currentRings << ". Split." << std::endl;
                    std::vector<Ring *> receivedRings = splitRing(outerRings[currentRings]);
                    for (std::vector<Ring *>::iterator currentRing = receivedRings.begin(); currentRing != receivedRings.end(); ++currentRing) {
                        if ((*currentRing)->is_clockwise_oriented()) {
                            outerRingsToBuild.push_back(*currentRing);
                        } else {
                            innerRingsToClassify.push_back(*currentRing);
                        }
                    }
                } else {
                    if (outerRings[currentRings].is_counterclockwise_oriented()) {
                        std::cout << "\tFeature #" << feature->GetFID() << ": incorrect winding in outer boundary #" << currentRings << ". Reversed." << std::endl;
                        outerRings[currentRings].reverse_orientation();
                    } outerRingsToBuild.push_back(new Ring(outerRings[currentRings]));
                    outerRings[currentRings].clear();
                } 
            }
            
            // Get inner rings
            for (unsigned int currentRings = 0; currentRings < innerRings.size(); currentRings++) {
                if (!innerRings[currentRings].is_simple()) {
                    std::cout << "\tFeature #" << feature->GetFID() << " (" << innerRings[currentRings].size() << " vertices): self intersecting inner boundary #" << currentRings << ". Split." << std::endl;
                    std::vector<Ring *> receivedRings = splitRing(innerRings[currentRings]);
                    for (std::vector<Ring *>::iterator currentRing = receivedRings.begin(); currentRing != receivedRings.end(); ++currentRing) {
                        if ((*currentRing)->is_clockwise_oriented()) {
                            innerRingsToClassify.push_back(*currentRing);
                        } else {
                            outerRingsToBuild.push_back(*currentRing);
                        }
                    }
                } else {
                    if (innerRings[currentRings].is_clockwise_oriented()) {
                        std::cout << "\tFeature #" << feature->GetFID() << ": incorrect winding in inner boundary #" << currentRings << ". Reversed." << std::endl;
                        innerRings[currentRings].reverse_orientation();
                    } innerRingsToClassify.push_back(new Ring(innerRings[currentRings]));
                    innerRings[currentRings].clear();
                }
            }
            
            // Make space for inner rings
            for (std::vector<Ring *>::iterator currentRing = outerRingsToBuild.begin(); currentRing != outerRingsToBuild.end(); ++currentRing) {
                innerRingsToBuild.push_back(std::vector<Ring>());
            } 
            
            // Put inner rings into the correct outer ring (and likely other ones). Incorrectly nested rings are found here.
            if (outerRingsToBuild.size() == 0) {
                // Outer ring had no area or there wasn't any. Delete all inner rings
                std::cout << "\tFeature #" << feature->GetFID() << ": zero area outer boundary. Inner boundaries removed." << std::endl;
                for (std::vector<Ring *>::iterator currentRing = innerRingsToClassify.begin(); currentRing != innerRingsToClassify.end(); ++currentRing) {
                    delete *currentRing;
                }
            } 
            
            // Now check them and put them in place
            else if (innerRingsToClassify.size() > 0) {
                testRings(outerRingsToBuild, innerRingsToClassify, innerRingsToBuild, feature->GetFID());
            }
            
            // Let's move on to CGAL data structures for Polygons
            for (unsigned int currentPolygon = 0; currentPolygon < outerRingsToBuild.size(); ++currentPolygon) {
                polygonsVector.push_back(Polygon(*outerRingsToBuild[currentPolygon], innerRingsToBuild[currentPolygon].begin(), innerRingsToBuild[currentPolygon].end()));
            } outerRingsToBuild.clear();
            innerRingsToBuild.clear();
			
			// STEP 3: Introduce edges as constraints in the triangulation
            for (std::vector<Polygon>::iterator currentPolygon = polygonsVector.begin(); currentPolygon != polygonsVector.end(); ++currentPolygon) {
				
				// Create and save polygon handle
				PolygonHandle *handle = new PolygonHandle(schemaIndex, fileNames.back(), currentLayer, feature->GetFID());
				polygons.push_back(handle);
				
				// Save other attributes to put back later
				copyFields(feature, handle);
				
				// Create edges vector for this handle
				edgesToTag.push_back(std::pair<std::vector<Triangulation::Vertex_handle>, std::vector<std::vector<Triangulation::Vertex_handle> > >());
				
				// Insert edges into the triangulation and edges vector
				for (Ring::Edge_const_iterator currentEdge = currentPolygon->outer_boundary().edges_begin(); 
					 currentEdge != currentPolygon->outer_boundary().edges_end(); 
					 ++currentEdge) {
					Triangulation::Vertex_handle sourceVertex = triangulation.insert(currentEdge->source(), startingSearchFace);
                    startingSearchFace = triangulation.incident_faces(sourceVertex);
					Triangulation::Vertex_handle targetVertex = triangulation.insert(currentEdge->target(), startingSearchFace);
					triangulation.insert_constraint(sourceVertex, targetVertex);
                    startingSearchFace = triangulation.incident_faces(targetVertex);
					edgesToTag.back().first.push_back(sourceVertex);
				} for (Polygon::Hole_const_iterator currentRing = currentPolygon->holes_begin(); currentRing != currentPolygon->holes_end(); ++currentRing) {
					edgesToTag.back().second.push_back(std::vector<Triangulation::Vertex_handle>());
					for (Ring::Edge_const_iterator currentEdge = currentRing->edges_begin(); currentEdge != currentRing->edges_end(); ++currentEdge) {
						Triangulation::Vertex_handle sourceVertex = triangulation.insert(currentEdge->source(), startingSearchFace);
                        startingSearchFace = triangulation.incident_faces(sourceVertex);
						Triangulation::Vertex_handle targetVertex = triangulation.insert(currentEdge->target(), startingSearchFace);
						triangulation.insert_constraint(sourceVertex, targetVertex);
                        startingSearchFace = triangulation.incident_faces(targetVertex);
						edgesToTag.back().second.back().push_back(sourceVertex);
					}
				}
			}
			
			// Free memory
			polygonsVector.clear();
			
			// Free OGR feature
			OGRFeature::DestroyFeature(feature);
		}
    }
    
    // Free OGR data source
	OGRDataSource::DestroyDataSource(dataSource);
    
    return true;
}

#include "ObjectPoints.h"
#include "LineTopologies.h"
#include "ObjectPoint.h"

bool IOWorker::addToTriangulation(Triangulation &triangulation, TaggingVector &edgesToTag,
	const ObjectPoints& localObjPnts, const LineTopologies& localTops) 
{
	unsigned int schemaIndex=0;
	unsigned int numberOfPolygons = localTops.size();
	polygons.reserve(polygons.size()+numberOfPolygons);
	OGRFeature *feature = NULL;
	int compValue;

	/*
#ifdef _DEBUG
	localTops.Write("debug.top");
	localObjPnts.Write("debug.objpts");

	vector<Ring> debPols;
	Ring debPolygon;
	for (int i=0; i<localTops.size(); i++)	 {
		debPolygon.clear();
		for (int j=0; j<localTops[i].size()-1; j++) {
			const ObjectPoint& pnt = localObjPnts[localTops[i][j].Number()];
			debPolygon.push_back(Point(pnt.X(), pnt.Y()));
		}
		debPols.push_back(debPolygon);
	}
		
	for (vector<Ring>::iterator itrPol=debPols.begin(); itrPol!=debPols.end(); itrPol++) {
		int loop = 0;
		for (Ring::Edge_const_iterator itrEdge = itrPol->edges_begin(); itrEdge != itrPol->edges_end(); ++itrEdge) {
			Triangulation::Vertex_handle sourceVertex = triangulation.insert(itrEdge->source(), startingSearchFace);
			startingSearchFace = triangulation.incident_faces(sourceVertex);
			Triangulation::Vertex_handle targetVertex = triangulation.insert(itrEdge->target(), startingSearchFace);
			triangulation.insert_constraint(sourceVertex, targetVertex);
			startingSearchFace = triangulation.incident_faces(targetVertex);
			loop++;
			exportTriangulation(triangulation, "debug_1.dxf", true, false, false);
			Point p11 = sourceVertex->point();
			Point p12 = targetVertex->point();
			Point p21 = itrEdge->source();
			Point p22 = itrEdge->target();
			int aaa = 0;
		}
		int aaa = 0;
		
	}

	//vertexes
	std::ofstream vertexStream("vertexesIN.txt");
	vertexStream.setf(std::ios::fixed, std:: ios::floatfield);
	for (vector<Ring>::iterator itrPol=debPols.begin(); itrPol!=debPols.end(); itrPol++) {
		for (Ring::Vertex_iterator v = itrPol->vertices_begin(); v != itrPol->vertices_end(); ++v)
			vertexStream<<v->x()<<" "<<v->y()<<" "<<0.0<<endl; 
	}
	vertexStream.close();

	//polygons for test_CDT
	std::ofstream polygonStream("debug.pnt");
	polygonStream.setf(std::ios::fixed, std:: ios::floatfield);
	for (vector<Ring>::iterator itrPol=debPols.begin(); itrPol!=debPols.end(); itrPol++) {
		polygonStream<<"polygon:"<<endl;
		for (Ring::Vertex_iterator v = itrPol->vertices_begin(); v != itrPol->vertices_end(); ++v)
			polygonStream<<v->x()<<" "<<v->y()<<" "<<0.0<<endl; 
		polygonStream<<"end"<<endl<<endl;;
	}
	polygonStream.close();

	return true;
#endif*/

	for (int i=0; i<localTops.size(); i++)	 {
		//////////////////////////////////////////////////////////////////////////
		// STEP 1: Get polygons from input
		std::vector<std::list<Point> > outerRingsList;
		std::vector<std::list<Point> > innerRingsList;
		if(localTops[i].HasAttribute(BuildingPartNumberTag))
			compValue = localTops[i].Attribute(BuildingPartNumberTag);
		else
			compValue = -1;

		// Get outer rings
		std::list<Point> ring;
		for (int j=0; j<localTops[i].size()-1; j++) {
			const ObjectPoint& pnt = localObjPnts[localTops[i][j].Number()];
			ring.push_back(Point(pnt.X(), pnt.Y()));
		}
		outerRingsList.push_back(ring);

		//////////////////////////////////////////////////////////////////////////
		// STEP 2: Check validity of individual polygons
		//  it's more efficient doing this during creation, but this is more readable and maintainable (check SVN v59).
		//  After all, this is not the main focus here.
		std::vector<Polygon> polygonsVector;

		// CHECKS ON VERTICES
		// Remove repeated vertices. One per ring is considered normal, since some specifications allow or require it (first == last).
		for (unsigned int currentRing = 0; currentRing < outerRingsList.size(); ++currentRing) {
			if (removeDuplicateVertices(outerRingsList[currentRing]) > 1) {
				//std::cout << "\tFeature #" << feature->GetFID() << ": duplicate vertices in outer boundary #" << currentRing << ". Removed duplicates." << std::endl;
			}
		} 

		// Trivial check for rings with less than 3 vertices. The ones with 3 or more vertices will be done in the triangulation
		for (int currentRing = 0; currentRing < (int)outerRingsList.size(); ++currentRing) {
			if (outerRingsList[currentRing].size() < 3) {
				//std::cout << "\tFeature #" << feature->GetFID() << ": less than 3 vertices in outer boundary #" << currentRing << ". Removed." << std::endl;
				outerRingsList.erase(outerRingsList.begin()+currentRing);
				--currentRing;
			}
		}

		// CHECKS ON RINGS

		// Let's move on to the CGAL data structures for Rings
		std::vector<Ring> outerRings;
		std::vector<Ring> innerRings;
		outerRings.reserve(outerRingsList.size());
		innerRings.reserve(innerRingsList.size());
		for (unsigned int currentRing = 0; currentRing < outerRingsList.size(); currentRing++) {
			outerRings.push_back(Ring(outerRingsList[currentRing].begin(), outerRingsList[currentRing].end()));
			outerRingsList[currentRing].clear();
		} for (unsigned int currentRing = 0; currentRing < innerRingsList.size(); currentRing++) {
			innerRings.push_back(Ring(innerRingsList[currentRing].begin(), innerRingsList[currentRing].end()));
			innerRingsList[currentRing].clear();
		}

		// Split self touching rings and correct winding
		std::vector<Ring *> outerRingsToBuild;
		std::vector<Ring *> innerRingsToClassify;
		std::vector<std::vector<Ring> > innerRingsToBuild;

		// Get outer rings
		for (unsigned int currentRings = 0; currentRings < outerRings.size(); currentRings++) {
			if (!outerRings[currentRings].is_simple()) {
				//std::cout << "\tFeature #" << feature->GetFID() << " (" << outerRings[currentRings].size() << " vertices): self intersecting outer boundary #" << currentRings << ". Split." << std::endl;
				std::vector<Ring *> receivedRings = splitRing(outerRings[currentRings]);
				for (std::vector<Ring *>::iterator currentRing = receivedRings.begin(); currentRing != receivedRings.end(); ++currentRing) {
					if ((*currentRing)->is_clockwise_oriented()) {
						outerRingsToBuild.push_back(*currentRing);
					} else {
						innerRingsToClassify.push_back(*currentRing);
					}
				}
			} else {
				if (outerRings[currentRings].is_counterclockwise_oriented()) {
					//std::cout << "\tFeature #" << feature->GetFID() << ": incorrect winding in outer boundary #" << currentRings << ". Reversed." << std::endl;
					outerRings[currentRings].reverse_orientation();
				} outerRingsToBuild.push_back(new Ring(outerRings[currentRings]));
				outerRings[currentRings].clear();
			} 

			/*if (outerRings[currentRings].is_counterclockwise_oriented()) {
				//std::cout << "\tFeature #" << feature->GetFID() << ": incorrect winding in outer boundary #" << currentRings << ". Reversed." << std::endl;
				outerRings[currentRings].reverse_orientation();
			} outerRingsToBuild.push_back(new Ring(outerRings[currentRings]));
			outerRings[currentRings].clear();*/
		}

		// Get inner rings
		for (unsigned int currentRings = 0; currentRings < innerRings.size(); currentRings++) {
			if (!innerRings[currentRings].is_simple()) {
				//std::cout << "\tFeature #" << feature->GetFID() << " (" << innerRings[currentRings].size() << " vertices): self intersecting inner boundary #" << currentRings << ". Split." << std::endl;
				std::vector<Ring *> receivedRings = splitRing(innerRings[currentRings]);
				for (std::vector<Ring *>::iterator currentRing = receivedRings.begin(); currentRing != receivedRings.end(); ++currentRing) {
					if ((*currentRing)->is_clockwise_oriented()) {
						innerRingsToClassify.push_back(*currentRing);
					} else {
						outerRingsToBuild.push_back(*currentRing);
					}
				}
			} else {
				if (innerRings[currentRings].is_clockwise_oriented()) {
					//	 std::cout << "\tFeature #" << feature->GetFID() << ": incorrect winding in inner boundary #" << currentRings << ". Reversed." << std::endl;
					innerRings[currentRings].reverse_orientation();
				} innerRingsToClassify.push_back(new Ring(innerRings[currentRings]));
				innerRings[currentRings].clear();
			}
		}

		// Make space for inner rings
		for (std::vector<Ring *>::iterator currentRing = outerRingsToBuild.begin(); currentRing != outerRingsToBuild.end(); ++currentRing)
			innerRingsToBuild.push_back(std::vector<Ring>());

		// Put inner rings into the correct outer ring (and likely other ones). 
		//Incorrectly nested rings are found here.
		if (outerRingsToBuild.empty()) {
			// Outer ring had no area or there wasn't any. Delete all inner rings
			// std::cout << "\tFeature #" << feature->GetFID() << ": zero area outer boundary. Inner boundaries removed." << std::endl;
			for (std::vector<Ring *>::iterator currentRing = innerRingsToClassify.begin(); currentRing != innerRingsToClassify.end(); ++currentRing) {
				delete *currentRing;
			}
		} 
		else if (innerRingsToClassify.size() > 0) {
			// Now check them and put them in place
			testRings(outerRingsToBuild, innerRingsToClassify, innerRingsToBuild, 0);
		}

		// Let's move on to CGAL data structures for Polygons
		for (unsigned int currentPolygon = 0; currentPolygon < outerRingsToBuild.size(); ++currentPolygon)
			polygonsVector.push_back(Polygon(*outerRingsToBuild[currentPolygon], innerRingsToBuild[currentPolygon].begin(), innerRingsToBuild[currentPolygon].end())); 
		outerRingsToBuild.clear();
		innerRingsToBuild.clear();

		// STEP 3: Introduce edges as constraints in the triangulation
		for (std::vector<Polygon>::iterator currentPolygon = polygonsVector.begin(); currentPolygon != polygonsVector.end(); ++currentPolygon) 
		{
			// Create and save polygon handle
			PolygonHandle *handle = new PolygonHandle(schemaIndex, NULL, 0, currentPolygon-polygonsVector.begin());
			polygons.push_back(handle);

			// Save other attributes to put back later
			//copyFields(feature, handle);
			AddField(handle, compValue);

			// Create edges vector for this handle
			edgesToTag.push_back(std::pair<std::vector<Triangulation::Vertex_handle>, std::vector<std::vector<Triangulation::Vertex_handle> > >());

			// Insert edges into the triangulation and edges vector
			for (Ring::Edge_const_iterator currentEdge = currentPolygon->outer_boundary().edges_begin(); 
				currentEdge != currentPolygon->outer_boundary().edges_end(); 
				++currentEdge) {
					Triangulation::Vertex_handle sourceVertex = triangulation.insert(currentEdge->source(), startingSearchFace);
					startingSearchFace = triangulation.incident_faces(sourceVertex);
					Triangulation::Vertex_handle targetVertex = triangulation.insert(currentEdge->target(), startingSearchFace);
					triangulation.insert_constraint(sourceVertex, targetVertex);
					startingSearchFace = triangulation.incident_faces(targetVertex);
					edgesToTag.back().first.push_back(sourceVertex);
#ifdef _DEBUG
					if(DebugEdge(sourceVertex->point(), targetVertex->point(),154904.137, 461820.490, 154900.849, 461824.665)) {
						int aaa = 0;
					}
#endif
			} 
			for (Polygon::Hole_const_iterator currentRing = currentPolygon->holes_begin(); currentRing != currentPolygon->holes_end(); ++currentRing) {
				edgesToTag.back().second.push_back(std::vector<Triangulation::Vertex_handle>());
				for (Ring::Edge_const_iterator currentEdge = currentRing->edges_begin(); currentEdge != currentRing->edges_end(); ++currentEdge) {
					Triangulation::Vertex_handle sourceVertex = triangulation.insert(currentEdge->source(), startingSearchFace);
					startingSearchFace = triangulation.incident_faces(sourceVertex);
					Triangulation::Vertex_handle targetVertex = triangulation.insert(currentEdge->target(), startingSearchFace);
					triangulation.insert_constraint(sourceVertex, targetVertex);
					startingSearchFace = triangulation.incident_faces(targetVertex);
					edgesToTag.back().second.back().push_back(sourceVertex);
				}
			}
		}

		// Free memory
		polygonsVector.clear();
	}

	/*
	//////////////////////////////////////////////////////////////////////////
	//refine constraints
	Triangulation::Vertex_handle v1, v2, temV1, temV2;
	Triangulation::Vertices_in_constraint previousVertex, currentVertex;
	std::stack<Triangulation::Vertex_handle> stackVerts;
	Triangulation::Segment segment;
	for (unsigned int iPolygon = 0; iPolygon < edgesToTag.size(); ++iPolygon) {
		for (unsigned int iEdge = 0; iEdge < edgesToTag[iPolygon].first.size(); ++iEdge) {
			v1 = edgesToTag[iPolygon].first[iEdge];
			v2 = edgesToTag[iPolygon].first[(iEdge+1)%edgesToTag[iPolygon].first.size()];
			stackVerts.push(v1);
			stackVerts.push(v2);
			segment = Triangulation::Segment(v1->point(), v2->point());
			double len = CGAL::to_double(segment.squared_length());
			if(segment.squared_length()<0.0000001) {
				bool bbb = segment.is_degenerate();
				continue;
			}
			
#ifdef _DEBUG
			double stax=CGAL::to_double(v1->point().x());	double stay=CGAL::to_double(v1->point().y());
			double endx=CGAL::to_double(v2->point().x());	double endy=CGAL::to_double(v2->point().y());
#endif

			while(!stackVerts.empty()) {
				temV1 = stackVerts.top();
				stackVerts.pop();
				Triangulation::Edge_circulator ec = triangulation.incident_edges(temV1);
				Triangulation::Edge_circulator done(ec);
				//check vertexes on neighboring edges
				do {
					if (triangulation.is_constrained(*ec)) continue;
					temV2 = ec->first->vertex(ec->first->cw(ec->second));
					if (temV2==temV1)
						temV2 = ec->first->vertex(ec->first->ccw(ec->second));
					if (triangulation.is_infinite(temV2)) continue;
					//bool isbbbValid = temV2->is_valid();
					//bool isbbbInfit = triangulation.is_infinite(temV2);
					double dist = CGAL::to_double(CGAL::squared_distance(segment, temV2->point()));
					if (CGAL::to_double(CGAL::squared_distance(segment, temV2->point()))>0.000001)
						continue;
#ifdef _DEBUG
					double v1x=CGAL::to_double(temV1->point().x());	double v1y=CGAL::to_double(temV1->point().y());
					double v2x=CGAL::to_double(temV2->point().x());	double v2y=CGAL::to_double(temV2->point().y());
					Triangulation::Edge temEdge = Triangulation::Edge(ec->first->neighbor(ec->second), ec->first->neighbor(ec->second)->index(ec->first));
					bool isConst = triangulation.is_constrained(temEdge);
#endif
					if ((temV1==v1&&temV2==v2)||(temV2==v1&&temV1==v2)) {
						int aaa = 0;
					}
					bool onLine = segment.has_on(temV2->point());
					triangulation.insert_constraint(temV1,temV2);
					stackVerts.push(temV2);
				} while (++ec!=done);
			}
		}
	}*/

	return true;
}

/*
#include <CGAL/Boolean_set_operations_2.h>
//Biao 2013/07/30
//use boundary polygon to precisely tag the triangulation
bool IOWorker::tagTriangulation(Triangulation &triangulation, TaggingVector &edgesToTag) 
{	
	//derive all boundary polygons
	std::vector<Ring> outBounds;
	std::vector<std::vector<Ring> > inBounds;
	Ring temRing;
	std::vector<Triangulation::Vertex_handle>::iterator itrV;
	for (unsigned int iPolygon=0; iPolygon<edgesToTag.size(); ++iPolygon) {
		std::vector<Triangulation::Vertex_handle>& vertexs = edgesToTag[iPolygon].first;
		temRing.clear();
		for (itrV=vertexs.begin(); itrV!=vertexs.end(); itrV++) 
			temRing.push_back((*itrV)->point());
		if (!temRing.is_counterclockwise_oriented()) 
			temRing.reverse_orientation();		
		outBounds.push_back(temRing);

		std::vector<Ring> curInBounds;
		inBounds.push_back(curInBounds);
		for (unsigned int jInPol=0; jInPol!=edgesToTag[iPolygon].second.size(); jInPol++) {
			std::vector<Triangulation::Vertex_handle>& vertexs = (edgesToTag[iPolygon].second)[jInPol];
			temRing.clear();
			for (itrV=vertexs.begin(); itrV!=vertexs.end(); itrV++) 
				temRing.push_back((*itrV)->point());
			if (!temRing.is_clockwise_oriented()) temRing.reverse_orientation();
			outBounds.push_back(temRing);
		}
	}

	//check triangulations
	Triangulation::Vertex_handle v0, v1, v2;
	Point mid01, mid12, mid02;
	CGAL::Bounded_side b0, b1, b2, b01, b02, b12, bFace;
	for (Triangulation::Finite_faces_iterator f=triangulation.finite_faces_begin(); f!=triangulation.finite_faces_end(); ++f) {
		v0 = f->vertex(0);	v1 = f->vertex(1); v2 = f->vertex(2);
		mid01 = CGAL::midpoint(v0->point(), v1->point());
		mid12 = CGAL::midpoint(v1->point(), v2->point());
		mid02 = CGAL::midpoint(v0->point(), v2->point());
		Triangle triangle  = Triangle(v0->point(), v1->point(), v2->point());
		CGAL::Bounded_side bTri01 = triangle.bounded_side(mid01);
		CGAL::Bounded_side bTri12 = triangle.bounded_side(mid12);
		CGAL::Bounded_side bTri02 = triangle.bounded_side(mid02);

#ifdef _DEBUG
		std::vector<Point> debugVerts;
		debugVerts.push_back(Point(154981.70, 461729.29));
		debugVerts.push_back(Point(154979.82, 461731.76));
		debugVerts.push_back(Point(154988.61, 461734.50));

		std::vector<bool> vecSamePos;
		bool bSame;
		for (int i=0; i<3; i++) {
			bSame = false;
			for (int j=0; j<3; j++) {
				double dist =CGAL::to_double(CGAL::squared_distance(debugVerts[i], f->vertex(j)->point()));
				if (dist<0.01)	{
					bSame = true; 
					break;
				}
			}
			vecSamePos.push_back(bSame);
		}
		if (vecSamePos[0]&&vecSamePos[1]&&vecSamePos[2]) {
			int aaa = 0;
		}
		
#endif

		if (triangle.is_degenerate()) continue;
		
		//Ring curTri;		
		//curTri.push_back(v0->point());
		//curTri.push_back(v1->point());
		//curTri.push_back(v2->point());
		//if(curTri.is_collinear_oriented()) 
		//	continue;
		//if (curTri.is_clockwise_oriented()) 
		//	curTri.reverse_orientation();
		
		for (int iPol=0; iPol<outBounds.size(); iPol++) {
			Ring& bound = outBounds[iPol];
			//if (CGAL::do_intersect(curTri, bound))	{
			//	f->info().addTag(polygons[iPol]);
			//}
			
			std::vector<Triangulation::Vertex_handle>& vertexs = edgesToTag[iPol].first;
			b0 = bound.bounded_side(v0->point());
			b1 = bound.bounded_side(v1->point());
			b2 = bound.bounded_side(v2->point());
			
			bFace = CGAL::ON_BOUNDARY;

			if (b0==CGAL::ON_UNBOUNDED_SIDE||b1==CGAL::ON_UNBOUNDED_SIDE||b2==CGAL::ON_UNBOUNDED_SIDE)
				bFace = CGAL::ON_UNBOUNDED_SIDE;
			else if(b0==CGAL::ON_BOUNDARY&&b1==CGAL::ON_BOUNDARY&&b2==CGAL::ON_BOUNDARY) {
				//check mid point and area
				//Triangle triangle  = Triangle(v0->point(), v1->point(), v2->point());
				if(triangle.is_degenerate()) 
					bFace = CGAL::ON_BOUNDED_SIDE;
				else{
					b01 = bound.bounded_side(mid01);
					b12 = bound.bounded_side(mid12);
					b02 = bound.bounded_side(mid02);
					if(b01==CGAL::ON_UNBOUNDED_SIDE||b12==CGAL::ON_UNBOUNDED_SIDE||b02==CGAL::ON_UNBOUNDED_SIDE)
						bFace = CGAL::ON_UNBOUNDED_SIDE;
				}
				//bFace = CGAL::ON_UNBOUNDED_SIDE;
			}
			
			if (bFace == CGAL::ON_BOUNDARY)
				f->info().addTag(polygons[iPol]);
		}
	}

	// Free remaining memory
	edgesToTag.clear();

	// Tag the universe
	Triangulation::Face_handle currentFace = triangulation.infinite_face();
	std::stack<Triangulation::Face_handle> stack;
	stack.push(currentFace);
	tagStack(stack, &universe);

	return true;
}*/

/*
bool IOWorker::tagTriangulation(Triangulation &triangulation, TaggingVector &edgesToTag) 
{	
	std::stack<Triangulation::Face_handle> stack;
	Triangulation::Vertices_in_constraint_iterator previousVertex, currentVertex;
	Triangulation::Face_handle currentFace;
	int incident;
	bool sameOrder;
	
	// Add all edges of a polygon
	for (unsigned int iPolygon = 0; iPolygon < edgesToTag.size(); ++iPolygon) {
		// Outer boundary
		for (unsigned int iEdge = 0; iEdge < edgesToTag[iPolygon].first.size(); ++iEdge) {
			previousVertex = triangulation.vertices_in_constraint_begin(
				edgesToTag[iPolygon].first[iEdge],
				edgesToTag[iPolygon].first[(iEdge+1)%edgesToTag[iPolygon].first.size()]);
			
			// Check if the returned order is the same
			sameOrder = (*previousVertex)->point()==edgesToTag[iPolygon].first[iEdge]->point();
			currentVertex = previousVertex;
			++currentVertex;
			while (currentVertex != triangulation.vertices_in_constraint_end(
				edgesToTag[iPolygon].first[iEdge], 
				edgesToTag[iPolygon].first[(iEdge+1)%edgesToTag[iPolygon].first.size()])) 
			{
				if (sameOrder) {
					if (!triangulation.is_edge(*previousVertex, *currentVertex, currentFace, incident)) {
						std::cout << "\tError: Cannot find adjoining face to an edge from the edge list!" << std::endl;
						return false;
					}
				} 
				else {
					if (!triangulation.is_edge(*currentVertex, *previousVertex, currentFace, incident)) {
						std::cout << "\tError: Cannot find adjoining face to an edge from the edge list!" << std::endl;
						return false;
					}
				} 
				
				previousVertex = currentVertex;
				currentVertex++;
				stack.push(currentFace);
			}
		}
		
		// Free memory for boundaries
		edgesToTag[iPolygon].first.clear();
		edgesToTag[iPolygon].second.clear();
		
		// Expand the tags
		tagStack(stack, polygons[iPolygon]);
		#ifdef _DEBUG
		exportTriangulation(triangulation, "debug_triangle.dxf", true, false, false);
		#endif	
	}
	
	// Free remaining memory
	edgesToTag.clear();
	
	// Tag the universe
	currentFace = triangulation.infinite_face();
	stack.push(currentFace);
	tagStack(stack, &universe);
	
	return true;
}


bool IOWorker::tagTriangulation03(Triangulation &triangulation, TaggingVector &edgesToTag) 
{	
	typedef std::pair<Triangulation::Vertex_handle, Triangulation::Vertex_handle> Edge;
	std::set<Edge> setBorders;

	std::stack<Triangulation::Face_handle> stack;
	Triangulation::Vertices_in_constraint_iterator previousVertex, currentVertex;
	Triangulation::Vertex_handle v1, v2;
	Triangulation::Face_handle curFace, neibFace;
	int incident;
	bool sameOrder, bOutSide;

	// Add all edges of a polygon
	for (unsigned int iPolygon = 0; iPolygon < edgesToTag.size(); ++iPolygon) 
	{
		//collect all constraint edges on current polygon
		// Outer boundary
		setBorders.clear();
		for (unsigned int iEdge = 0; iEdge < edgesToTag[iPolygon].first.size(); ++iEdge) {
			v1 = edgesToTag[iPolygon].first[iEdge];
			v2 = edgesToTag[iPolygon].first[(iEdge+1)%edgesToTag[iPolygon].first.size()];
			
			currentVertex = triangulation.vertices_in_constraint_begin(v1, v2);

			// Check if the returned order is the same
			sameOrder = (*currentVertex)->point()==edgesToTag[iPolygon].first[iEdge]->point();
			previousVertex = currentVertex;
			++currentVertex;
			while (currentVertex != triangulation.vertices_in_constraint_end(v1, v2)) {
				if (sameOrder)
					setBorders.insert(Edge(*previousVertex, *currentVertex));
				else 
					setBorders.insert(Edge(*currentVertex, *previousVertex));

				previousVertex = currentVertex;
				currentVertex++;
			}
		}

		// Free memory for boundaries
		edgesToTag[iPolygon].first.clear();
		edgesToTag[iPolygon].second.clear();

		//////////////////////////////////////////////////////////////////////////
		// Expand the tags
		//tagStack(stack, polygons[iPolygon]);
		if(!triangulation.is_edge(setBorders.begin()->first, setBorders.begin()->second, curFace, incident))
			continue;

		stack.push(curFace);
		while(!stack.empty()) {
			curFace = stack.top();
			stack.pop();
			if(!curFace->info().hasTag(polygons[iPolygon]))
				curFace->info().addTag(polygons[iPolygon]);

			//check all neighbor faces
			for (int i=0; i<3; i++)	{
				neibFace = curFace->neighbor(i);
				if(neibFace->info().hasTag(polygons[iPolygon])) continue;

				bOutSide = false;
				//the face which has edge v2->v1 is outside the polygon
				for (int j=0; j<3; j++) {
					v1 = neibFace->vertex(j);
					v2 = neibFace->vertex((j+1)%3);
					if (setBorders.count(Edge(v2, v1)))	{//inverse direction of edge
						bOutSide = true;
						break;
					}
				}

				if (bOutSide) continue;
				stack.push(neibFace);
				neibFace->info().addTag(polygons[iPolygon]);
			}
		}

#ifdef _DEBUG
		exportTriangulation(triangulation, "debug_triangle.dxf", true, false, false);
#endif		
	}

	// Free remaining memory
	edgesToTag.clear();

	// Tag the universe
	curFace = triangulation.infinite_face();
	stack.push(curFace);
	tagStack(stack, &universe);

	return true;
}*/

//04
bool IOWorker::tagTriangulation(Triangulation &triangulation, TaggingVector &edgesToTag) 
{	
	typedef std::pair<Triangulation::Vertex_handle, Triangulation::Vertex_handle> Edge;
	std::set<Edge> setBorders;

	std::stack<Triangulation::Face_handle> stack;
	Triangulation::Vertices_in_constraint_iterator previousVertex, currentVertex;
	Triangulation::Face_handle currentFace, neibFace;
	int incident;
	bool sameOrder, bOutSide;
	Triangulation::Vertex_handle v1,v2;

	//////////////////////////////////////////////////////////////////////////
	// Tag the universe
	currentFace = triangulation.infinite_face();
	stack.push(currentFace);
	tagStack(stack, &universe);

	//////////////////////////////////////////////////////////////////////////
	// Add all edges of a polygon
	for (unsigned int iPolygon = 0; iPolygon < edgesToTag.size(); ++iPolygon) {
		// Outer boundary
		setBorders.clear();
		for (unsigned int iEdge = 0; iEdge < edgesToTag[iPolygon].first.size(); ++iEdge) {
			v1 = edgesToTag[iPolygon].first[iEdge];
			v2 = edgesToTag[iPolygon].first[(iEdge+1)%edgesToTag[iPolygon].first.size()];
			previousVertex = triangulation.vertices_in_constraint_begin(v1, v2);
			//if(DebugEdge(v1->point(), v2->point(), 154870.26,461813.58, 154870.76,461810.98))
			//	int aaa = 0;

			// Check if the returned order is the same
			sameOrder = (*previousVertex)->point()==v1->point();
			currentVertex = previousVertex;
			++currentVertex;
			while (currentVertex != triangulation.vertices_in_constraint_end(v1, v2)) 
			{
				if (sameOrder) {
					setBorders.insert(Edge(*previousVertex, *currentVertex));
					if (!triangulation.is_edge(*previousVertex, *currentVertex, currentFace, incident)) {
						std::cout << "\tError: Cannot find adjoining face to an edge from the edge list!" << std::endl;
						return false;
					}
				}
				else {
					setBorders.insert(Edge(*currentVertex, *previousVertex));
					if (!triangulation.is_edge(*currentVertex, *previousVertex, currentFace, incident)) {
						std::cout << "\tError: Cannot find adjoining face to an edge from the edge list!" << std::endl;
						return false;
					}
				}

				previousVertex = currentVertex;
				currentVertex++;
				stack.push(currentFace);
			}
		}

		// Free memory for boundaries
		edgesToTag[iPolygon].first.clear();
		edgesToTag[iPolygon].second.clear();

		// Expand the tags
		while(!stack.empty()) {
			currentFace = stack.top();
			stack.pop();
			//if (IsDegenerateFace(triangulation, currentFace)) {
				//currentFace->info().addTag(&universe);
			//	continue;
			//}

			if(!currentFace->info().hasTag(polygons[iPolygon]))
				currentFace->info().addTag(polygons[iPolygon]);

			//check all neighbor faces
			for (int i=0; i<3; i++)	{
				neibFace = currentFace->neighbor(i);
				if(neibFace->info().hasTag(polygons[iPolygon])) continue;
				if(neibFace->info().hasTag(&universe)) {
					int aaa = 0 ;
					continue;
				}

				//if (IsDegenerateFace(triangulation, neibFace)) {
					//neibFace->info().addTag(&universe);
				//	continue;
				//}

/*#ifdef _DEBUG
				float curV0x = CGAL::to_double(currentFace->vertex(0)->point().x()); float curV0y = CGAL::to_double(currentFace->vertex(0)->point().y());
				float curV1x = CGAL::to_double(currentFace->vertex(1)->point().x()); float curV1y = CGAL::to_double(currentFace->vertex(1)->point().y());
				float curV2x = CGAL::to_double(currentFace->vertex(2)->point().x()); float curV2y = CGAL::to_double(currentFace->vertex(2)->point().y());
				float neiV0x = CGAL::to_double(neibFace->vertex(0)->point().x()); float neiV0y = CGAL::to_double(neibFace->vertex(0)->point().y());
				float neiV1x = CGAL::to_double(neibFace->vertex(1)->point().x()); float neiV1y = CGAL::to_double(neibFace->vertex(1)->point().y());
				float neiV2x = CGAL::to_double(neibFace->vertex(2)->point().x()); float neiV2y = CGAL::to_double(neibFace->vertex(2)->point().y());
				if(DebugTriangle(currentFace, 154967.28, 461741.12,154971.21, 461728.93, 154968.74, 461736.60)) {
					int aaa = 0;
					IsDegenerateFace(triangulation, currentFace);
					Triangle curTri(currentFace->vertex(0)->point(), currentFace->vertex(1)->point(), currentFace->vertex(2)->point());
					double curArea = CGAL::to_double(curTri.area());
					bool curDeg = curTri.is_degenerate();
				}
				if(DebugTriangle(neibFace, 154967.28, 461741.12,154971.21, 461728.93, 154968.74, 461736.60)) {
					int aaa = 0;
					IsDegenerateFace(triangulation, neibFace);
					Triangle neiTri(neibFace->vertex(0)->point(), neibFace->vertex(1)->point(), neibFace->vertex(2)->point());
					bool neiDeg = neiTri.is_degenerate();
					double neiArea = CGAL::to_double(neiTri.area());				
				}
				
#endif*/

				bOutSide = false;
				//the face which has edge v2->v1 is outside the polygon
				for (int j=0; j<3; j++) {
					if (setBorders.count(Edge(neibFace->vertex(j), neibFace->vertex((j+1)%3))))	{//inverse direction of edge
						bOutSide = true;
						break;
					}
				}

				if (bOutSide) continue;

				stack.push(neibFace);
				neibFace->info().addTag(polygons[iPolygon]);
			}
		}
/*#ifdef _DEBUG
		exportTriangulation(triangulation, "debug_triangle.dxf", true, false, false);
#endif	*/
	}

	// Free remaining memory
	edgesToTag.clear();

	//////////////////////////////////////////////////////////////////////////
	//check degenerate face
	double len0, len1, len2;
	PolygonHandle *tag0, *tag1, *tag2;
	int nDegFaces = 0;
	for(Triangulation::Finite_faces_iterator f =triangulation.finite_faces_begin(); f!=triangulation.finite_faces_end(); f++) {
#ifdef _DEBUG
		if(DebugTriangle(f, 154995.57,461756.44, 154995.03,461756.66, 154995.57,461756.44)
			||DebugTriangle(f, 154995.57,461756.44, 154995.03,461756.66, 154995.03,461756.66))
			int aaa = 0;
#endif
		
		if (f->info().hasNoTags() && IsDegenerateFace(triangulation, f)) {
			nDegFaces++;
			len0 = CGAL::to_double(CGAL::squared_distance(f->vertex(1)->point(), f->vertex(2)->point()));
			len1 = CGAL::to_double(CGAL::squared_distance(f->vertex(0)->point(), f->vertex(2)->point()));
			len2 = CGAL::to_double(CGAL::squared_distance(f->vertex(0)->point(), f->vertex(1)->point()));
			tag0 = f->neighbor(0)->info().getTags();
			tag1 = f->neighbor(1)->info().getTags();
			tag2 = f->neighbor(2)->info().getTags();
			if (!tag0) len0 = 0.0;
			if (!tag1) len1 = 0.0;
			if (!tag2) len2 = 0.0;

			//make face 1 equal to face 0, if possible
			if (tag0==tag2 ) {
				std::swap(tag1, tag2);
				std::swap(len1, len2);
			}
			else if(tag1==tag2) {
				std::swap(tag0, tag2);
				std::swap(len0, len2);
			}

			if (tag0==tag1 &&  tag0) 
				f->info().addTag(tag0);
			//else if (tag0==&universe || tag1==&universe || tag2==&universe)
			//	f->info().addTag(&universe);
			else if (tag0&&!tag1&&!tag2)
				f->info().addTag(tag0);
			else if (tag1&&!tag0&&!tag2)
				f->info().addTag(tag1);
			else if (tag2&&!tag0&&!tag1)
				f->info().addTag(tag2);
			else 
			{
				if (len0>=len1 && len0>=len2) {
					f->info().addTag(tag0);
				}
				else if (len1>=len0 && len1>=len2) {
					f->info().addTag(tag1);
				}
				else if (len2>=len0 && len2>=len1) {
					f->info().addTag(tag2);
				}
			}
		}
	}

	return true;
}

bool IOWorker::IsDegenerateFace(Triangulation &triangulation, Triangulation::Face_handle face)
{
	Triangulation::Vertex_handle v0 = face->vertex(0);
	Triangulation::Vertex_handle v1 = face->vertex(1);
	Triangulation::Vertex_handle v2 = face->vertex(2);
	cgalTriangle triagle(v0->point(), v1->point(), v2->point());
	double area = CGAL::to_double(triagle.area());
	return area < 0.000001;

/*	for (int i=0; i<3; i++)	{
		Triangulation::Vertex_handle v0 = face->vertex(i);
		Triangulation::Vertex_handle v1 = face->vertex(face->cw(i));
		Triangulation::Vertex_handle v2 = face->vertex(face->ccw(i));
		if(!triangulation.is_constrained(std::pair<Triangulation::Face_handle, int>(face, i))) continue;

		for (Triangulation::Vertices_in_constraint temV=triangulation.vertices_in_constraint_begin(v2,v1);
			temV!=triangulation.vertices_in_constraint_end(v2, v1); 
			temV++)	{
				if (v2->point()==(*temV)->point()) {
					return true;
				}
		}
	}*/

	return false;
}

bool IOWorker::makeAllHolesValid(Triangulation &triangulation) 
{
	for (Triangulation::Finite_faces_iterator currentFace = triangulation.finite_faces_begin(); currentFace != triangulation.finite_faces_end(); ++currentFace) {
		if (currentFace->info().hasNoTags()) {
			currentFace->info().addTag(&universe);
		}
	}
	
	return true;
}

bool IOWorker::splitRegions(Triangulation &triangulation, double ratio) 
{
	double shortSide, longSide, thisSide;
	unsigned int whichSide, splits = 0;
	
	for (Triangulation::Finite_faces_iterator currentFace = triangulation.finite_faces_begin(); currentFace != triangulation.finite_faces_end(); ++currentFace) 
	{		
		// Check for the longest and shortest sides
		shortSide = longSide = sqrt(CGAL::to_double(triangulation.segment(currentFace, 0).squared_length()));
		whichSide = 0;
		thisSide = sqrt(CGAL::to_double(triangulation.segment(currentFace, 1).squared_length()));
	
		if (thisSide > longSide) longSide = thisSide;
		else if (thisSide < shortSide) {
			shortSide = thisSide;
			whichSide = 1;
		} thisSide = sqrt(CGAL::to_double(triangulation.segment(currentFace, 2).squared_length()));
	
		if (thisSide > longSide) longSide = thisSide;
		else if (thisSide < shortSide) {
			shortSide = thisSide;
			whichSide = 2;
		}
		
		// Add constrained edge if they exceed the long/short ratio
		if (longSide/shortSide >= ratio) {
			currentFace->set_constraint(whichSide, true);
			++splits;
		}
	}
	
	std::cout << "\t" << splits << " constrained edges added." << std::endl;
	
	return true;
}

bool IOWorker::repairTrianglesByNumberOfNeighbours(Triangulation &triangulation, bool alsoUniverse) 
{
	bool repaired = true;
	
	// Use a temporary vector to make it deterministic and order independent
	std::vector<std::pair<Triangulation::Face_handle, PolygonHandle *> > facesToRepair;
	for (Triangulation::Finite_faces_iterator currentFace = triangulation.finite_faces_begin(); currentFace != triangulation.finite_faces_end(); ++currentFace) {
		std::map<PolygonHandle *, unsigned int> tagCount;
		if (!currentFace->info().hasOneTag()) {
			
			// Count the number of times each tag appears
			addtoCount(tagCount, currentFace->neighbor(0)->info().getTags());
			addtoCount(tagCount, currentFace->neighbor(1)->info().getTags());
			addtoCount(tagCount, currentFace->neighbor(2)->info().getTags());
            
			// Find the tag with highest count
			unsigned int maxCount = 0;
			std::map<PolygonHandle *, unsigned int>::iterator mostTimesAppeared = tagCount.end();
			for (std::map<PolygonHandle *, unsigned int>::iterator currentCount = tagCount.begin(); currentCount != tagCount.end(); ++currentCount) {
				if (currentCount->first != NULL && (alsoUniverse || currentCount->first != &universe)) {
					if (currentCount->second > maxCount && (currentFace->info().hasTag(currentCount->first) || currentFace->info().hasNoTags())) {
						currentCount->second = maxCount;
						mostTimesAppeared = currentCount;
					} else if (currentCount->second == maxCount) {
						mostTimesAppeared = tagCount.end();
					}
				}
			}
			
			// Assign the triangle to the tag with the highest count (if there is one)
			if (mostTimesAppeared == tagCount.end()) repaired = false;
			else facesToRepair.push_back(std::pair<Triangulation::Face_handle, PolygonHandle *>(currentFace, mostTimesAppeared->first));
		}
	}
	
	// Re-tag faces in the vector
	for (std::vector<std::pair<Triangulation::Face_handle, PolygonHandle *> >::iterator currentFace = facesToRepair.begin(); currentFace != facesToRepair.end(); ++currentFace) {
		currentFace->first->info().removeAllTags();
		currentFace->first->info().addTag(currentFace->second);
	}
	
	return repaired;
}

bool IOWorker::repairTrianglesByAbsoluteMajority(Triangulation &triangulation, bool alsoUniverse) 
{
	bool repaired = true;
	
	// Put faces to repair in the vector
	// Use a temporary vector to make it deterministic and order independent
	std::vector<std::pair<Triangulation::Face_handle, Triangulation::Face_handle> > facesToRepair;
	for (Triangulation::Finite_faces_iterator currentFace = triangulation.finite_faces_begin(); currentFace != triangulation.finite_faces_end(); ++currentFace) 
	{
		if (currentFace->info().hasOneTag()) continue;

		if (currentFace->neighbor(0)->info().hasOneTag() && currentFace->neighbor(1)->info().hasOneTag() &&
			currentFace->neighbor(0)->info().getTags() == currentFace->neighbor(1)->info().getTags() && 
			(currentFace->info().hasTag(currentFace->neighbor(0)->info().getTags()) || currentFace->info().hasNoTags())) {
				if ((currentFace->neighbor(0)->info().getTags() != &universe &&
					currentFace->neighbor(0)->info().getTags() != NULL) || 
					alsoUniverse) {
						facesToRepair.push_back(std::pair<Triangulation::Face_handle, Triangulation::Face_handle>(currentFace, currentFace->neighbor(0)));
				}
		} 
		else if (currentFace->neighbor(0)->info().hasOneTag() && currentFace->neighbor(2)->info().hasOneTag() &&
			currentFace->neighbor(0)->info().getTags() == currentFace->neighbor(2)->info().getTags() &&
			(currentFace->info().hasTag(currentFace->neighbor(2)->info().getTags()) || currentFace->info().hasNoTags())) {
				if ((currentFace->neighbor(2)->info().getTags() != &universe &&
					currentFace->neighbor(2)->info().getTags() != NULL) || 
					alsoUniverse) {
						facesToRepair.push_back(std::pair<Triangulation::Face_handle, Triangulation::Face_handle>(currentFace, currentFace->neighbor(2)));
				}
		} 
		else if (currentFace->neighbor(1)->info().hasOneTag() && currentFace->neighbor(2)->info().hasOneTag() &&
			currentFace->neighbor(1)->info().getTags() == currentFace->neighbor(2)->info().getTags() &&
			(currentFace->info().hasTag(currentFace->neighbor(1)->info().getTags()) || currentFace->info().hasNoTags())) {
				if ((currentFace->neighbor(1)->info().getTags() != &universe &&
					currentFace->neighbor(1)->info().getTags() != NULL) || 
					alsoUniverse) {
						facesToRepair.push_back(std::pair<Triangulation::Face_handle, Triangulation::Face_handle>(currentFace, currentFace->neighbor(1)));
				}
		} 
		else {
			repaired = false;
		}
	}
	
	// Re-tag faces in the vector
	for (std::vector<std::pair<Triangulation::Face_handle, Triangulation::Face_handle> >::iterator currentFace = facesToRepair.begin();
		 currentFace != facesToRepair.end();
		 ++currentFace) {
		currentFace->first->info().removeAllTags();
		currentFace->first->info().addTag(currentFace->second->info().getTags());
	}
	
	return repaired;
}

bool IOWorker::repairTrianglesByLongestBoundary(Triangulation &triangulation, bool alsoUniverse) {
	
	bool repaired = true;
	
	// Use a temporary vector to make it deterministic and order independent
	std::vector<std::pair<Triangulation::Face_handle, PolygonHandle *> > facesToRepair;
	for (Triangulation::Finite_faces_iterator currentFace = triangulation.finite_faces_begin(); currentFace != triangulation.finite_faces_end(); ++currentFace) {
		std::map<PolygonHandle *, double> tagBoundaryLength;
		if (!currentFace->info().hasOneTag()) {
			
			// Add up the boundary for each tag
			addToLength(tagBoundaryLength, currentFace->neighbor(0)->info().getTags(), sqrt(CGAL::to_double(triangulation.segment(currentFace, 0).squared_length())));
			addToLength(tagBoundaryLength, currentFace->neighbor(1)->info().getTags(), sqrt(CGAL::to_double(triangulation.segment(currentFace, 1).squared_length())));
			addToLength(tagBoundaryLength, currentFace->neighbor(2)->info().getTags(), sqrt(CGAL::to_double(triangulation.segment(currentFace, 2).squared_length())));
            
			// Find the tag with longest boundary
			double maxLength = 0.0;
			std::map<PolygonHandle *, double>::iterator longest = tagBoundaryLength.end();
			for (std::map<PolygonHandle *, double>::iterator currentLength = tagBoundaryLength.begin(); currentLength != tagBoundaryLength.end(); ++currentLength) {
				if (currentLength->first != NULL && (alsoUniverse || currentLength->first != &universe)) {
					if (currentLength->second > maxLength && (currentFace->info().hasTag(currentLength->first) || currentFace->info().hasNoTags())) {
						maxLength = currentLength->second;
						longest = currentLength;
					} else if (currentLength->second == maxLength) {
						longest = tagBoundaryLength.end();
					}
				}
			}
			
			// Assign the triangle to the tag with the longest boundary (if there is one)
			if (longest == tagBoundaryLength.end()) repaired = false;
			else facesToRepair.push_back(std::pair<Triangulation::Face_handle, PolygonHandle *>(currentFace, longest->first));
		}
	}
	
	// Re-tag faces in the vector
	for (std::vector<std::pair<Triangulation::Face_handle, PolygonHandle *> >::iterator currentFace = facesToRepair.begin(); currentFace != facesToRepair.end(); ++currentFace) {
		currentFace->first->info().removeAllTags();
		currentFace->first->info().addTag(currentFace->second);
	}
	
	return repaired;
}

bool IOWorker::repairRegionsByLongestBoundary(Triangulation &triangulation, bool alsoUniverse) 
{
	bool repaired = true;
	
	// Use a temporary vector to make it deterministic and order independent
	std::vector<std::pair<Triangulation::Face_handle, PolygonHandle *> > facesToRepair;
	std::set<Triangulation::Face_handle> processedFaces;
	for (Triangulation::Finite_faces_iterator currentFace = triangulation.finite_faces_begin(); currentFace != triangulation.finite_faces_end(); ++currentFace) {
		std::map<PolygonHandle *, double> tagBoundaryLength;
		if (!currentFace->info().hasOneTag() && !processedFaces.count(currentFace)) {
			
			// Expand this triangle into a complete region
			std::set<Triangulation::Face_handle> facesInRegion;
			facesInRegion.insert(currentFace);
			std::stack<Triangulation::Face_handle> facesToProcess;
			facesToProcess.push(currentFace);
			while (facesToProcess.size() > 0) {
				Triangulation::Face_handle currentFaceInStack = facesToProcess.top();
				facesToProcess.pop();
				processedFaces.insert(currentFaceInStack);
				if (!currentFaceInStack->neighbor(0)->info().hasOneTag() && !facesInRegion.count(currentFaceInStack->neighbor(0)) && 
					!triangulation.is_constrained(std::pair<Triangulation::Face_handle, int>(currentFaceInStack, 0))) {
					facesInRegion.insert(currentFaceInStack->neighbor(0));
					facesToProcess.push(currentFaceInStack->neighbor(0));
				} if (!currentFaceInStack->neighbor(1)->info().hasOneTag() && !facesInRegion.count(currentFaceInStack->neighbor(1)) && 
                      !triangulation.is_constrained(std::pair<Triangulation::Face_handle, int>(currentFaceInStack, 1))) {
					facesInRegion.insert(currentFaceInStack->neighbor(1));
					facesToProcess.push(currentFaceInStack->neighbor(1));
				} if (!currentFaceInStack->neighbor(2)->info().hasOneTag() && !facesInRegion.count(currentFaceInStack->neighbor(2)) && 
					  !triangulation.is_constrained(std::pair<Triangulation::Face_handle, int>(currentFaceInStack, 2))) {
					facesInRegion.insert(currentFaceInStack->neighbor(2));
					facesToProcess.push(currentFaceInStack->neighbor(2));
				}
			}
			
			// Add up the boundary for each triangle and tag
			for (std::set<Triangulation::Face_handle>::iterator currentFaceInRegion = facesInRegion.begin(); currentFaceInRegion != facesInRegion.end(); ++currentFaceInRegion) {
				if (!facesInRegion.count((*currentFaceInRegion)->neighbor(0))) {
					addToLength(tagBoundaryLength, (*currentFaceInRegion)->neighbor(0)->info().getTags(), sqrt(CGAL::to_double(triangulation.segment(*currentFaceInRegion, 0).squared_length())));
				} if (!facesInRegion.count((*currentFaceInRegion)->neighbor(1))) {
					addToLength(tagBoundaryLength, (*currentFaceInRegion)->neighbor(1)->info().getTags(), sqrt(CGAL::to_double(triangulation.segment(*currentFaceInRegion, 1).squared_length())));
				} if (!facesInRegion.count((*currentFaceInRegion)->neighbor(2))) {
					addToLength(tagBoundaryLength, (*currentFaceInRegion)->neighbor(2)->info().getTags(), sqrt(CGAL::to_double(triangulation.segment(*currentFaceInRegion, 2).squared_length())));
				}
			}
			
			// Find the tag with longest boundary
			double maxLength = 0.0;
			std::map<PolygonHandle *, double>::iterator longest = tagBoundaryLength.end();
			for (std::map<PolygonHandle *, double>::iterator currentLength = tagBoundaryLength.begin(); currentLength != tagBoundaryLength.end(); ++currentLength) {
				if (currentLength->first != NULL && (alsoUniverse || currentLength->first != &universe)) {
					if (currentLength->second > maxLength && (currentFace->info().hasTag(currentLength->first) || currentFace->info().hasNoTags())) {
						maxLength = currentLength->second;
						longest = currentLength;
					} else if (currentLength->second == maxLength) {
						longest = tagBoundaryLength.end();
					}
				}
			}
			
			// Assign the region to the tag with the longest boundary (if there is one)
			if (longest == tagBoundaryLength.end()) repaired = false;
			else {
				for (std::set<Triangulation::Face_handle>::iterator currentFaceInRegion = facesInRegion.begin(); currentFaceInRegion != facesInRegion.end(); ++currentFaceInRegion) {
					facesToRepair.push_back(std::pair<Triangulation::Face_handle, PolygonHandle *>(*currentFaceInRegion, longest->first));
				}
			}
		}
	}
	
	// Re-tag faces in the vector
	for (std::vector<std::pair<Triangulation::Face_handle, PolygonHandle *> >::iterator currentFace = facesToRepair.begin(); currentFace != facesToRepair.end(); ++currentFace) {
		currentFace->first->info().removeAllTags();
		currentFace->first->info().addTag(currentFace->second);
	}
	
	return repaired;
}

bool IOWorker::repairRegionsByRandomNeighbour(Triangulation &triangulation, bool alsoUniverse) 
{
	bool repaired = true;
	
	// Use a temporary vector to make it deterministic and order independent
	std::vector<std::pair<Triangulation::Face_handle, PolygonHandle *> > facesToRepair;
	std::set<Triangulation::Face_handle> processedFaces;
	for (Triangulation::Finite_faces_iterator currentFace = triangulation.finite_faces_begin(); currentFace != triangulation.finite_faces_end(); ++currentFace) {
		if (!currentFace->info().hasOneTag() && !processedFaces.count(currentFace)) {
			
			// Expand this triangle into a complete region
			std::set<Triangulation::Face_handle> facesInRegion;
			facesInRegion.insert(currentFace);
			std::stack<Triangulation::Face_handle> facesToProcess;
			facesToProcess.push(currentFace);
			while (facesToProcess.size() > 0) {
				Triangulation::Face_handle currentFaceInStack = facesToProcess.top();
				facesToProcess.pop();
				processedFaces.insert(currentFaceInStack);
				if (!currentFaceInStack->neighbor(0)->info().hasOneTag() && !facesInRegion.count(currentFaceInStack->neighbor(0)) && 
					!triangulation.is_constrained(std::pair<Triangulation::Face_handle, int>(currentFaceInStack, 0))) {
					facesInRegion.insert(currentFaceInStack->neighbor(0));
					facesToProcess.push(currentFaceInStack->neighbor(0));
				} if (!currentFaceInStack->neighbor(1)->info().hasOneTag() && !facesInRegion.count(currentFaceInStack->neighbor(1)) && 
					  !triangulation.is_constrained(std::pair<Triangulation::Face_handle, int>(currentFaceInStack, 1))) {
					facesInRegion.insert(currentFaceInStack->neighbor(1));
					facesToProcess.push(currentFaceInStack->neighbor(1));
				} if (!currentFaceInStack->neighbor(2)->info().hasOneTag() && !facesInRegion.count(currentFaceInStack->neighbor(2)) && 
					  !triangulation.is_constrained(std::pair<Triangulation::Face_handle, int>(currentFaceInStack, 2))) {
					facesInRegion.insert(currentFaceInStack->neighbor(2));
					facesToProcess.push(currentFaceInStack->neighbor(2));
				}
			}
			
			// Find a random tag
			PolygonHandle *tagToAssign;
			while (true) {
				std::set<Triangulation::Face_handle>::iterator randomFace = facesInRegion.begin();
				std::advance(randomFace, rand()%facesInRegion.size());
				int neighbourIndex = rand()%3;
				unsigned int numberOfTags = (*randomFace)->neighbor(neighbourIndex)->info().numberOfTags();
				if (numberOfTags == 0) continue;
				if (numberOfTags == 1) {
					tagToAssign = (*randomFace)->neighbor(neighbourIndex)->info().getTags();
					if (alsoUniverse || tagToAssign != &universe) break;
				} else {
					std::list<PolygonHandle *>::const_iterator randomTag = static_cast<MultiPolygonHandle *>((*randomFace)->neighbor(neighbourIndex)->info().getTags())->getHandles()->begin();
					std::advance(randomTag, rand()%numberOfTags);
					tagToAssign = *randomTag;
					if (alsoUniverse || tagToAssign != &universe) break;
				}
			}
			
			// Assign the region to the random tag
			for (std::set<Triangulation::Face_handle>::iterator currentFaceInRegion = facesInRegion.begin(); currentFaceInRegion != facesInRegion.end(); ++currentFaceInRegion) {
				facesToRepair.push_back(std::pair<Triangulation::Face_handle, PolygonHandle *>(*currentFaceInRegion, tagToAssign));
			}
		}
	}
	
	// Re-tag faces in the vector
	for (std::vector<std::pair<Triangulation::Face_handle, PolygonHandle *> >::iterator currentFace = facesToRepair.begin(); currentFace != facesToRepair.end(); ++currentFace) {
		currentFace->first->info().removeAllTags();
		currentFace->first->info().addTag(currentFace->second);
	}
	
	return repaired;
}

//the higher the priority number is, the lower priority it is
//confusing
bool IOWorker::ReadPriorityMaph(const char *file)
{
	// Process priority file
	std::ifstream priorityFile;
	priorityFile.open(file, std::ios::in);
	if (!priorityFile.is_open()) {
		std::cout << "Priority file could not be opened." << std::endl;
		return false;
	} std::map<Field *, unsigned int, FieldComparator> priorityMap;

	unsigned int currentPriority = 0;
	while (!priorityFile.eof()) {
		switch (schemaFieldType) {
		case OFTString: {
			std::string fieldAsString;
			// If we deal with strings, take a whole line (since spaces could be valid)
			std::getline(priorityFile, fieldAsString);		
			StringField *newField = new StringField(fieldAsString.c_str());
			priorityMap[newField] = currentPriority;
			break;} 
		case OFTReal: {
			double fieldAsDouble;
			priorityFile >> fieldAsDouble;
			DoubleField *newField = new DoubleField(fieldAsDouble);
			priorityMap[newField] = currentPriority;} 
		case OFTInteger: {
			int fieldAsInt;
			priorityFile >> fieldAsInt;
			IntField *newField = new IntField(fieldAsInt);
			priorityMap[newField] = currentPriority;} 
		default: {
			std::cout << "Field type not supported." << std::endl;
			std::string fieldAsString;
			std::getline(priorityFile, fieldAsString);
			break;}
		} 
		++currentPriority;
	} 
	priorityFile.close();
}

bool IOWorker::repairByPriorityList(Triangulation &triangulation, const std::vector<int>& priorList)
{
	if (priorList.empty()) return false;
	
	std::vector<int>::const_iterator itr;
	priorityMap.clear();
	unsigned int currentPriority = 0;

	for (itr=priorList.begin(); itr!=priorList.end(); itr++) {
		IntField *newField = new IntField(int(*itr));
		priorityMap[newField] = currentPriority;
	}
	
	return DoRepairByPriorityList(triangulation);
}

bool IOWorker::DoRepairByPriorityList(Triangulation &triangulation)
{
	// Use a temporary vector to make it deterministic and order independent
	std::vector<std::pair<Triangulation::Face_handle, PolygonHandle *> > facesToRepair;
	std::set<Triangulation::Face_handle> processedFaces;
	for (Triangulation::Finite_faces_iterator currentFace = triangulation.finite_faces_begin(); currentFace != triangulation.finite_faces_end(); ++currentFace) 
	{
		if (currentFace->info().hasOneTag()  || processedFaces.count(currentFace)) continue;

		// Expand this triangle into a complete region
		std::set<Triangulation::Face_handle> facesInRegion;
		facesInRegion.insert(currentFace);
		std::stack<Triangulation::Face_handle> facesToProcess;
		facesToProcess.push(currentFace);
		while (facesToProcess.size() > 0) {
			Triangulation::Face_handle currentFaceInStack = facesToProcess.top();
			facesToProcess.pop();
			processedFaces.insert(currentFaceInStack);
			if (!currentFaceInStack->neighbor(0)->info().hasOneTag() && !facesInRegion.count(currentFaceInStack->neighbor(0)) && 
				!triangulation.is_constrained(std::pair<Triangulation::Face_handle, int>(currentFaceInStack, 0))) {
					facesInRegion.insert(currentFaceInStack->neighbor(0));
					facesToProcess.push(currentFaceInStack->neighbor(0));
			} if (!currentFaceInStack->neighbor(1)->info().hasOneTag() && !facesInRegion.count(currentFaceInStack->neighbor(1)) && 
				!triangulation.is_constrained(std::pair<Triangulation::Face_handle, int>(currentFaceInStack, 1))) {
					facesInRegion.insert(currentFaceInStack->neighbor(1));
					facesToProcess.push(currentFaceInStack->neighbor(1));
			} if (!currentFaceInStack->neighbor(2)->info().hasOneTag() && !facesInRegion.count(currentFaceInStack->neighbor(2)) && 
				!triangulation.is_constrained(std::pair<Triangulation::Face_handle, int>(currentFaceInStack, 2))) {
					facesInRegion.insert(currentFaceInStack->neighbor(2));
					facesToProcess.push(currentFaceInStack->neighbor(2));
			}
		}

		// Find the tag with the highest priority
		PolygonHandle *tagToAssign = NULL;
		unsigned int priorityOfTag = UINT_MAX;
		for (std::set<Triangulation::Face_handle>::iterator currentFaceInRegion = facesInRegion.begin(); currentFaceInRegion != facesInRegion.end(); ++currentFaceInRegion) {
			// Gap, check neighbours
			if ((*currentFaceInRegion)->info().hasNoTags()) {
				if (!(*currentFaceInRegion)->neighbor(0)->info().hasNoTags()) {
					if ((*currentFaceInRegion)->neighbor(0)->info().hasOneTag() && (*currentFaceInRegion)->neighbor(0)->info().getTags() != &universe) {
						if (priorityMap[(*currentFaceInRegion)->neighbor(0)->info().getTags()->getSchemaField()] < priorityOfTag) {
							priorityOfTag = priorityMap[(*currentFaceInRegion)->neighbor(0)->info().getTags()->getSchemaField()];
							tagToAssign = (*currentFaceInRegion)->neighbor(0)->info().getTags();
						}
					} 
					else {
						MultiPolygonHandle *handle = static_cast<MultiPolygonHandle *>((*currentFaceInRegion)->neighbor(0)->info().getTags());
						for (std::list<PolygonHandle *>::const_iterator currentTag = handle->getHandles()->begin(); currentTag != handle->getHandles()->end(); ++currentTag) {
							if (*currentTag == &universe) continue;
							if (priorityMap[(*currentTag)->getSchemaField()] < priorityOfTag) {
								priorityOfTag = priorityMap[(*currentTag)->getSchemaField()];
								tagToAssign = *currentTag;
							}
						}
					}
				}

				if (!(*currentFaceInRegion)->neighbor(1)->info().hasNoTags()) {
					if ((*currentFaceInRegion)->neighbor(1)->info().hasOneTag() && (*currentFaceInRegion)->neighbor(1)->info().getTags() != &universe) {
						if (priorityMap[(*currentFaceInRegion)->neighbor(1)->info().getTags()->getSchemaField()] < priorityOfTag) {
							priorityOfTag = priorityMap[(*currentFaceInRegion)->neighbor(1)->info().getTags()->getSchemaField()];
							tagToAssign = (*currentFaceInRegion)->neighbor(1)->info().getTags();
						}
					} else {
						MultiPolygonHandle *handle = static_cast<MultiPolygonHandle *>((*currentFaceInRegion)->neighbor(1)->info().getTags());
						for (std::list<PolygonHandle *>::const_iterator currentTag = handle->getHandles()->begin(); currentTag != handle->getHandles()->end(); ++currentTag) {
							if (*currentTag == &universe) continue;
							if (priorityMap[(*currentTag)->getSchemaField()] < priorityOfTag) {
								priorityOfTag = priorityMap[(*currentTag)->getSchemaField()];
								tagToAssign = *currentTag;
							}
						}
					}
				} 

				if (!(*currentFaceInRegion)->neighbor(2)->info().hasNoTags()) {
					if ((*currentFaceInRegion)->neighbor(2)->info().hasOneTag() && (*currentFaceInRegion)->neighbor(2)->info().getTags() != &universe) {
						if (priorityMap[(*currentFaceInRegion)->neighbor(2)->info().getTags()->getSchemaField()] < priorityOfTag) {
							priorityOfTag = priorityMap[(*currentFaceInRegion)->neighbor(2)->info().getTags()->getSchemaField()];
							tagToAssign = (*currentFaceInRegion)->neighbor(2)->info().getTags();
						}
					} else {
						MultiPolygonHandle *handle = static_cast<MultiPolygonHandle *>((*currentFaceInRegion)->neighbor(2)->info().getTags());
						for (std::list<PolygonHandle *>::const_iterator currentTag = handle->getHandles()->begin(); currentTag != handle->getHandles()->end(); ++currentTag) {
							if (*currentTag == &universe) continue;
							if (priorityMap[(*currentTag)->getSchemaField()] < priorityOfTag) {
								priorityOfTag = priorityMap[(*currentTag)->getSchemaField()];
								tagToAssign = *currentTag;
							}
						}
					}
				}
			}
			else {// Overlap, check this one
				if ((*currentFaceInRegion)->info().hasOneTag() && (*currentFaceInRegion)->info().getTags() != &universe) {
					if (priorityMap[(*currentFaceInRegion)->info().getTags()->getSchemaField()] < priorityOfTag) {
						priorityOfTag = priorityMap[(*currentFaceInRegion)->info().getTags()->getSchemaField()];
						tagToAssign = (*currentFaceInRegion)->info().getTags();
					}
				} 
				else {
					MultiPolygonHandle *handle = static_cast<MultiPolygonHandle *>((*currentFaceInRegion)->info().getTags());
					for (std::list<PolygonHandle *>::const_iterator currentTag = handle->getHandles()->begin(); currentTag != handle->getHandles()->end(); ++currentTag) {
						if (*currentTag == &universe) continue;
						if (priorityMap[(*currentTag)->getSchemaField()] < priorityOfTag) {
							priorityOfTag = priorityMap[(*currentTag)->getSchemaField()];
							tagToAssign = *currentTag;
						}
					}
				}
			}
		}

		// Assign the tag to the triangles in the region
		for (std::set<Triangulation::Face_handle>::iterator currentFaceInRegion = facesInRegion.begin(); currentFaceInRegion != facesInRegion.end(); ++currentFaceInRegion) {
			facesToRepair.push_back(std::pair<Triangulation::Face_handle, PolygonHandle *>(*currentFaceInRegion, tagToAssign));
		}
	}

	// Re-tag faces in the vector
	for (std::vector<std::pair<Triangulation::Face_handle, PolygonHandle *> >::iterator currentFace = facesToRepair.begin(); currentFace != facesToRepair.end(); ++currentFace) {
		currentFace->first->info().removeAllTags();
		currentFace->first->info().addTag(currentFace->second);
	}

	return true;
}

std::set<Triangulation::Face_handle> IOWorker::RegionGrow(Triangulation &triangulation, Triangulation::Face_handle& seedFace)
{
	// Expand this triangle into a complete region
	std::set<Triangulation::Face_handle> facesInRegion;
	facesInRegion.insert(seedFace);
	std::stack<Triangulation::Face_handle> facesStack;
	facesStack.push(seedFace);
	Triangulation::Face_handle curFace, neibFace;

	while (facesStack.size() > 0) {
		curFace = facesStack.top();
		facesStack.pop();

		for (int i=0; i<3; i++)	{
			neibFace = curFace->neighbor(i);
			if (neibFace->info()!=curFace->info()) continue;
			if (facesInRegion.count(neibFace)) continue;
			if(triangulation.is_constrained(std::pair<Triangulation::Face_handle, int>(curFace, i)))
				continue;
			facesInRegion.insert(neibFace);
			facesStack.push(neibFace);
		}
	}

	
	return facesInRegion;
}

//only repair the top roof layer
//the top layer is tagged by the highest priority
//the overlapped area is repair to the first layer polygon under the top layer
//the gap area is repaired to be the top layer
bool IOWorker::RepairTopRoofLayer(Triangulation &triangulation, const std::vector<int>& priorList)
{
	
	//////////////////////////////////////////////////////////////////////////
	//find the tag of top layer
	if (priorList.empty()) return false;
	int topLayNum = priorList[0];
	PolygonHandle* topLayerTag, *curPolTag;
	for (Triangulation::Finite_faces_iterator currentFace = triangulation.finite_faces_begin(); currentFace != triangulation.finite_faces_end(); ++currentFace) {
		if (!currentFace->info().hasOneTag()) continue;
		curPolTag = currentFace->info().getOneTag();
		if (curPolTag->getSchemaField()->getType()!=OFTInteger) return false;		
		IntField* filed = static_cast<IntField *>(curPolTag->getSchemaField());
		
		if (filed->getValueAsInt()==topLayNum) {
			topLayerTag = curPolTag;
			break;
		}
	}
	
	// Use a temporary vector to make it deterministic and order independent
	std::vector<std::pair<Triangulation::Face_handle, PolygonHandle *> > facesToRepair;
	std::set<Triangulation::Face_handle> processedFaces, facesInRegion;
	PolygonHandle* tagToAssign;
	for (Triangulation::Finite_faces_iterator currentFace = triangulation.finite_faces_begin(); currentFace != triangulation.finite_faces_end(); ++currentFace) 
	{
		if (currentFace->info().hasOneTag()  || processedFaces.count(currentFace)) continue;

		// Expand this triangle into a complete region
		//biao
		Triangulation::Face_handle curFaceHandle = Triangulation::Face_handle(currentFace);
		facesInRegion = RegionGrow(triangulation, curFaceHandle);
		processedFaces.insert(facesInRegion.begin(), facesInRegion.end());
		
		if (currentFace->info().hasNoTags()){//gap, assign to the top layer
			for (std::set<Triangulation::Face_handle>::iterator curFace = facesInRegion.begin(); curFace != facesInRegion.end(); ++curFace)
				facesToRepair.push_back(std::pair<Triangulation::Face_handle, PolygonHandle *>(*curFace, topLayerTag));
		}
		else if(currentFace->info().numberOfTags()>1) {//overlap, assign to the layer under top layer
			for (std::set<Triangulation::Face_handle>::iterator curFace = facesInRegion.begin(); curFace != facesInRegion.end(); ++curFace) {
				MultiPolygonHandle *handle = static_cast<MultiPolygonHandle *>((*curFace)->info().getTags());
				
				//find the tag of layer behind top layer
				for (std::list<PolygonHandle *>::const_iterator currentTag = handle->getHandles()->begin(); currentTag != handle->getHandles()->end(); ++currentTag) {
					if (*currentTag == topLayerTag) continue;
					tagToAssign = *currentTag;
					break;
				}

				facesToRepair.push_back(std::pair<Triangulation::Face_handle, PolygonHandle *>(*curFace, tagToAssign));
			}
		}
		
		// Assign the tag to the triangles in the region
		for (std::set<Triangulation::Face_handle>::iterator currentFaceInRegion = facesInRegion.begin(); currentFaceInRegion != facesInRegion.end(); ++currentFaceInRegion) {
			facesToRepair.push_back(std::pair<Triangulation::Face_handle, PolygonHandle *>(*currentFaceInRegion, tagToAssign));
		}
	}

	// Re-tag faces in the vector
	for (std::vector<std::pair<Triangulation::Face_handle, PolygonHandle *> >::iterator currentFace = facesToRepair.begin(); currentFace != facesToRepair.end(); ++currentFace) {
		currentFace->first->info().removeAllTags();
		currentFace->first->info().addTag(currentFace->second);
	}
	
	return true;
}

double ComputeRegionArea(const std::set<Triangulation::Face_handle>& region)
{
	Triangulation::Triangle triangle;
	double area = 0.0;
	std::set<Triangulation::Face_handle>::const_iterator face;
	for (face=region.begin(); face!=region.end(); face++) {
		triangle = Triangulation::Triangle((*face)->vertex(0)->point(), (*face)->vertex(1)->point(), (*face)->vertex(2)->point());
		area += CGAL::to_double(triangle.area());
	}

	return area;
}

//Biao 2013, 07, 20
//Snap the top roof layer to footprint
//the all roof layers are inside the footprint pol
//bKeepHole: true, keep the biggest hole. false, do not keep hole
bool IOWorker::RepairBySnaptoFootPrint(Triangulation &triangulation, const std::vector<int>& priorList, bool bKeepHole)
{
	//////////////////////////////////////////////////////////////////////////
	//find the tag of top and bottom layer
	if (priorList.size()<2) return false;
	int topLayNum = priorList[0], footLayNum = priorList[priorList.size()-1];
	PolygonHandle* topLayerTag=NULL, *footLayerTag=NULL, *curPolTag=NULL;
	for (Triangulation::Finite_faces_iterator currentFace = triangulation.finite_faces_begin(); currentFace != triangulation.finite_faces_end(); ++currentFace) {
		FaceInfo& info = currentFace->info();
		if (currentFace->info().getTags()==&universe || !currentFace->info().hasOneTag()) continue;
		curPolTag = currentFace->info().getOneTag();
		if (curPolTag->getSchemaField()->getType()!=OFTInteger) return false;
		IntField* filed = static_cast<IntField *>(curPolTag->getSchemaField());

		if (filed->getValueAsInt()==topLayNum)
			topLayerTag = curPolTag;
		else if (filed->getValueAsInt()==footLayNum)
			footLayerTag = curPolTag;

		if (topLayerTag&&footLayerTag) break;
	}

	if (!topLayerTag||!footLayerTag) return false;
	
	//////////////////////////////////////////////////////////////////////////
	// Use a temporary vector to make it deterministic and order independent
	std::vector<std::pair<Triangulation::Face_handle, PolygonHandle *> > facesToRepair;
	std::set<Triangulation::Face_handle> processedFaces, facesInRegion;
	vector<std::set<Triangulation::Face_handle> > vecFootRegs;
	PolygonHandle* tagToAssign;
	for (Triangulation::Finite_faces_iterator currentFace = triangulation.finite_faces_begin(); currentFace != triangulation.finite_faces_end(); ++currentFace) 
	{
		FaceInfo& info = currentFace->info();
		if(DebugTriangle(currentFace, 154980.45,461752.12, 154979.46,461751.49, 154979.46,461751.49)
			||DebugTriangle(currentFace, 154980.45,461752.12, 154979.46,461751.49,  154980.45,461752.12))
			int aaa = 0;

		if (currentFace->info().hasTag(&universe) || processedFaces.count(currentFace)) continue;

		// Expand this triangle into a complete region
		//biao
		Triangulation::Face_handle  curFaceHandle = Triangulation::Face_handle(currentFace);
		facesInRegion = RegionGrow(triangulation, curFaceHandle);
		//facesInRegion = RegionGrow(triangulation, (Triangulation::Face_handle(currentFace)));
		
		processedFaces.insert(facesInRegion.begin(), facesInRegion.end());

		//gap area in side the footprint area, some bugs in the tagtrinaglation result in this wrong tag
		//this bug should be fixed
		if (currentFace->info().hasNoTags()) {
			for (std::set<Triangulation::Face_handle>::iterator curFace = facesInRegion.begin(); curFace != facesInRegion.end(); ++curFace)
				//vecFootRegs.push_back(facesInRegion);
				facesToRepair.push_back(std::pair<Triangulation::Face_handle, PolygonHandle *>(*curFace, topLayerTag));
		}
		else if (!currentFace->info().hasTag(footLayerTag)) {//outside footprint area
			for (std::set<Triangulation::Face_handle>::iterator curFace = facesInRegion.begin(); curFace != facesInRegion.end(); ++curFace)
				facesToRepair.push_back(std::pair<Triangulation::Face_handle, PolygonHandle *>(*curFace, &universe));//universe tag?
		}
		else {//inside footprint area
			if(currentFace->info().hasOneTag()) {//foot region
				vecFootRegs.push_back(facesInRegion);
			}
			else if (currentFace->info().numberOfTags()==2) {//only one roof layer overlapped with footprint
				//get un-foot-layer tag
				MultiPolygonHandle *handle = static_cast<MultiPolygonHandle *>((*facesInRegion.begin())->info().getTags());
				for (std::list<PolygonHandle *>::const_iterator currentTag = handle->getHandles()->begin(); currentTag != handle->getHandles()->end(); ++currentTag) {
					if (*currentTag == footLayerTag) continue;
					tagToAssign = *currentTag;
					break;
				}

				for (std::set<Triangulation::Face_handle>::iterator curFace = facesInRegion.begin(); curFace != facesInRegion.end(); ++curFace)
					facesToRepair.push_back(std::pair<Triangulation::Face_handle, PolygonHandle *>(*curFace, tagToAssign));
			}
			else {//several roof layer overlapped with footprint
				for (std::set<Triangulation::Face_handle>::iterator curFace = facesInRegion.begin(); curFace != facesInRegion.end(); ++curFace) {
					//find tag that is not foot layer tag or top layer tag
					MultiPolygonHandle *handle = static_cast<MultiPolygonHandle *>((*facesInRegion.begin())->info().getTags());
					for (std::list<PolygonHandle *>::const_iterator curTag = handle->getHandles()->begin(); curTag != handle->getHandles()->end(); ++curTag) {
						if (*curTag==footLayerTag || *curTag==topLayerTag) continue;
						tagToAssign = *curTag;
						break;
					}

					facesToRepair.push_back(std::pair<Triangulation::Face_handle, PolygonHandle *>(*curFace, tagToAssign));
				}
			}
		}
	}

	/*
	Triangulation::Triangle triangle;
	for (Triangulation::Finite_faces_iterator currentFace = triangulation.finite_faces_begin(); currentFace != triangulation.finite_faces_end(); ++currentFace) 
	{
		FaceInfo& info = currentFace->info();
		triangle = Triangulation::Triangle(currentFace->vertex(0)->point(),currentFace->vertex(1)->point(),currentFace->vertex(2)->point());
		if (DebugTriangle(currentFace, 154992.97, 461755.32, 154992.97, 461755.63, 154992.97, 461756.01))
		{
			int aaa = 0;
		}
		
		if (triangle.is_degenerate()) {
			int aaa  = 0;
		}
		
		if (currentFace->info().hasTag(&universe) || processedFaces.count(currentFace)) continue;

		// Expand this triangle into a complete region
		facesInRegion = RegionGrow(triangulation, (Triangulation::Face_handle(currentFace)));
		processedFaces.insert(facesInRegion.begin(), facesInRegion.end());

		//gap area in side the footprint area, some bugs in the tagtrinaglation result in this wrong tag
		//this bug should be fixed
		MultiPolygonHandle *mulHandle;
		tagToAssign = NULL;
		switch (currentFace->info().numberOfTags())
		{
		case 0://gap inside the footprint
			tagToAssign = topLayerTag;
			//tagToAssign = &universe;
			break;
		case 1:
			if(currentFace->info().hasTag(footLayerTag)) //footprint areas
				vecFootRegs.push_back(facesInRegion);
			else //outside footprint area
				tagToAssign = &universe;
			break;
		default://more than 1
			mulHandle = static_cast<MultiPolygonHandle *>((*facesInRegion.begin())->info().getTags());
			for (std::list<PolygonHandle *>::const_iterator curTag = mulHandle->getHandles()->begin(); curTag != mulHandle->getHandles()->end(); ++curTag) {
				if (*curTag==footLayerTag||*curTag==topLayerTag) continue;
				tagToAssign = *curTag;
				break;
			}

			if (!tagToAssign) //do not have the third tag
				tagToAssign = (*facesInRegion.begin())->info().hasTag(topLayerTag)?topLayerTag:footLayerTag;
			break;
		}

		if(tagToAssign) {
			for (std::set<Triangulation::Face_handle>::iterator curFace = facesInRegion.begin(); curFace != facesInRegion.end(); ++curFace)
				facesToRepair.push_back(std::pair<Triangulation::Face_handle, PolygonHandle *>(*curFace, tagToAssign));
		}
	}*/
	
	//////////////////////////////////////////////////////////////////////////
	//repair small footprint areas
	std::set<PolygonHandle *> adjAreaTag;
	int size1 = vecFootRegs.size(), size2;
	double temArea, maxArea=-9999.0;
	if (vecFootRegs.size()>=2) {
		//find the main footprint area
		//triangle area or numbers?
		//triangle number is not accurate
		int indMaxArea, maxTriCount=-1;
		for (int i=0; i<vecFootRegs.size(); i++) {
			/*size2 = vecFootRegs[i].size();
			if (size2>maxTriCount) {
				maxTriCount = size2;
				indMaxArea = i;
			}*/
			temArea = ComputeRegionArea(vecFootRegs[i]);
			if (temArea>maxArea) {
				maxArea = temArea;
				indMaxArea = i;
			}
		}

		//repair the tiny footprint areas
		
		for (int i=0; i<vecFootRegs.size(); i++) {
			if(bKeepHole && i==indMaxArea) continue;//kept the biggest hole
			//check whether the area is adjacent to top layer area
			adjAreaTag.clear();
			for (std::set<Triangulation::Face_handle>::iterator itrFace=vecFootRegs[i].begin(); itrFace!=vecFootRegs[i].end(); itrFace++) {
				for (int j=0; j<3; j++)	{
					if(!triangulation.is_constrained(std::pair<Triangulation::Face_handle, int>(*itrFace, j)))
						continue;
					FaceInfo& faceInfo = (*itrFace)->neighbor(j)->info();
					if (faceInfo.hasNoTags()) continue;

					if(faceInfo.hasOneTag())
						adjAreaTag.insert(faceInfo.getTags());
					else {
						MultiPolygonHandle * polyTag = static_cast<MultiPolygonHandle *>(faceInfo.getTags());
						list<PolygonHandle*>::const_iterator itrTag;
						for (itrTag=polyTag->getHandles()->begin(); itrTag!=polyTag->getHandles()->end(); itrTag++) {
							if (*itrTag!=footLayerTag)
								adjAreaTag.insert(*itrTag);
						}
					}					
				}
			}

			//if adjacent to top layer, re-tag with the top layer tag, otherwise re-tag with tag of random adjacent area
			PolygonHandle* thirdTag=NULL;
			for (std::set<PolygonHandle *>::iterator itrTag=adjAreaTag.begin(); itrTag!=adjAreaTag.end(); itrTag++) {
				if (*itrTag!=&universe && (*itrTag)!=topLayerTag && (*itrTag)!=footLayerTag) {
					thirdTag = *itrTag;
					break;
				}				
			}
			
			int areaSize = vecFootRegs[i].size();
			if(adjAreaTag.count(topLayerTag))
				tagToAssign = topLayerTag;
			else if(thirdTag)
				tagToAssign = thirdTag;
			else
				tagToAssign = &universe;
			
			//tagToAssign = adjAreaTag.count(topLayerTag) ? topLayerTag:*adjAreaTag.begin();
			for (std::set<Triangulation::Face_handle>::iterator itrFace=vecFootRegs[i].begin(); itrFace!=vecFootRegs[i].end(); itrFace++) {
				facesToRepair.push_back(std::pair<Triangulation::Face_handle, PolygonHandle *>(*itrFace, tagToAssign));
			}
		}
	}	

	//////////////////////////////////////////////////////////////////////////
	// Re-tag faces in the vector
	for (std::vector<std::pair<Triangulation::Face_handle, PolygonHandle *> >::iterator currentFace = facesToRepair.begin(); currentFace != facesToRepair.end(); ++currentFace) {
		currentFace->first->info().removeAllTags();
		currentFace->first->info().addTag(currentFace->second);
	}

	return true;
}

//Biao 2013, 08, 16
//Snap all contours to the footprints
//the all roof layers are inside the footprint pol
//bKeepHole: true, keep the biggest hole. false, do not keep hole
bool IOWorker::RepairBySnaptoFootPrint2(Triangulation &triangulation, const std::vector<int>& priorList, bool bKeepHole)
{
	//////////////////////////////////////////////////////////////////////////
	//find the tag of footprint layer
	if (priorList.size()<2) return false;
	int footLayNum = priorList[priorList.size()-1];
	PolygonHandle* topLayerTag=NULL, *footLayerTag=NULL, *curPolTag=NULL;
	for (Triangulation::Finite_faces_iterator currentFace = triangulation.finite_faces_begin(); currentFace != triangulation.finite_faces_end(); ++currentFace) {
		FaceInfo& info = currentFace->info();
		if (currentFace->info().getTags()==&universe || !currentFace->info().hasOneTag()) continue;
		curPolTag = currentFace->info().getOneTag();
		if (curPolTag->getSchemaField()->getType()!=OFTInteger) return false;
		IntField* filed = static_cast<IntField *>(curPolTag->getSchemaField());

		if (filed->getValueAsInt()==footLayNum) {
			footLayerTag = curPolTag;
			break;
		}
	}

	if (!footLayerTag) return false;
	
	//////////////////////////////////////////////////////////////////////////
	// Use a temporary vector to make it deterministic and order independent
	std::vector<std::pair<Triangulation::Face_handle, PolygonHandle *> > facesToRepair;
	std::set<Triangulation::Face_handle> processedFaces, facesInRegion;
	vector<std::set<Triangulation::Face_handle> > vecFootRegs;
	PolygonHandle* tagToAssign;
	//for (Triangulation::Finite_faces_iterator currentFace = triangulation.finite_faces_begin(); currentFace != triangulation.finite_faces_end(); ++currentFace) 
	for (Triangulation::Face_iterator currentFace=triangulation.faces_begin(); currentFace!=triangulation.faces_end(); ++currentFace)
	{
		FaceInfo& info = currentFace->info();
#ifdef _DEBUG
		if(DebugTriangle(currentFace, 154980.45,461752.12, 154979.46,461751.49, 154979.46,461751.49)
			||DebugTriangle(currentFace, 154980.45,461752.12, 154979.46,461751.49,  154980.45,461752.12))
			int aaa = 0;
#endif		

		if (currentFace->info().hasTag(&universe) || processedFaces.count(currentFace)) continue;

		// Expand this triangle into a complete region
		//biao
		Triangulation::Face_handle  curFaceHandle = Triangulation::Face_handle(currentFace);
		facesInRegion = RegionGrow(triangulation, curFaceHandle);
		//facesInRegion = RegionGrow(triangulation, (Triangulation::Face_handle(currentFace)));
		processedFaces.insert(facesInRegion.begin(), facesInRegion.end());

		if (currentFace->info().hasNoTags()) {//should not happen
			for (std::set<Triangulation::Face_handle>::iterator curFace = facesInRegion.begin(); curFace != facesInRegion.end(); ++curFace)
				facesToRepair.push_back(std::pair<Triangulation::Face_handle, PolygonHandle *>(*curFace, topLayerTag));
		}
		else if (!currentFace->info().hasTag(footLayerTag)) {//outside footprint area
			for (std::set<Triangulation::Face_handle>::iterator curFace = facesInRegion.begin(); curFace != facesInRegion.end(); ++curFace)
				facesToRepair.push_back(std::pair<Triangulation::Face_handle, PolygonHandle *>(*curFace, &universe));//universe tag?
		}
		else {//inside footprint area
			if(currentFace->info().hasOneTag()) {//foot region
				vecFootRegs.push_back(facesInRegion);
			}
			else if (currentFace->info().numberOfTags()==2) {//only one roof layer overlapped with footprint
				//get un-foot-layer tag
				MultiPolygonHandle *handle = static_cast<MultiPolygonHandle *>((*facesInRegion.begin())->info().getTags());
				for (std::list<PolygonHandle *>::const_iterator currentTag = handle->getHandles()->begin(); currentTag != handle->getHandles()->end(); ++currentTag) {
					if (*currentTag == footLayerTag) continue;
					tagToAssign = *currentTag;
					break;
				}

				for (std::set<Triangulation::Face_handle>::iterator curFace = facesInRegion.begin(); curFace != facesInRegion.end(); ++curFace)
					facesToRepair.push_back(std::pair<Triangulation::Face_handle, PolygonHandle *>(*curFace, tagToAssign));
			}
			else {//several roof layer overlapped with footprint
				for (std::set<Triangulation::Face_handle>::iterator curFace = facesInRegion.begin(); curFace != facesInRegion.end(); ++curFace) {
					//find tag that is not foot layer tag or top layer tag
					tagToAssign = GetTopPriorityTag(*curFace, priorList);
					if (!tagToAssign) continue;
					facesToRepair.push_back(std::pair<Triangulation::Face_handle, PolygonHandle *>(*curFace, tagToAssign));
				}
			}
		}
	}
	
	//////////////////////////////////////////////////////////////////////////
	//repair small footprint areas
	std::set<PolygonHandle *> adjAreaTag;
	int size1 = vecFootRegs.size(), size2;
	double temArea, maxArea=-9999.0;
	if (!vecFootRegs.empty()) {
		//find the main footprint area
		//triangle area or numbers?
		//triangle number is not accurate
		int indMaxArea, maxTriCount=-1;
		for (int i=0; i<vecFootRegs.size(); i++) {
			temArea = ComputeRegionArea(vecFootRegs[i]);
			if (temArea>maxArea) {
				maxArea = temArea;
				indMaxArea = i;
			}
		}

		//repair the tiny footprint areas
		for (int i=0; i<vecFootRegs.size(); i++) {
			if(bKeepHole && i==indMaxArea) continue;//kept the biggest hole
			tagToAssign = GetNeibTopPriorityTag(triangulation, vecFootRegs[i], priorList);
			if (!tagToAssign) tagToAssign = &universe;

			for (std::set<Triangulation::Face_handle>::iterator itrFace=vecFootRegs[i].begin(); itrFace!=vecFootRegs[i].end(); itrFace++) {
				facesToRepair.push_back(std::pair<Triangulation::Face_handle, PolygonHandle *>(*itrFace, tagToAssign));
			}
		}
	}	

	//////////////////////////////////////////////////////////////////////////
	// Re-tag faces in the vector
	for (std::vector<std::pair<Triangulation::Face_handle, PolygonHandle *> >::iterator currentFace = facesToRepair.begin(); currentFace != facesToRepair.end(); ++currentFace) {
		currentFace->first->info().removeAllTags();
		currentFace->first->info().addTag(currentFace->second);
	}

	return true;
}

#include <queue>
//Biao, Jan 25, 2015
//keep the stable regions,
//competing all unstable regions to be merged by nearby big region
bool IOWorker::RepairBySnaptoFootPrint3(Triangulation &triangulation, const std::vector<int>& priorList, bool bKeepHole)
{
	//////////////////////////////////////////////////////////////////////////
	//find the tag of footprint layer
	if (priorList.size()<2) return false;
	int footLayNum = priorList[priorList.size()-1];
	PolygonHandle* topLayerTag=NULL, *footLayerTag=NULL, *curPolTag=NULL;
	for (Triangulation::Finite_faces_iterator currentFace = triangulation.finite_faces_begin(); currentFace != triangulation.finite_faces_end(); ++currentFace) {
		FaceInfo& info = currentFace->info();
		if (currentFace->info().getTags()==&universe || !currentFace->info().hasOneTag()) continue;
		curPolTag = currentFace->info().getOneTag();
		if (curPolTag->getSchemaField()->getType()!=OFTInteger) return false;
		IntField* filed = static_cast<IntField *>(curPolTag->getSchemaField());

		if (filed->getValueAsInt()==footLayNum) {
			footLayerTag = curPolTag;
			break;
		}
	}

	if (!footLayerTag) return false;

	//////////////////////////////////////////////////////////////////////////
	// Use a temporary vector to make it deterministic and order independent
	std::vector<std::pair<Triangulation::Face_handle, PolygonHandle *> > facesToRepair;
	std::set<Triangulation::Face_handle> processedFaces, facesInRegion;
	PolygonHandle* tagToAssign;
	typedef std::multimap<SetIntenterField, std::set<Triangulation::Face_handle> > MultiMapRegions;
	MultiMapRegions multiMapRegions;
	MultiMapRegions::iterator itrMap0, itrMap1, itrMapTem;
	//////////////////////////////////////////////////////////////////////////
	//get regions
	printf("\n\nRegion Growing:\n");
	
	for (Triangulation::Face_iterator currentFace=triangulation.faces_begin(); currentFace!=triangulation.faces_end(); ++currentFace)
	{
		FaceInfo& info = currentFace->info();
#ifdef _DEBUG
		if(DebugTriangle(currentFace, 154980.45,461752.12, 154979.46,461751.49, 154979.46,461751.49)
			||DebugTriangle(currentFace, 154980.45,461752.12, 154979.46,461751.49,  154980.45,461752.12))
			int aaa = 0;
#endif

		if (currentFace->info().hasTag(&universe) || processedFaces.count(currentFace)) continue;

		// Expand this triangle into a complete region
		//biao
		Triangulation::Face_handle  curFaceHandle = Triangulation::Face_handle(currentFace);
		facesInRegion = RegionGrow(triangulation, curFaceHandle);
		//facesInRegion = RegionGrow(triangulation, (Triangulation::Face_handle(currentFace)));
		processedFaces.insert(facesInRegion.begin(), facesInRegion.end());


		if (currentFace->info().hasNoTags()) {//should not happen
			for (std::set<Triangulation::Face_handle>::iterator curFace = facesInRegion.begin(); curFace != facesInRegion.end(); ++curFace){
				//facesToRepair.push_back(std::pair<Triangulation::Face_handle, PolygonHandle *>(*curFace, topLayerTag));
				(*curFace)->info().substituteTagsWith(topLayerTag);
			}
		}
		else if (!currentFace->info().hasTag(footLayerTag)) {//outside footprint area
			for (std::set<Triangulation::Face_handle>::iterator curFace = facesInRegion.begin(); curFace != facesInRegion.end(); ++curFace) {
				//facesToRepair.push_back(std::pair<Triangulation::Face_handle, PolygonHandle *>(*curFace, &universe));//universe tag
				(*curFace)->info().substituteTagsWith(&universe);
			}
		}
		else {//inside footprint area
			multiMapRegions.insert(std::make_pair(SetIntenterField(currentFace->info().getTags()), facesInRegion));
			int nTags = currentFace->info().numberOfTags();
			printf("Regions: %4d triangles, %4d tags.\n", facesInRegion.size(), nTags);
			if(nTags <= 1) 
				int aaa = 0;
		}
	}
	
	printf("Got %4d Regions.\n", multiMapRegions.size());

	////////////////////////////////////////////////////////////////////////////
	//get one stable region for each label in the priory list, except -1 label, which is the footprint region
	for (unsigned int i=0; i<priorList.size()-1; i++) {
		double temArea, maxArea=-9999.0;
		itrMap0 = multiMapRegions.end();
		
		for (itrMapTem=multiMapRegions.begin(); itrMapTem!=multiMapRegions.end(); itrMapTem++) {
			if (!itrMapTem->first.HasField(priorList[i])) continue;
			temArea = ComputeRegionArea(itrMapTem->second);
			if (temArea<=maxArea) continue;
			maxArea = temArea;
			itrMap0 = itrMapTem;
		}

		if (itrMap0 == multiMapRegions.end()) continue;
		
		Triangulation::Face_handle debugFace = *itrMap0->second.begin();
		printf("Layer %d with %4.2f area, %d tags.\n", priorList[i], maxArea, debugFace->info().numberOfTags());

		//repair this region
		ChangeRegionTagWithFieldNum(itrMap0->second, priorList[i]);
		facesInRegion = itrMap0->second;
		multiMapRegions.erase(itrMap0);

		//expand this region to nearby region
		for (itrMapTem=multiMapRegions.begin(); itrMapTem!=multiMapRegions.end(); ) {
			if (!itrMapTem->first.HasField(priorList[i]) || 
				!IsNearbyRegion(facesInRegion, itrMapTem->second)) {
					itrMapTem++;
			}
			else {
				debugFace = *itrMapTem->second.begin();
				printf("\tExpanded region with %2d triangle, %d tags.\n", 
					itrMapTem->second.size(), debugFace->info().numberOfTags());
				ChangeRegionTagWithFieldNum(itrMapTem->second, priorList[i]);
				
				MultiMapRegions::iterator save = itrMapTem;
				++save;
				multiMapRegions.erase(itrMapTem);
				itrMapTem = save;
			}
		}
	}


	//////////////////////////////////////////////////////////////////////////
	//tag regions with stable neighbor regions
	std::queue<std::set<Triangulation::Face_handle> > queFacesToRepair;
	for (itrMap0=multiMapRegions.begin(); itrMap0!=multiMapRegions.end(); itrMap0++) {
		queFacesToRepair.push(itrMap0->second);
	}

	int nLoops = 0;
	while (!queFacesToRepair.empty()) {
		if (nLoops++>5000) break;

		facesInRegion = queFacesToRepair.front();
		queFacesToRepair.pop();
		FaceInfo& info = (*facesInRegion.begin())->info();
		tagToAssign = NULL;

#ifdef _DEBUG
		if(facesInRegion.size() == 1 &&
			DebugTriangle(*facesInRegion.begin(), 185529.416,319951.428, 
			185529.416,319951.428, 185529.313, 319951.482)) {
				int aaa = 0;
		}
#endif		
		//get tagToAssign
		//tagToAssign = GetPriTagByStableNeibLongestBorder(triangulation, facesInRegion, priorList, 100.0);
		//if (!tagToAssign) {
		if (info.numberOfTags()==1)//hole area, only footprint tag
			tagToAssign = GetStableNeibBottomPriTag(triangulation, facesInRegion, priorList);
		else
			tagToAssign = GetStableNeibTopPriTag(triangulation, facesInRegion, priorList);	
		//}

		//Assign
		if (tagToAssign == NULL) {//cannot find stable neighbors
			queFacesToRepair.push(facesInRegion);
		}
		else {
			for (std::set<Triangulation::Face_handle>::iterator curFace = facesInRegion.begin(); 
				curFace != facesInRegion.end(); ++curFace)
				(*curFace)->info().substituteTagsWith(tagToAssign);
		}
	}

	return true;
}

void IOWorker::ChangeRegionTagWithFieldNum(const std::set<Triangulation::Face_handle>& regionFaces, int filedTag)
{
	PolygonHandle* tagToAssign;
	Triangulation::Face_handle face = *(regionFaces.begin());

	
	MultiPolygonHandle *handle = dynamic_cast<MultiPolygonHandle *>(face->info().getTags());
	if(handle == NULL) {
		int iii = 0;
		
		PolygonHandle* debugH = face->info().getTags();
		int nFields = debugH->getNumberOfFields();
		bool isMultiHandle = debugH->isMultiPolygonHandle();
		MultiPolygonHandle *debugMH1 = (MultiPolygonHandle *)face->info().getTags();
		MultiPolygonHandle *debugMH2 = static_cast<MultiPolygonHandle *>(face->info().getTags());
		int nH1 = debugMH1->numberOfHandles();
		int nH2 = debugMH2->numberOfHandles();
		
//		return;
	}
		
	
	int nH3 = handle->numberOfHandles();
	
	for (std::list<PolygonHandle *>::const_iterator currentTag = handle->getHandles()->begin(); 
		currentTag != handle->getHandles()->end(); ++currentTag) {
			if(!(*currentTag))
				continue; 
			
			if ((*currentTag)->getSchemaField()->getValueAsInt() == filedTag) {
				tagToAssign = *currentTag;
				break;
			}
	}

	for (std::set<Triangulation::Face_handle>::const_iterator curFace = regionFaces.begin(); 
		curFace != regionFaces.end(); ++curFace)
		(*curFace)->info().substituteTagsWith(tagToAssign);
}

bool IOWorker::IsNearbyRegion(const std::set<Triangulation::Face_handle>& lhv, const std::set<Triangulation::Face_handle>& rhv)
{
	std::set<Triangulation::Face_handle>::const_iterator itrF;
	int neibCount = 0;

	for (itrF=lhv.begin(); itrF!=lhv.end(); itrF++) {
		for (int j=0; j<3; j++)	{
			Face_handle neibF = (*itrF)->neighbor(j);
			if (rhv.count(neibF) != 0)
				neibCount++;
		}
	}


	for (itrF=rhv.begin(); itrF!=rhv.end(); itrF++) {
		for (int j=0; j<3; j++)	{
			Face_handle neibF = (*itrF)->neighbor(j);
			if (lhv.count(neibF) != 0)
				neibCount++;
		}
	}

	return neibCount>0;
}

/*bool IOWorker::RepairBySnaptoFootPrint3(Triangulation &triangulation, const std::vector<int>& priorList, bool bKeepHole)
{
	//////////////////////////////////////////////////////////////////////////
	//find the tag of footprint layer
	if (priorList.size()<2) return false;
	int footLayNum = priorList[priorList.size()-1];
	PolygonHandle* topLayerTag=NULL, *footLayerTag=NULL, *curPolTag=NULL;
	for (Triangulation::Finite_faces_iterator currentFace = triangulation.finite_faces_begin(); currentFace != triangulation.finite_faces_end(); ++currentFace) {
		FaceInfo& info = currentFace->info();
		if (currentFace->info().getTags()==&universe || !currentFace->info().hasOneTag()) continue;
		curPolTag = currentFace->info().getOneTag();
		if (curPolTag->getSchemaField()->getType()!=OFTInteger) return false;
		IntField* filed = static_cast<IntField *>(curPolTag->getSchemaField());

		if (filed->getValueAsInt()==footLayNum) {
			footLayerTag = curPolTag;
			break;
		}
	}

	if (!footLayerTag) return false;

	//////////////////////////////////////////////////////////////////////////
	// Use a temporary vector to make it deterministic and order independent
	std::vector<std::pair<Triangulation::Face_handle, PolygonHandle *> > facesToRepair;
	std::set<Triangulation::Face_handle> processedFaces, facesInRegion;
	PolygonHandle* tagToAssign;
	typedef std::multimap<SetIntenterField, std::set<Triangulation::Face_handle>> MultiMapRegions;
	MultiMapRegions multiMapRegions;

	//////////////////////////////////////////////////////////////////////////
	//get regions
	for (Triangulation::Face_iterator currentFace=triangulation.faces_begin(); currentFace!=triangulation.faces_end(); ++currentFace)
	{
		FaceInfo& info = currentFace->info();
#ifdef _DEBUG
		if(DebugTriangle(currentFace, 154980.45,461752.12, 154979.46,461751.49, 154979.46,461751.49)
			||DebugTriangle(currentFace, 154980.45,461752.12, 154979.46,461751.49,  154980.45,461752.12))
			int aaa = 0;
#endif		

		if (currentFace->info().hasTag(&universe) || processedFaces.count(currentFace)) continue;

		// Expand this triangle into a complete region
		facesInRegion = RegionGrow(triangulation, (Triangulation::Face_handle(currentFace)));
		processedFaces.insert(facesInRegion.begin(), facesInRegion.end());

		if (currentFace->info().hasNoTags()) {//should not happen
			for (std::set<Triangulation::Face_handle>::iterator curFace = facesInRegion.begin(); curFace != facesInRegion.end(); ++curFace){
				//facesToRepair.push_back(std::pair<Triangulation::Face_handle, PolygonHandle *>(*curFace, topLayerTag));
				(*curFace)->info().substituteTagsWith(topLayerTag);
			}
		}
		else if (!currentFace->info().hasTag(footLayerTag)) {//outside footprint area
			for (std::set<Triangulation::Face_handle>::iterator curFace = facesInRegion.begin(); curFace != facesInRegion.end(); ++curFace) {
				//facesToRepair.push_back(std::pair<Triangulation::Face_handle, PolygonHandle *>(*curFace, &universe));//universe tag
				(*curFace)->info().substituteTagsWith(&universe);
			}
		}
		else {//inside footprint area
			multiMapRegions.insert(std::make_pair(SetIntenterField(currentFace->info().getTags()), facesInRegion));
		}
	}

	//////////////////////////////////////////////////////////////////////////
	// Re-tag faces in the vector
	//for (std::vector<std::pair<Triangulation::Face_handle, PolygonHandle *> >::iterator currentFace = facesToRepair.begin(); currentFace != facesToRepair.end(); ++currentFace) {
	//	currentFace->first->info().removeAllTags();
	//	currentFace->first->info().addTag(currentFace->second);
	//}

	//////////////////////////////////////////////////////////////////////////
	//tag stable regions, which is with one positive tag, and is the biggest region with same tags
	std::queue<std::set<Triangulation::Face_handle>> queFacesToRepair;
	MultiMapRegions::iterator itrMap0, itrMap1, itrMapTem;
	for (itrMap0=multiMapRegions.begin(); itrMap0!=multiMapRegions.end(); itrMap0++) {
		if (itrMap0->first.fields.size()!=2) {//hole, or competitive area
			queFacesToRepair.push(itrMap0->second);
		}
		else {//only one positive tag
			std::pair<MultiMapRegions::iterator, MultiMapRegions::iterator> pair0;
			pair0 = multiMapRegions.equal_range(itrMap0->first);
			itrMap0 = pair0.second;
			itrMap0--;//for next iteration

			//get the biggest area
			double temArea, maxArea=-9999.0;
			for (itrMapTem=pair0.first; itrMapTem!=pair0.second; itrMapTem++){
				temArea = ComputeRegionArea(itrMapTem->second);
				if (temArea>maxArea) {
					maxArea = temArea;
					itrMap1 = itrMapTem;
				}
			}

			//repair the biggest area, and keep other
			for (itrMapTem=pair0.first; itrMapTem!=pair0.second; itrMapTem++){
				if (itrMapTem!=itrMap1)//store to waiting list
					queFacesToRepair.push(itrMapTem->second);
				else {//repair
					Triangulation::Face_handle face = *(itrMapTem->second.begin());
					MultiPolygonHandle *handle = static_cast<MultiPolygonHandle *>(face->info().getTags());
					for (std::list<PolygonHandle *>::const_iterator currentTag = handle->getHandles()->begin(); 
						currentTag != handle->getHandles()->end(); ++currentTag) {
						if (*currentTag == footLayerTag) continue;
						tagToAssign = *currentTag;
						break;
					}

					for (std::set<Triangulation::Face_handle>::iterator curFace = itrMapTem->second.begin(); 
						curFace != itrMapTem->second.end(); ++curFace)
						(*curFace)->info().substituteTagsWith(tagToAssign);
				}
			}
		}
	}


	//////////////////////////////////////////////////////////////////////////
	//tag regions with stable neighbor regions
	int nLoops = 0;
	while (!queFacesToRepair.empty()) {
		if (nLoops++>5000) break;

		facesInRegion = queFacesToRepair.front();
		queFacesToRepair.pop();
		FaceInfo& info = (*facesInRegion.begin())->info();
		tagToAssign = NULL;

#ifdef _DEBUG
		if(facesInRegion.size() == 1 &&
			DebugTriangle(*facesInRegion.begin(), 185529.416,319951.428, 
			185529.416,319951.428, 185529.313, 319951.482)) {
				int aaa = 0;
		}
#endif		
		//get tagToAssign
		//tagToAssign = GetPriTagByStableNeibLongestBorder(triangulation, facesInRegion, priorList, 100.0);
		//if (!tagToAssign) {
		if (info.numberOfTags()==1)//hole area, only footprint tag
			tagToAssign = GetStableNeibBottomPriTag(triangulation, facesInRegion, priorList);
		else
			tagToAssign = GetStableNeibTopPriTag(triangulation, facesInRegion, priorList);	
		//}

		//Assign
		if (tagToAssign == NULL) {//cannot find stable neighbors
			queFacesToRepair.push(facesInRegion);
		}
		else {
			for (std::set<Triangulation::Face_handle>::iterator curFace = facesInRegion.begin(); 
				curFace != facesInRegion.end(); ++curFace)
				(*curFace)->info().substituteTagsWith(tagToAssign);
		}
	}

	return true;
}*/

PolygonHandle* IOWorker::GetTopPriorityTag(Triangulation::Face_handle f, const std::vector<int>& priorList)
{
	PolygonHandle* result = NULL;
	if (f->info().hasNoTags()) return result;

	if(!f->info().getTags()->isMultiPolygonHandle()) 
		return f->info().getTags();

	MultiPolygonHandle* polyTag = static_cast<MultiPolygonHandle *>(f->info().getTags());
	list<PolygonHandle*>::const_iterator itrTag;
	int field, priority, topPri=9999999;
	std::vector<int>::const_iterator itrField;

	for (itrTag=polyTag->getHandles()->begin(); itrTag!=polyTag->getHandles()->end(); itrTag++) {
		if (!(*itrTag)->hasSchemaField()) continue;
		field = (*itrTag)->getSchemaField()->getValueAsInt();
		itrField = std::find(priorList.begin(), priorList.end(), field);
		if (itrField==priorList.end()) continue;
		priority = itrField-priorList.begin();
		if (priority<topPri) {
			result = *itrTag;
			topPri = priority;
		}
	}

	return result;
}

PolygonHandle* IOWorker::GetNeibTopPriorityTag(Triangulation& triangulation, const std::set<Triangulation::Face_handle>& regionFaces, const std::vector<int>& priorList)
{
	std::set<Triangulation::Face_handle>::const_iterator itrF;
	PolygonHandle *resultTag= NULL, *temTag=NULL;
	int field, priority, topPri=9999999;
	std::vector<int>::const_iterator itrField;

	for (itrF=regionFaces.begin(); itrF!=regionFaces.end(); itrF++) {
		for (int j=0; j<3; j++)	{
			if(!triangulation.is_constrained(std::pair<Triangulation::Face_handle, int>(*itrF, j)))
				continue;
			Face_handle neibF = (*itrF)->neighbor(j);
			if (neibF->info().hasNoTags()) continue;
			temTag = GetTopPriorityTag(neibF, priorList);
			if (!temTag) continue;

			if (!temTag->hasSchemaField()) continue;
			field = temTag->getSchemaField()->getValueAsInt();
			itrField = std::find(priorList.begin(), priorList.end(), field);
			if (itrField==priorList.end()) continue;
			priority = itrField-priorList.begin();
			if (priority<topPri) {
				resultTag = temTag;
				topPri = priority;
			}
		}
	}

	return resultTag;
}

std::set<Triangulation::Face_handle> IOWorker::GetStableNeighbores(Triangulation& triangulation, const std::set<Triangulation::Face_handle>& regionFaces, const std::vector<int>& priorList)
{
	std::set<Triangulation::Face_handle> stabelNeibs;
	std::set<Triangulation::Face_handle>::const_iterator itrF;
	PolygonHandle *resultTag= NULL, *temTag=NULL;
	int field, priority, topPri=9999999;
	std::vector<int>::const_iterator itrField;

	for (itrF=regionFaces.begin(); itrF!=regionFaces.end(); itrF++) {
		for (int j=0; j<3; j++)	{
			if(!triangulation.is_constrained(std::pair<Triangulation::Face_handle, int>(*itrF, j)))
				continue;
			Face_handle neibF = (*itrF)->neighbor(j);
			if (neibF->info().numberOfTags() != 1) continue;
			temTag = neibF->info().getOneTag();
			if (!temTag) continue;

			if (!temTag->hasSchemaField()) continue;
			field = temTag->getSchemaField()->getValueAsInt();
			itrField = std::find(priorList.begin(), priorList.end(), field);
			if (itrField==priorList.end()) continue;

			stabelNeibs.insert(neibF);
		}
	}

	return stabelNeibs;
}

PolygonHandle* IOWorker::GetStableNeibTopPriTag(Triangulation& triangulation, const std::set<Triangulation::Face_handle>& regionFaces, const std::vector<int>& priorList)
{
	std::set<Triangulation::Face_handle>::const_iterator itrF;
	PolygonHandle *resultTag= NULL, *temTag=NULL;
	int field, priority, topPri=9999999;
	std::vector<int>::const_iterator itrField;

	for (itrF=regionFaces.begin(); itrF!=regionFaces.end(); itrF++) {
		for (int j=0; j<3; j++)	{
			if(!triangulation.is_constrained(std::pair<Triangulation::Face_handle, int>(*itrF, j)))
				continue;
			Face_handle neibF = (*itrF)->neighbor(j);
			if (regionFaces.count(neibF)) continue;//belonging to current region
			if (neibF->info().numberOfTags() != 1) continue;//stable face should only has one tag
			temTag = neibF->info().getOneTag();
			if (!temTag) continue;

			if (!temTag->hasSchemaField()) continue;
			field = temTag->getSchemaField()->getValueAsInt();
			itrField = std::find(priorList.begin(), priorList.end(), field);
			if (itrField==priorList.end() || itrField==priorList.end()-1) continue;//the tag should be in the priority list
			priority = itrField-priorList.begin();
			
			if (priority<topPri) {
				resultTag = temTag;
				topPri = priority;
			}
		}
	}

	return resultTag;
}

#include <CGAL/squared_distance_2.h>
#include <CGAL/number_utils.h>
PolygonHandle* IOWorker::GetPriTagByStableNeibLongestBorder(Triangulation& triangulation, 
	const std::set<Triangulation::Face_handle>& regionFaces, const std::vector<int>& priorList, double big2SecondRatio)
{
	std::set<Triangulation::Face_handle>::const_iterator itrF;
	PolygonHandle *resultTag= NULL, *temTag=NULL;
	int field;
	std::vector<int>::const_iterator itrField;
	std::map<PolygonHandle*, double> mapRegionLength;
	

	for (itrF=regionFaces.begin(); itrF!=regionFaces.end(); itrF++) {
		for (int j=0; j<3; j++)	{
			if(!triangulation.is_constrained(std::pair<Triangulation::Face_handle, int>(*itrF, j)))
				continue;
			Face_handle neibF = (*itrF)->neighbor(j);
			if (regionFaces.count(neibF)) continue;//belonging to current region
			if (neibF->info().numberOfTags() != 1) continue;//stable face should only has one tag
			temTag = neibF->info().getOneTag();
			if (!temTag) continue;

			if (!temTag->hasSchemaField()) continue;
			field = temTag->getSchemaField()->getValueAsInt();
			itrField = std::find(priorList.begin(), priorList.end(), field);
			if (itrField==priorList.end() || itrField==priorList.end()-1) continue;//the tag should be in the priority list
			
			Triangulation::Vertex_handle v0=(*itrF)->vertex((*itrF)->ccw(j));
			Triangulation::Vertex_handle v1=(*itrF)->vertex((*itrF)->cw(j));
			double length = to_double(squared_distance(v0->point(),v1->point()));
			//double x0 = v0->point().x; 
			//double x1 = v1->point.x;
			//double y0 = v0->point.y; 
			//double y1 = v1->point.y;
			//double length = sqrt((x0-x1)*(x0-x1)+(y0-y1)*(y0-y1));

			mapRegionLength[temTag] += length;
		}
	}

	if (mapRegionLength.size()>=2) {
		vector<double> vecLens;
		for (std::map<PolygonHandle*, double>::iterator itr=mapRegionLength.begin(); 
			itr!=mapRegionLength.end(); itr++)
			vecLens.push_back(itr->second);

		std::sort(vecLens.begin(), vecLens.end());
		double ratio = vecLens[vecLens.size()-1]/vecLens[vecLens.size()-2] ;
		if (ratio > big2SecondRatio) {
			for (std::map<PolygonHandle*, double>::iterator itr=mapRegionLength.begin(); 
				itr!=mapRegionLength.end(); itr++) {
					if (itr->second == vecLens[vecLens.size()-1])
						resultTag = itr->first;
			}
		}
	}

	return resultTag;
}

PolygonHandle* IOWorker::GetStableNeibBottomPriTag(Triangulation& triangulation, const std::set<Triangulation::Face_handle>& regionFaces, const std::vector<int>& priorList)
{
	std::set<Triangulation::Face_handle>::const_iterator itrF;
	PolygonHandle *resultTag= NULL, *temTag=NULL;
	int field, priority, bottomPri=-1;
	std::vector<int>::const_iterator itrField;

	for (itrF=regionFaces.begin(); itrF!=regionFaces.end(); itrF++) {
		for (int j=0; j<3; j++)	{
			if(!triangulation.is_constrained(std::pair<Triangulation::Face_handle, int>(*itrF, j)))
				continue;
			Face_handle neibF = (*itrF)->neighbor(j);
			if (regionFaces.count(neibF)) continue;//belonging to current region
			if (neibF->info().numberOfTags() != 1) continue;//stable face should only has one tag
			temTag = neibF->info().getOneTag();
			if (!temTag) continue;

			if (!temTag->hasSchemaField()) continue;
			field = temTag->getSchemaField()->getValueAsInt();
			itrField = std::find(priorList.begin(), priorList.end(), field);
			if (itrField==priorList.end() || itrField==priorList.end()-1) continue;//the tag should be in the priority list
			priority = itrField-priorList.begin();

			if (priority>bottomPri) {
				resultTag = temTag;
				bottomPri = priority;
			}
		}
	}

	return resultTag;
}

bool IOWorker::repairByPriorityList(Triangulation &triangulation, const char *file) 
{
	// Process priority file
	if (!ReadPriorityMaph(file)) {
		std::cout << "Priority file could not be opened." << std::endl;
		return false;
	} 

	DoRepairByPriorityList(triangulation);
}

bool IOWorker::repairEdgeMatching(Triangulation &triangulation, const char *file) 
{
	// Process priority file
	std::ifstream priorityFile;
	priorityFile.open(file, std::ios::in);
	if (!priorityFile.is_open()) {
		std::cout << "Priority file could not be opened." << std::endl;
		return false;
	} std::map<Field *, unsigned int, FieldComparator> priorityMap;
	unsigned int currentPriority = 0;
	while (!priorityFile.eof()) {
		switch (schemaFieldType) {
			case OFTString: {
				std::string fieldAsString;
				std::getline(priorityFile, fieldAsString);		// If we deal with strings take a whole line (since spaces could be valid)
				StringField *newField = new StringField(fieldAsString.c_str());
				priorityMap[newField] = currentPriority;
				break;
			} case OFTReal: {
				double fieldAsDouble;
				priorityFile >> fieldAsDouble;
				DoubleField *newField = new DoubleField(fieldAsDouble);
				priorityMap[newField] = currentPriority;
			} case OFTInteger: {
				int fieldAsInt;
				priorityFile >> fieldAsInt;
				IntField *newField = new IntField(fieldAsInt);
				priorityMap[newField] = currentPriority;
			} default: {
				std::cout << "Field type not supported." << std::endl;
				std::string fieldAsString;
				std::getline(priorityFile, fieldAsString);
				break;
			}
		} ++currentPriority;
	} priorityFile.close();
	
	// Use a temporary vector to make it deterministic and order independent
	std::vector<std::pair<Triangulation::Face_handle, PolygonHandle *> > facesToRepair;
	std::set<Triangulation::Face_handle> processedFaces;
	for (Triangulation::Finite_faces_iterator currentFace = triangulation.finite_faces_begin(); currentFace != triangulation.finite_faces_end(); ++currentFace) {
		if (!currentFace->info().hasOneTag() && !processedFaces.count(currentFace)) {
			
			// Expand this triangle into a complete region
			std::set<Triangulation::Face_handle> facesInRegion;
			facesInRegion.insert(currentFace);
			std::stack<Triangulation::Face_handle> facesToProcess;
			facesToProcess.push(currentFace);
			while (facesToProcess.size() > 0) {
				Triangulation::Face_handle currentFaceInStack = facesToProcess.top();
				facesToProcess.pop();
				processedFaces.insert(currentFaceInStack);
				if (!currentFaceInStack->neighbor(0)->info().hasOneTag() && !facesInRegion.count(currentFaceInStack->neighbor(0)) && 
					!triangulation.is_constrained(std::pair<Triangulation::Face_handle, int>(currentFaceInStack, 0))) {
					facesInRegion.insert(currentFaceInStack->neighbor(0));
					facesToProcess.push(currentFaceInStack->neighbor(0));
				} if (!currentFaceInStack->neighbor(1)->info().hasOneTag() && !facesInRegion.count(currentFaceInStack->neighbor(1)) && 
					  !triangulation.is_constrained(std::pair<Triangulation::Face_handle, int>(currentFaceInStack, 1))) {
					facesInRegion.insert(currentFaceInStack->neighbor(1));
					facesToProcess.push(currentFaceInStack->neighbor(1));
				} if (!currentFaceInStack->neighbor(2)->info().hasOneTag() && !facesInRegion.count(currentFaceInStack->neighbor(2)) && 
					  !triangulation.is_constrained(std::pair<Triangulation::Face_handle, int>(currentFaceInStack, 2))) {
					facesInRegion.insert(currentFaceInStack->neighbor(2));
					facesToProcess.push(currentFaceInStack->neighbor(2));
				}
			}
			
			// Find the tag with the highest priority
			PolygonHandle *tagToAssign = NULL;
			unsigned int priorityOfTagh = UINT_MAX;
            unsigned int priorityOfTagg = 0;
			for (std::set<Triangulation::Face_handle>::iterator currentFaceInRegion = facesInRegion.begin(); currentFaceInRegion != facesInRegion.end(); ++currentFaceInRegion) {
				// Gap, check neighbours
				if ((*currentFaceInRegion)->info().hasNoTags()) {
					if (!(*currentFaceInRegion)->neighbor(0)->info().hasNoTags()) {
						if ((*currentFaceInRegion)->neighbor(0)->info().hasOneTag() && (*currentFaceInRegion)->neighbor(0)->info().getTags() != &universe) {
							if (priorityMap[(*currentFaceInRegion)->neighbor(0)->info().getTags()->getSchemaField()] >= priorityOfTagg) {
								priorityOfTagg = priorityMap[(*currentFaceInRegion)->neighbor(0)->info().getTags()->getSchemaField()];
								tagToAssign = (*currentFaceInRegion)->neighbor(0)->info().getTags();
							}
						} else {
							MultiPolygonHandle *handle = static_cast<MultiPolygonHandle *>((*currentFaceInRegion)->neighbor(0)->info().getTags());
							for (std::list<PolygonHandle *>::const_iterator currentTag = handle->getHandles()->begin(); currentTag != handle->getHandles()->end(); ++currentTag) {
								if (*currentTag == &universe) continue;
								if (priorityMap[(*currentTag)->getSchemaField()] < priorityOfTagg) {
									priorityOfTagg = priorityMap[(*currentTag)->getSchemaField()];
									tagToAssign = *currentTag;
								}
							}
						}
					} if (!(*currentFaceInRegion)->neighbor(1)->info().hasNoTags()) {
						if ((*currentFaceInRegion)->neighbor(1)->info().hasOneTag() && (*currentFaceInRegion)->neighbor(1)->info().getTags() != &universe) {
							if (priorityMap[(*currentFaceInRegion)->neighbor(1)->info().getTags()->getSchemaField()] >= priorityOfTagg) {
								priorityOfTagg = priorityMap[(*currentFaceInRegion)->neighbor(1)->info().getTags()->getSchemaField()];
								tagToAssign = (*currentFaceInRegion)->neighbor(1)->info().getTags();
							}
						} else {
							MultiPolygonHandle *handle = static_cast<MultiPolygonHandle *>((*currentFaceInRegion)->neighbor(1)->info().getTags());
							for (std::list<PolygonHandle *>::const_iterator currentTag = handle->getHandles()->begin(); currentTag != handle->getHandles()->end(); ++currentTag) {
								if (*currentTag == &universe) continue;
								if (priorityMap[(*currentTag)->getSchemaField()] < priorityOfTagg) {
									priorityOfTagg = priorityMap[(*currentTag)->getSchemaField()];
									tagToAssign = *currentTag;
								}
							}
						}
					} if (!(*currentFaceInRegion)->neighbor(2)->info().hasNoTags()) {
						if ((*currentFaceInRegion)->neighbor(2)->info().hasOneTag() && (*currentFaceInRegion)->neighbor(2)->info().getTags() != &universe) {
							if (priorityMap[(*currentFaceInRegion)->neighbor(2)->info().getTags()->getSchemaField()] < priorityOfTagg) {
								priorityOfTagg = priorityMap[(*currentFaceInRegion)->neighbor(2)->info().getTags()->getSchemaField()];
								tagToAssign = (*currentFaceInRegion)->neighbor(2)->info().getTags();
							}
						} else {
							MultiPolygonHandle *handle = static_cast<MultiPolygonHandle *>((*currentFaceInRegion)->neighbor(2)->info().getTags());
							for (std::list<PolygonHandle *>::const_iterator currentTag = handle->getHandles()->begin(); currentTag != handle->getHandles()->end(); ++currentTag) {
								if (*currentTag == &universe) continue;
								if (priorityMap[(*currentTag)->getSchemaField()] < priorityOfTagg) {
									priorityOfTagg = priorityMap[(*currentTag)->getSchemaField()];
									tagToAssign = *currentTag;
								}
							}
						}
					}
				}
				
				// Overlap, check this one
				else {
					if ((*currentFaceInRegion)->info().hasOneTag() && (*currentFaceInRegion)->info().getTags() != &universe) {
						if (priorityMap[(*currentFaceInRegion)->info().getTags()->getSchemaField()] < priorityOfTagh) {
							priorityOfTagh = priorityMap[(*currentFaceInRegion)->info().getTags()->getSchemaField()];
							tagToAssign = (*currentFaceInRegion)->info().getTags();
						}
					} else {
						MultiPolygonHandle *handle = static_cast<MultiPolygonHandle *>((*currentFaceInRegion)->info().getTags());
						for (std::list<PolygonHandle *>::const_iterator currentTag = handle->getHandles()->begin(); currentTag != handle->getHandles()->end(); ++currentTag) {
							if (*currentTag == &universe) continue;
							if (priorityMap[(*currentTag)->getSchemaField()] < priorityOfTagh) {
								priorityOfTagh = priorityMap[(*currentTag)->getSchemaField()];
								tagToAssign = *currentTag;
							}
						}
					}
				}
			}
			
			// Assign the tag to the triangles in the region
			for (std::set<Triangulation::Face_handle>::iterator currentFaceInRegion = facesInRegion.begin(); currentFaceInRegion != facesInRegion.end(); ++currentFaceInRegion) {
				facesToRepair.push_back(std::pair<Triangulation::Face_handle, PolygonHandle *>(*currentFaceInRegion, tagToAssign));
			}
		}
	}
	
	// Re-tag faces in the vector
	for (std::vector<std::pair<Triangulation::Face_handle, PolygonHandle *> >::iterator currentFace = facesToRepair.begin(); currentFace != facesToRepair.end(); ++currentFace) {
		currentFace->first->info().removeAllTags();
		currentFace->first->info().addTag(currentFace->second);
	}
	
	return true;
}

bool IOWorker::matchSchemata(Triangulation &triangulation) 
{
	std::map<Field *, PolygonHandle *, FieldComparator> fieldMatch;
	std::map<PolygonHandle *, PolygonHandle *> equivalencies;
	
	// Find equivalencies 
	for (std::vector<PolygonHandle *>::iterator currentPolygon = polygons.begin(); currentPolygon != polygons.end(); ++currentPolygon) {
		if (fieldMatch.count((*currentPolygon)->getSchemaField()) == 0) {
			fieldMatch[(*currentPolygon)->getSchemaField()] = *currentPolygon;
			equivalencies[*currentPolygon] = *currentPolygon;
		} else equivalencies[*currentPolygon] = fieldMatch[(*currentPolygon)->getSchemaField()];
	} std::cout << fieldMatch.size() << " classes found." << std::endl;
	
	// Re-tag
	for (Triangulation::Finite_faces_iterator currentFace = triangulation.finite_faces_begin(); currentFace != triangulation.finite_faces_end(); ++currentFace) {
		currentFace->info().substituteTagsWith(equivalencies[currentFace->info().getTags()]);
	}
	
	return true;
}

void IOWorker::removeConstraints(Triangulation &triangulation) 
{
#ifdef _DEBUG
	exportTriangulation(triangulation, "debug1.dxf", true, false, false);
#endif
	// Remove constrained edges that have the same polygon on both sides
    unsigned long long int constrainedEdgesRemoved = 0;
    for (Triangulation::All_edges_iterator currentEdge = triangulation.all_edges_begin(); currentEdge != triangulation.all_edges_end(); ++currentEdge) {
        if (!triangulation.is_constrained(*currentEdge)) continue;
        if (currentEdge->first->info().getOneTag() == currentEdge->first->neighbor(currentEdge->second)->info().getOneTag()) {
            triangulation.remove_constrained_edge(currentEdge->first, currentEdge->second);
            ++constrainedEdgesRemoved;
        }
    } std::cout << "\tRemoved " << constrainedEdgesRemoved << " constrained edges" << std::endl;

#ifdef _DEBUG
	exportTriangulation(triangulation, "debug1.dxf", true, false, false);
#endif

	unsigned long long int constrainedEdgesAdded = 0;
	for (Triangulation::All_edges_iterator currentEdge = triangulation.all_edges_begin(); currentEdge != triangulation.all_edges_end(); ++currentEdge) {
		if (triangulation.is_constrained(*currentEdge)) continue;
		if (currentEdge->first->info().getOneTag() != currentEdge->first->neighbor(currentEdge->second)->info().getOneTag()) {
			triangulation.insert_constraint(currentEdge->first->vertex(currentEdge->first->cw(currentEdge->second)), currentEdge->first->vertex(currentEdge->first->ccw(currentEdge->second)));
			++constrainedEdgesAdded;
		}
	} std::cout << "\tRemoved " << constrainedEdgesRemoved << " constrained edges" << std::endl;
#ifdef _DEBUG
	exportTriangulation(triangulation, "debug1.dxf", true, false, false);
#endif
}

void IOWorker::removeVertices(Triangulation &triangulation) 
{
    // Remove unnecessary vertices completely surrounded by the same polygon
    // TODO: This can be optimised
    
    std::cout << "\tBefore: " << triangulation.number_of_faces() << " triangles in the triangulation" << std::endl;
    
    unsigned long long int surroundedVerticesRemoved = 0;
    Triangulation::Finite_vertices_iterator currentVertex = triangulation.finite_vertices_begin();
    while (currentVertex != triangulation.finite_vertices_end()) {
        if (triangulation.are_there_incident_constraints(currentVertex)) {
            ++currentVertex;
            continue;
        }
        
        Triangulation::Face_circulator firstFace = triangulation.incident_faces(currentVertex), currentFace = firstFace;
        ++currentFace;
        bool allEqual = true;
        while (currentFace != firstFace) {
            if (currentFace->info().getOneTag() != firstFace->info().getOneTag()) {
                allEqual = false;
                break;
            } ++currentFace;
        }
        
        if (allEqual) {
            Triangulation::Finite_vertices_iterator vertexToRemove = currentVertex;
            ++currentVertex;
            
            Point location = vertexToRemove->point();
            //Triangulation::Face_handle approximateLocation;
            PolygonHandle *tag = triangulation.incident_faces(vertexToRemove)->info().getOneTag();
            triangulation.remove(vertexToRemove);
            std::stack<Triangulation::Face_handle> stack;
            Triangulation::Face_handle emptyFace = triangulation.locate(location);
            stack.push(emptyFace);
            tagStack(stack, tag);
            
            ++surroundedVerticesRemoved;
        } else {
            ++currentVertex;
        }
    } std::cout << "\tRemoved " << surroundedVerticesRemoved << " surrounded vertices" << std::endl;
    
    std::cout << "\tAfter: " << triangulation.number_of_faces() << " triangles in the triangulation" << std::endl;
}

/*
bool IOWorker::reconstructPolygons(Triangulation &triangulation, std::vector<std::pair<PolygonHandle *, Polygon> > &outputPolygons) 
{
	for (Triangulation::Finite_faces_iterator seedingFace = triangulation.finite_faces_begin(); seedingFace != triangulation.finite_faces_end(); ++seedingFace) 
	{
        PolygonHandle *currentTag = seedingFace->info().getOneTag();
        if (currentTag == NULL) continue;
        
        // STEP 1: Find a suitable seeding triangle (connected to the outer boundary)
		if (currentTag == &universe) {
			seedingFace->info().removeAllTags();
			continue;
		} 
		
		//STEP 2:
		//find all vertexes on border
		std::set<Triangulation::Face_handle> region = RegionGrow(triangulation, Triangulation::Face_handle(seedingFace));
		std::set<Triangulation::Vertex_handle> vertices;
		for (set<Face_handle>::iterator itrF=region.begin(); itrF!=region.end(); itrF++) {
			for (int iV=0; iV<3; iV++ )	{
				if((*itrF)->neighbor(iV)->info().hasTag(currentTag)) continue;//only store border edges
				vertices.insert((*itrF)->vertex((*itrF)->cw(iV)));
				vertices.insert((*itrF)->vertex((*itrF)->ccw(iV)));
			}
		}
		for (set<Face_handle>::iterator itrF=region.begin(); itrF!=region.end(); itrF++) 
			(*itrF)->info().removeAllTags();
        
		//STEP 3:
		//sort the border vertexes into ring
		std::vector<Vertex_handle> polygon;
		std::stack<Vertex_handle> starckVs;
		std::set<Vertex_handle> visitedVs;
		polygon.push_back(*(vertices.begin()));
		starckVs.push(*(vertices.begin()));
		Vertex_handle curV;
		while(!starckVs.empty()) {
			curV = starckVs.top();
			starckVs.pop();
			
			for (set<Vertex_handle>::iterator itrV1=vertices.begin(); itrV1!=vertices.end(); itrV1++) {
				//already have this vertex
				if (std::find(polygon.begin(), polygon.end(), *itrV1)!=polygon.end()) continue;
				//NOT a edge in the triangulation
				if(!triangulation.is_edge(curV,*itrV1)) continue;
				polygon.push_back(*itrV1);
				starckVs.push(*itrV1);
			}
		}

        // STEP 2: Get boundary
        seedingFace->info().removeAllTags();
        std::list<Triangulation::Vertex_handle> vertices;
        if (seedingFace->neighbor(2)->info().hasTag(currentTag)) {
			seedingFace->neighbor(2)->info().removeAllTags();
			std::list<Triangulation::Vertex_handle> *l2 = getBoundary(seedingFace->neighbor(2), seedingFace->neighbor(2)->index(seedingFace), currentTag);
            vertices.splice(vertices.end(), *l2);
			delete l2;
		} 
		vertices.push_back(seedingFace->vertex(0));
		
		if (seedingFace->neighbor(1)->info().hasTag(currentTag)) {
			seedingFace->neighbor(1)->info().removeAllTags();
			std::list<Triangulation::Vertex_handle> *l1 = getBoundary(seedingFace->neighbor(1), seedingFace->neighbor(1)->index(seedingFace), currentTag);
            vertices.splice(vertices.end(), *l1);
			delete l1;
		}
		vertices.push_back(seedingFace->vertex(2));
		
		if (seedingFace->neighbor(0)->info().hasTag(currentTag)) {
			seedingFace->neighbor(0)->info().removeAllTags();
			std::list<Triangulation::Vertex_handle> *l0 = getBoundary(seedingFace->neighbor(0), seedingFace->neighbor(0)->index(seedingFace), currentTag);
            vertices.splice(vertices.end(), *l0);
			delete l0;
		}
		vertices.push_back(seedingFace->vertex(1));
        
        // STEP 3: Find cutting vertices
        std::set<Triangulation::Vertex_handle> visitedVertices;
        std::set<Triangulation::Vertex_handle> repeatedVertices;
        for (std::list<Triangulation::Vertex_handle>::iterator currentVertex = vertices.begin(); currentVertex != vertices.end(); ++currentVertex) {
            if (!visitedVertices.insert(*currentVertex).second) repeatedVertices.insert(*currentVertex);
        } visitedVertices.clear();
        
        // STEP 4: Cut and join rings in the correct order
        std::list<std::list<Triangulation::Vertex_handle> *> rings;
        std::stack<std::list<Triangulation::Vertex_handle> *> chainsStack;
        std::map<Triangulation::Vertex_handle, std::list<Triangulation::Vertex_handle> *> vertexChainMap;
        std::list<Triangulation::Vertex_handle> *newChain = new std::list<Triangulation::Vertex_handle>();
        
        // New vertex
        for (std::list<Triangulation::Vertex_handle>::iterator currentVertex = vertices.begin(); currentVertex != vertices.end(); ++currentVertex) {
            // New chain
            if (repeatedVertices.count(*currentVertex) > 0) {
                // Closed by itself
				if(newChain->empty()) 
					delete newChain;
                else if (newChain->front() == *currentVertex) {
                    // Degenerate (insufficient vertices to be valid)
                    if ( newChain->size() < 3) delete newChain;
                    else {
                        std::list<Triangulation::Vertex_handle>::iterator secondElement = newChain->begin();
                        ++secondElement;
                        // Degenerate (zero area)
                        if (newChain->back() == *secondElement) delete newChain;
                        // Valid
                        else rings.push_back(newChain);
                    }
                }
                // Open by itself
                else {
                    // Closed with others in stack
                    if (vertexChainMap.count(*currentVertex)) {
                        while (chainsStack.top() != vertexChainMap[*currentVertex]) {
                            newChain->splice(newChain->begin(), *chainsStack.top());
                            chainsStack.pop();
                        } newChain->splice(newChain->begin(), *chainsStack.top());
                        chainsStack.pop();
                        vertexChainMap.erase(*currentVertex);
                        // Degenerate (insufficient vertices to be valid)
                        if (newChain->size() < 3) delete newChain;
                        else {
                            std::list<Triangulation::Vertex_handle>::iterator secondElement = newChain->begin();
                            ++secondElement;
                            // Degenerate (zero area)
                            if (newChain->back() == *secondElement) delete newChain;
                            // Valid
                            else rings.push_back(newChain);
                        }
                    }
                    // Open
                    else {
                        // Not first chain
                        if (repeatedVertices.count(newChain->front()) > 0) vertexChainMap[newChain->front()] = newChain;
                        chainsStack.push(newChain);
                    }
                } 
				newChain = new std::list<Triangulation::Vertex_handle>();
            } 
			
			newChain->push_back(*currentVertex);
        }
        
        // Final ring
        while (chainsStack.size() > 0) {
            newChain->splice(newChain->begin(), *chainsStack.top());
            chainsStack.pop();
        }
        
        // Degenerate (insufficient vertices to be valid)
        if (newChain->size() < 3) {
            delete newChain;
        } else {
            std::list<Triangulation::Vertex_handle>::iterator secondElement = newChain->begin();
            ++secondElement;
            // Degenerate (zero area)
            if (newChain->back() == *secondElement) delete newChain;
            // Valid
            else rings.push_back(newChain);
        }
        
        if (chainsStack.size() > 0) std::cout << "Error: Stack has " << chainsStack.size() << " elements. Should be empty." << std::endl;
        
        // STEP 5: Make a polygon from this list and save it
        std::vector<Ring> innerRings;
        Ring outerRing;
        for (std::list<std::list<Triangulation::Vertex_handle> *>::iterator currentRing = rings.begin(); currentRing != rings.end(); ++currentRing) {
            Ring newRing;
            for (std::list<Triangulation::Vertex_handle>::iterator currentPoint = (*currentRing)->begin(); currentPoint != (*currentRing)->end(); ++currentPoint) {
                newRing.push_back((*currentPoint)->point());
            } 
			
			if (newRing.is_clockwise_oriented()) outerRing = newRing;
            else innerRings.push_back(newRing);
        } 
		outputPolygons.push_back(std::pair<PolygonHandle *, Polygon>(currentTag, Polygon(outerRing, innerRings.begin(), innerRings.end())));
       
		// Free memory from the chains
        for (std::list<std::list<Triangulation::Vertex_handle> *>::iterator currentRing = rings.begin(); currentRing != rings.end(); ++currentRing) {
            delete *currentRing;
        }
    }
	
	return true;
}*/


bool IOWorker::reconstructPolygons(Triangulation &triangulation, std::vector<std::pair<PolygonHandle *, Polygon> > &outputPolygons) 
{
	std::set<Face_handle>* visitedFaces = new std::set<Face_handle>();

	for (Triangulation::Finite_faces_iterator seedingFace = triangulation.finite_faces_begin(); seedingFace != triangulation.finite_faces_end(); ++seedingFace) 
	{

#ifdef _DEBUG
		//if (DebugTriangle(seedingFace, 155001.78,461754.73,155001.66,461754.71,155001.68,461755.18)) {
		//	int aaa = 0;
		//}
		if (DebugTriangle(seedingFace, 155000.97,461754.00,155001.66,461754.71,155001.68,461755.18)) {
			int aaa = 0;
		}
		//exportTriangulation(triangulation, "debug1.dxf", true,false, false);
#endif
		if(visitedFaces->count(seedingFace)) continue;
		PolygonHandle *currentTag = seedingFace->info().getOneTag();
		Triangulation::Face_handle neibFace;
		if (currentTag == NULL) continue;

		// STEP 1: Find a suitable seeding triangle (connected to the outer boundary)
		if (currentTag == &universe) {
			//seedingFace->info().removeAllTags();
			continue;
		} 
		if (seedingFace->neighbor(0)->info().getOneTag() == currentTag &&
			seedingFace->neighbor(1)->info().getOneTag() == currentTag &&
			seedingFace->neighbor(2)->info().getOneTag() == currentTag) continue;

		// STEP 2: Get boundary
		//seedingFace->info().removeAllTags();
		visitedFaces->insert(seedingFace);

		std::list<Triangulation::Vertex_handle> vertices;
		neibFace = seedingFace->neighbor(2);
		if (neibFace->info().hasTag(currentTag)&&!visitedFaces->count(neibFace)) {
			//neibFace->info().removeAllTags();
			visitedFaces->insert(neibFace);
			std::list<Triangulation::Vertex_handle> *l2 = getBoundary(neibFace, 
				neibFace->index(seedingFace), currentTag, visitedFaces);
			vertices.splice(vertices.end(), *l2);
			delete l2;
		} 
		vertices.push_back(seedingFace->vertex(0));

		neibFace = seedingFace->neighbor(1);
		if (neibFace->info().hasTag(currentTag)&&!visitedFaces->count(neibFace)) {
			//neibFace->info().removeAllTags();
			visitedFaces->insert(neibFace);
			std::list<Triangulation::Vertex_handle> *l1 = getBoundary(neibFace, 
				neibFace->index(seedingFace), currentTag, visitedFaces);
			vertices.splice(vertices.end(), *l1);
			delete l1;
		}
		vertices.push_back(seedingFace->vertex(2));

		neibFace = seedingFace->neighbor(0);
		if (neibFace->info().hasTag(currentTag) &&!visitedFaces->count(neibFace)) {
			//neibFace->info().removeAllTags();
			visitedFaces->insert(neibFace);
			std::list<Triangulation::Vertex_handle> *l0 = getBoundary(neibFace, 
				neibFace->index(seedingFace), currentTag, visitedFaces);
			vertices.splice(vertices.end(), *l0);
			delete l0;
		}
		vertices.push_back(seedingFace->vertex(1));

		// STEP 3: Find cutting vertices
		std::set<Triangulation::Vertex_handle> visitedVertices;
		std::set<Triangulation::Vertex_handle> repeatedVertices;
		for (std::list<Triangulation::Vertex_handle>::iterator currentVertex = vertices.begin(); currentVertex != vertices.end(); ++currentVertex) {
			if (!visitedVertices.insert(*currentVertex).second) repeatedVertices.insert(*currentVertex);
		} visitedVertices.clear();

		// STEP 4: Cut and join rings in the correct order
		std::list<std::list<Triangulation::Vertex_handle> *> rings;
		std::stack<std::list<Triangulation::Vertex_handle> *> chainsStack;
		std::map<Triangulation::Vertex_handle, std::list<Triangulation::Vertex_handle> *> vertexChainMap;
		std::list<Triangulation::Vertex_handle> *newChain = new std::list<Triangulation::Vertex_handle>();

		// New vertex
		for (std::list<Triangulation::Vertex_handle>::iterator currentVertex = vertices.begin(); currentVertex != vertices.end(); ++currentVertex) 
		{
			// New chain
			if (repeatedVertices.count(*currentVertex) > 0) {
				// Closed by itself
				if(newChain->empty()) 
					delete newChain;
				else if (newChain->front() == *currentVertex) {
					// Degenerate (insufficient vertices to be valid)
					if ( newChain->size() < 3) delete newChain;
					else {
						std::list<Triangulation::Vertex_handle>::iterator secondElement = newChain->begin();
						++secondElement;
						// Degenerate (zero area)
						if (newChain->back() == *secondElement) delete newChain;
						// Valid
						else rings.push_back(newChain);
					}
				}
				// Open by itself
				else {
					// Closed with others in stack
					if (vertexChainMap.count(*currentVertex)) {
						while (chainsStack.top() != vertexChainMap[*currentVertex]) {
							newChain->splice(newChain->begin(), *chainsStack.top());
							chainsStack.pop();
						} newChain->splice(newChain->begin(), *chainsStack.top());
						chainsStack.pop();
						vertexChainMap.erase(*currentVertex);
						// Degenerate (insufficient vertices to be valid)
						if (newChain->size() < 3) delete newChain;
						else {
							std::list<Triangulation::Vertex_handle>::iterator secondElement = newChain->begin();
							++secondElement;
							// Degenerate (zero area)
							if (newChain->back() == *secondElement) delete newChain;
							// Valid
							else rings.push_back(newChain);
						}
					}
					// Open
					else {
						// Not first chain
						if (repeatedVertices.count(newChain->front()) > 0) vertexChainMap[newChain->front()] = newChain;
						chainsStack.push(newChain);
					}
				} 
				newChain = new std::list<Triangulation::Vertex_handle>();
			} 

			newChain->push_back(*currentVertex);
		}

		// Final ring
		while (chainsStack.size() > 0) {
			newChain->splice(newChain->begin(), *chainsStack.top());
			chainsStack.pop();
		}

		// Degenerate (insufficient vertices to be valid)
		if (newChain->size() < 3) {
			delete newChain;
		} else {
			std::list<Triangulation::Vertex_handle>::iterator secondElement = newChain->begin();
			++secondElement;
			// Degenerate (zero area)
			if (newChain->back() == *secondElement) delete newChain;
			// Valid
			else rings.push_back(newChain);
		}

		if (chainsStack.size() > 0) std::cout << "Error: Stack has " << chainsStack.size() << " elements. Should be empty." << std::endl;

		// STEP 5: Make a polygon from this list and save it
		std::vector<Ring> innerRings;
		Ring outerRing;
		for (std::list<std::list<Triangulation::Vertex_handle> *>::iterator currentRing = rings.begin(); currentRing != rings.end(); ++currentRing) {
			Ring newRing;
			for (std::list<Triangulation::Vertex_handle>::iterator currentPoint = (*currentRing)->begin(); currentPoint != (*currentRing)->end(); ++currentPoint) {
				newRing.push_back((*currentPoint)->point());
			} 

			if (newRing.is_clockwise_oriented()) outerRing = newRing;
			else innerRings.push_back(newRing);
		} 
		outputPolygons.push_back(std::pair<PolygonHandle *, Polygon>(currentTag, Polygon(outerRing, innerRings.begin(), innerRings.end())));

		// Free memory from the chains
		for (std::list<std::list<Triangulation::Vertex_handle> *>::iterator currentRing = rings.begin(); currentRing != rings.end(); ++currentRing) {
			delete *currentRing;
		}
	}
	delete visitedFaces;

	return true;
}

bool IOWorker::exportPolygons(std::vector<std::pair<PolygonHandle *, Polygon> > &outputPolygons, const char *file, bool withProvenance) 
{
	// Prepare file
	const char *driverName = "ESRI Shapefile";
	OGRSFDriver *driver = OGRSFDriverRegistrar::GetRegistrar()->GetDriverByName(driverName);
	if (driver == NULL) {
		std::cout << "\tError: OGR Shapefile driver not found." << std::endl;
		return false;
	}
	
	OGRDataSource *dataSource = driver->Open(file, false);
	if (dataSource != NULL) {
		std::cout << "\tOverwriting file..." << std::endl;
		if (driver->DeleteDataSource(dataSource->GetName())!= OGRERR_NONE) {
			std::cout << "\tError: Couldn't erase file with same name." << std::endl;
			return false;
		} OGRDataSource::DestroyDataSource(dataSource);
	}
	
	std::cout << "\tWriting file... " << std::endl;
	dataSource = driver->CreateDataSource(file, NULL);
	if (dataSource == NULL) {
		std::cout << "\tError: Could not create file." << std::endl;
		return false;
	}
	
	OGRLayer *layer = dataSource->CreateLayer("polygons", NULL, wkbPolygon, NULL);
	if (layer == NULL) {
		std::cout << "\tError: Could not create layer." << std::endl;
		return false;
	}
	
	// Set up the fields that there will be
	if (withProvenance) {
		unsigned int longest = 0;
		for (std::vector<char *>::iterator currentFileName = fileNames.begin(); currentFileName != fileNames.end(); ++currentFileName) {
			if (strlen(*currentFileName) > longest) longest = strlen(*currentFileName);
		} OGRFieldDefn filenameField("File", OFTString);
		filenameField.SetWidth(longest);
		if (layer->CreateField(&filenameField) != OGRERR_NONE) {
			std::cout << "\tError: Could not create field File." << std::endl;
			return false;
		} OGRFieldDefn layerField("Layer", OFTInteger);
		if (layer->CreateField(&layerField) != OGRERR_NONE) {
			std::cout << "\tError: Could not create field Layer." << std::endl;
			return false;
		}
	}
	
	for (std::vector<FieldDefinition *>::iterator currentField = fields.begin(); currentField != fields.end(); ++currentField) {
		OGRFieldDefn newField((*currentField)->name, (*currentField)->type);
		newField.SetJustify((*currentField)->justification);
		newField.SetWidth((*currentField)->width);
		newField.SetPrecision((*currentField)->precision);
		if (layer->CreateField(&newField) != OGRERR_NONE) {
			std::cout << "\tError: Could not create field " << (*currentField)->name << "." << std::endl;
			return false;
		}
	}
	
	// Put fields in
	for (std::vector<std::pair<PolygonHandle *, Polygon> >::iterator currentPolygon = outputPolygons.begin(); currentPolygon != outputPolygons.end(); ++currentPolygon) {
		OGRPolygon polygon;
		OGRLinearRing outerRing;
        if (currentPolygon->second.outer_boundary().size() < 1) continue;
		for (Ring::Vertex_iterator currentVertex = currentPolygon->second.outer_boundary().vertices_begin();
			 currentVertex != currentPolygon->second.outer_boundary().vertices_end();
			 ++currentVertex) {
			outerRing.addPoint(CGAL::to_double(currentVertex->x()), CGAL::to_double(currentVertex->y()));
		} outerRing.addPoint(CGAL::to_double(currentPolygon->second.outer_boundary().vertex(0).x()), CGAL::to_double(currentPolygon->second.outer_boundary().vertex(0).y()));
		polygon.addRing(&outerRing);
		for (Polygon::Hole_const_iterator currentRing = currentPolygon->second.holes_begin(); currentRing != currentPolygon->second.holes_end(); ++currentRing) {
			OGRLinearRing innerRing;
			for (Ring::Vertex_iterator currentVertex = currentRing->vertices_begin(); currentVertex != currentRing->vertices_end(); ++currentVertex) {
				innerRing.addPoint(CGAL::to_double(currentVertex->x()), CGAL::to_double(currentVertex->y()));
			} innerRing.addPoint(CGAL::to_double(currentRing->vertex(0).x()), CGAL::to_double(currentRing->vertex(0).y()));
			polygon.addRing(&innerRing);
		} OGRFeature *feature = OGRFeature::CreateFeature(layer->GetLayerDefn());
		if (withProvenance) {
			feature->SetField("File", currentPolygon->first->getOriginalFile());
			feature->SetField("Layer", (int)currentPolygon->first->getLayer());
		} for (unsigned int currentField = 0; currentField < currentPolygon->first->getNumberOfFields(); currentField++) {
			switch (currentPolygon->first->getField(currentField)->getType()) {
				case OFTString:
					feature->SetField(fields[fieldEquivalencies[FieldDescriptor(currentPolygon->first->getOriginalFile(), currentPolygon->first->getLayer(), currentField)]]->name,
									  currentPolygon->first->getField(currentField)->getValueAsString());
					break;
				case OFTReal:
					feature->SetField(fields[fieldEquivalencies[FieldDescriptor(currentPolygon->first->getOriginalFile(), currentPolygon->first->getLayer(), currentField)]]->name,
									  currentPolygon->first->getField(currentField)->getValueAsDouble());
					break;
				case OFTInteger:
					feature->SetField(fields[fieldEquivalencies[FieldDescriptor(currentPolygon->first->getOriginalFile(), currentPolygon->first->getLayer(), currentField)]]->name,
									  currentPolygon->first->getField(currentField)->getValueAsInt());
					break;
				default:
					std::cout << "\tError: Type not implemented." << std::endl;
					break;
			}
		}
		
		// Put geometry in
		feature->SetGeometry(&polygon);
		
		// Create OGR feature
		if (layer->CreateFeature(feature) != OGRERR_NONE) std::cout << "\tError: Could not create feature." << std::endl;
		
		// Free OGR feature
		OGRFeature::DestroyFeature(feature);
	}
	
	// Free OGR data source
	OGRDataSource::DestroyDataSource(dataSource);
	
	return true;
}

bool IOWorker::exportPolygons(std::vector<std::pair<PolygonHandle *, Polygon> > &outputPolygons, ObjectPoints& outObjPts, LineTopologies& outTops) 
{
	outObjPts.clear();
	outTops.clear();

	ObjectPoint temObj;
	LineTopology temTop;
	int pntNum = 0;

	// Put fields in
	for (std::vector<std::pair<PolygonHandle *, Polygon> >::iterator currentPolygon = outputPolygons.begin(); currentPolygon != outputPolygons.end(); ++currentPolygon) {
		OGRPolygon polygon;
		OGRLinearRing outerRing;
		Ring ring = currentPolygon->second.outer_boundary();
		if (ring.size() < 1) continue;
		std::vector<Ring*> regionRings = splitRing(ring);
		temObj.Z() = (currentPolygon-outputPolygons.begin())*1.0;

		for (int i=0; i<regionRings.size(); i++) {
			Ring& temRing = *regionRings[i];
			temTop.clear();
			for (Ring::Vertex_iterator curV=temRing.vertices_begin(); curV!=temRing.vertices_end(); ++curV) {
				//outerRing.addPoint(CGAL::to_double(curV->x()), CGAL::to_double(curV->y()));
				temObj.X() = CGAL::to_double(curV->x());
				temObj.Y() = CGAL::to_double(curV->y());
				temObj.Number() = pntNum++;
				outObjPts.push_back(temObj);
				temTop.push_back(temObj.Number());
			}
			temTop.push_back(temTop[0]);
			temTop.Attribute(BuildingPartNumberTag) = currentPolygon->first->getSchemaField()->getValueAsInt();
			temTop.Number() = outTops.size();
			outTops.push_back(temTop);
		}
	}

	return true;
}

bool IOWorker::exportTriangulation(Triangulation &t, const char *file, bool withNumberOfTags, bool withFields, bool withProvenance) 
{
	// Prepare file
	const char *driverName = "ESRI Shapefile";
	OGRSFDriver *driver = OGRSFDriverRegistrar::GetRegistrar()->GetDriverByName(driverName);
	if (driver == NULL) {
		std::cout << "Driver not found." << std::endl;
		return false;
	}
	
	OGRDataSource *dataSource = driver->Open(file, false);
	if (dataSource != NULL) {
		std::cout << "Erasing current file..." << std::endl;
		if (driver->DeleteDataSource(dataSource->GetName())!= OGRERR_NONE) {
			std::cout << "Couldn't erase current file." << std::endl;
			return false;
		} OGRDataSource::DestroyDataSource(dataSource);
	}
	
	std::cout << "Writing file... " << std::endl;
	dataSource = driver->CreateDataSource(file, NULL);
	if (dataSource == NULL) {
		std::cout << "Could not create file." << std::endl;
		return false;
	}
	

	//////////////////////////////////////////////////////////////////////////
	//output triangles
	OGRLayer *layer = dataSource->CreateLayer("triangles", NULL, wkbPolygon, NULL);
	if (layer == NULL) {
		std::cout << "Could not create layer." << std::endl;
		return false;
	}
	
	// Set up the fields that there will be
	if (withNumberOfTags) {
		OGRFieldDefn numberOfTagsField("Tags", OFTInteger);
		if (layer->CreateField(&numberOfTagsField) != OGRERR_NONE) {
			std::cout << "Could not create field Tags." << std::endl;
			return false;
		}
		
		/*OGRFieldDefn TagField = OGRFieldDefn ("TagCount", OFTInteger);
		if (layer->CreateField(&TagField) != OGRERR_NONE) {
			std::cout << "Could not create field TagCount." << std::endl;
			return false;
		}*/
	}
	
	if (withProvenance) {
		unsigned int longest = 0;
		for (std::vector<char *>::iterator currentFileName = fileNames.begin(); currentFileName != fileNames.end(); ++currentFileName) {
			if (strlen(*currentFileName) > longest) longest = strlen(*currentFileName);
		} OGRFieldDefn filenameField("File", OFTString);
		filenameField.SetWidth(longest);
		if (layer->CreateField(&filenameField) != OGRERR_NONE) {
			std::cout << "Could not create field Filename." << std::endl;
			return false;
		} OGRFieldDefn layerField("Layer", OFTInteger);
		if (layer->CreateField(&layerField) != OGRERR_NONE) {
			std::cout << "Could not create field Layer." << std::endl;
			return false;
		}
	}
	
	if (withFields) {
		for (std::vector<FieldDefinition *>::iterator currentField = fields.begin(); currentField != fields.end(); ++currentField) {
			OGRFieldDefn newField((*currentField)->name, (*currentField)->type);
			newField.SetJustify((*currentField)->justification);
			newField.SetWidth((*currentField)->width);
			newField.SetPrecision((*currentField)->precision);
			if (layer->CreateField(&newField) != OGRERR_NONE) {
				std::cout << "Could not create field " << (*currentField)->name << "." << std::endl;
				return false;
			}
		}
	}
	
	// Put fields in
	for (CDT::Finite_faces_iterator currentFace = t.finite_faces_begin(); currentFace != t.finite_faces_end(); ++currentFace) 
	{
		OGRLinearRing ring;
		ring.addPoint(CGAL::to_double((*(*currentFace).vertex(0)).point().x()), CGAL::to_double((*(*currentFace).vertex(0)).point().y()), 0.0);
		ring.addPoint(CGAL::to_double((*(*currentFace).vertex(1)).point().x()), CGAL::to_double((*(*currentFace).vertex(1)).point().y()), 0.0);
		ring.addPoint(CGAL::to_double((*(*currentFace).vertex(2)).point().x()), CGAL::to_double((*(*currentFace).vertex(2)).point().y()), 0.0);
		ring.addPoint(CGAL::to_double((*(*currentFace).vertex(0)).point().x()), CGAL::to_double((*(*currentFace).vertex(0)).point().y()), 0.0);
		OGRPolygon polygon;
		polygon.addRing(&ring);
		OGRFeature *feature = OGRFeature::CreateFeature(layer->GetLayerDefn());
		if (withNumberOfTags) {
			// Put number of tags
			/*if ((*currentFace).info().getTags()==NULL) 
				feature->SetField("Tags", 0);
			else if ((*currentFace).info().hasTag(&universe)) 
				feature->SetField("Tags", -2);
			else 
				feature->SetField("Tags", (int)(*currentFace).info().numberOfTags());
			*/ 
			
			// Put number of tags, the universe doesn't count
			if ((*currentFace).info().getTags() == NULL) {
				feature->SetField("Tags", 0);
			} else if((*currentFace).info().hasTag(&universe)) {
				feature->SetField("Tags",  -2);
			} else {
				feature->SetField("Tags",  (*currentFace).info().getOneTag()->getSchemaField()->getValueAsInt());
			}
		} 
		
		if (withProvenance) {
            if ((*currentFace).info().getTags() == NULL) {
                feature->SetField("File", "");
                feature->SetField("Layer", -1);
            } else {
                feature->SetField("File", (*currentFace).info().getTags()->getOriginalFile());
                feature->SetField("Layer", (int)(*currentFace).info().getTags()->getLayer());
            }
		} if (withFields && (*currentFace).info().getTags() != NULL) for (unsigned int currentField = 0; currentField < (*currentFace).info().getTags()->getNumberOfFields(); currentField++) {
			switch ((*currentFace).info().getTags()->getField(currentField)->getType()) {
				case OFTString:
                    feature->SetField(fields[fieldEquivalencies[FieldDescriptor((*currentFace).info().getTags()->getOriginalFile(), (*currentFace).info().getTags()->getLayer(), currentField)]]->name,
                                      (*currentFace).info().getTags()->getField(currentField)->getValueAsString());
				case OFTReal:
                    feature->SetField(fields[fieldEquivalencies[FieldDescriptor((*currentFace).info().getTags()->getOriginalFile(), (*currentFace).info().getTags()->getLayer(), currentField)]]->name,
                                      (*currentFace).info().getTags()->getField(currentField)->getValueAsDouble());
				case OFTInteger:
                    feature->SetField(fields[fieldEquivalencies[FieldDescriptor((*currentFace).info().getTags()->getOriginalFile(), (*currentFace).info().getTags()->getLayer(), currentField)]]->name,
                                      (*currentFace).info().getTags()->getField(currentField)->getValueAsInt());
					break;
				default:
					std::cout << "Error: Type not implemented." << std::endl;
					break;
			}
            
		} 
		
		// Put geometry in
		feature->SetGeometry(&polygon);
		
		// Create OGR feature
		if (layer->CreateFeature(feature) != OGRERR_NONE) std::cout << "Could not create feature." << std::endl;
		
		// Free OGR feature
		OGRFeature::DestroyFeature(feature);
	}


	//////////////////////////////////////////////////////////////////////////
	//output finite vertexes
	layer = dataSource->CreateLayer("vertexes", NULL, wkbPoint, NULL);
	if (layer == NULL) {
		std::cout << "Could not create layer." << std::endl;
		return false;
	}

	for (CDT::Vertex_iterator v = t.finite_vertices_begin(); v != t.finite_vertices_end(); ++v) 
	{
		OGRPoint point(CGAL::to_double(v->point().x()), CGAL::to_double(v->point().y()), 0.0);
		OGRFeature *feature = OGRFeature::CreateFeature(layer->GetLayerDefn());
			
		// Put geometry in
		feature->SetGeometry(&point);
		// Create OGR feature
		if (layer->CreateFeature(feature) != OGRERR_NONE) 
			std::cout << "Could not create feature." << std::endl;
		// Free OGR feature
		OGRFeature::DestroyFeature(feature);
	}

	//////////////////////////////////////////////////////////////////////////
	//output constrained edges
	layer = dataSource->CreateLayer("constrains", NULL, wkbLineString, NULL);
	if (layer == NULL) {
		std::cout << "Could not create layer." << std::endl;
		return false;
	}

	for (CDT::Finite_edges_iterator e = t.finite_edges_begin(); e!=t.finite_edges_end(); ++e) 
	{
		if (!e->first->is_constrained(e->second)) continue;
		
		CDT::Vertex_handle v1 = e->first->vertex(e->first->ccw(e->second));
		CDT::Vertex_handle v2 = e->first->vertex(e->first->cw(e->second));
		OGRLineString edge;
		edge.addPoint(CGAL::to_double(v1->point().x()), CGAL::to_double(v1->point().y()), 0.0 );
		edge.addPoint(CGAL::to_double(v2->point().x()), CGAL::to_double(v2->point().y()), 0.0 );
		
		// Put geometry in
		OGRFeature *feature = OGRFeature::CreateFeature(layer->GetLayerDefn());
		feature->SetGeometry(&edge);
		// Create OGR feature
		if (layer->CreateFeature(feature) != OGRERR_NONE) 
			std::cout << "Could not create feature." << std::endl;
		// Free OGR feature
		OGRFeature::DestroyFeature(feature);
	}
	
	// Free OGR data source
	OGRDataSource::DestroyDataSource(dataSource);
	
	return true;
}

unsigned int IOWorker::removeDuplicateVertices(std::list<Point> &ring) 
{
	unsigned int removed = 0;
	ring.push_back(ring.front());
	std::list<Point>::iterator previousVertex = ring.begin();
	std::list<Point>::iterator nextVertex = previousVertex;
	++nextVertex;
	while (nextVertex != ring.end()) {
		if (*previousVertex == *nextVertex) {
			nextVertex = ring.erase(nextVertex);
			++removed;
		} else {
			++previousVertex;
			++nextVertex;
		}
	} ring.pop_back();
	return removed;
}

std::vector<Ring *> IOWorker::splitRing(Ring &ring) 
{
	std::vector<Ring *> outputRings;
	
	// STEP 1: Put the edges in a triangulation
	Triangulation ringTriangulation;
    startingSearchFaceInRing = Triangulation::Face_handle();
	for (Ring::Edge_const_iterator currentEdge = ring.edges_begin(); currentEdge != ring.edges_end(); ++currentEdge) {
		Triangulation::Vertex_handle source = ringTriangulation.insert(currentEdge->source(), startingSearchFaceInRing);
        startingSearchFaceInRing = ringTriangulation.incident_faces(source);
		Triangulation::Vertex_handle target = ringTriangulation.insert(currentEdge->target(), startingSearchFaceInRing);
		Triangulation::Face_handle correspondingFace;
		int correspondingVertex;
        // Remove identical degenerate edges
		if (ringTriangulation.is_edge(source, target, correspondingFace, correspondingVertex)) {
			if (ringTriangulation.is_constrained(std::pair<Triangulation::Face_handle, int>(correspondingFace, correspondingVertex))) {
                //std::cout << "Removing duplicate constraint <" << *source << ", " << *target << ">" << std::endl;
				ringTriangulation.remove_constraint(source, target);
				continue;
			}
		} //std::cout << "Inserting constraint <" << *source << ", " << *target << ">" << std::endl;
        ringTriangulation.insert_constraint(source, target);
        startingSearchFaceInRing = ringTriangulation.incident_faces(target);
	}
	
	// Free space of the old ring
	ring.clear();
	
	// STEP 2: Remove degenerate edges (not identical, so not caught during creation)
	for (Triangulation::Subconstraint_iterator currentEdge = ringTriangulation.subconstraints_begin();
		 currentEdge != ringTriangulation.subconstraints_end();
		 ++currentEdge) {
        //std::cout << "Checking subconstraint: <" << *(currentEdge->first.first) << ", " << *(currentEdge->first.second) << ">: " << ringTriangulation.number_of_enclosing_constraints(currentEdge->first.first, currentEdge->first.second) << " enclosing constraints." << std::endl;
		// Subconstraint_iterator has a weird return value...
		if (ringTriangulation.number_of_enclosing_constraints(currentEdge->first.first, currentEdge->first.second) % 2 == 0) {
			Triangulation::Face_handle f;
			int i;
			ringTriangulation.is_edge(currentEdge->first.first, currentEdge->first.second, f, i);
            if (ringTriangulation.is_constrained(std::pair<Triangulation::Face_handle, int>(f, i))) {
                //std::cout << "Removing constraint..." << std::endl;
                ringTriangulation.remove_constraint(currentEdge->first.first, currentEdge->first.second);
            } else {
                //std::cout << "Adding constraint..." << std::endl;
                ringTriangulation.insert_constraint(currentEdge->first.first, currentEdge->first.second);
            }
		}
	}
	
	// STEP 3: Tag triangles
	PolygonHandle interior, exterior;
	std::stack<Triangulation::Face_handle> interiorStack, exteriorStack;
	exteriorStack.push(ringTriangulation.infinite_face());
	tagStack(interiorStack, exteriorStack, &interior, &exterior);
	
	// STEP 4: Get chains representing boundaries
	Triangulation::Face_handle seedingFace;
	std::vector<std::list<Triangulation::Vertex_handle> *> verticesList;
    for (Triangulation::Finite_faces_iterator seedingFace = ringTriangulation.finite_faces_begin(); seedingFace != ringTriangulation.finite_faces_end(); ++seedingFace) {
		
        if (seedingFace->info().getTags() != &interior) continue;
        
        std::list<Triangulation::Vertex_handle> *vertices = new std::list<Triangulation::Vertex_handle>();
		seedingFace->info().setTags(NULL);
		
		if (seedingFace->neighbor(2)->info().getTags() == &interior) {
			seedingFace->neighbor(2)->info().setTags(NULL);
			std::list<Triangulation::Vertex_handle> *l2 = getBoundary(seedingFace->neighbor(2), seedingFace->neighbor(2)->index(seedingFace), &interior);
			vertices->splice(vertices->end(), *l2);
			delete l2;
		} vertices->push_back(seedingFace->vertex(0));
		if (seedingFace->neighbor(1)->info().getTags() == &interior) {
			seedingFace->neighbor(1)->info().setTags(NULL);
			std::list<Triangulation::Vertex_handle> *l1 = getBoundary(seedingFace->neighbor(1), seedingFace->neighbor(1)->index(seedingFace), &interior);
			vertices->splice(vertices->end(), *l1);
			delete l1;
		} vertices->push_back(seedingFace->vertex(2));
		if (seedingFace->neighbor(0)->info().getTags() == &interior) {
			seedingFace->neighbor(0)->info().setTags(NULL);
			std::list<Triangulation::Vertex_handle> *l0 = getBoundary(seedingFace->neighbor(0), seedingFace->neighbor(0)->index(seedingFace), &interior);
			vertices->splice(vertices->end(), *l0);
			delete l0;
		} vertices->push_back(seedingFace->vertex(1));

		verticesList.push_back(vertices);
	}
	
	// From now on, process each list...
	for (std::vector<std::list<Triangulation::Vertex_handle> *>::iterator currentVerticesList = verticesList.begin(); currentVerticesList != verticesList.end(); ++currentVerticesList) {
		
		// STEP 5: Find cutting vertices
		std::set<Triangulation::Vertex_handle> visitedVertices;
        std::set<Triangulation::Vertex_handle> repeatedVertices;
		for (std::list<Triangulation::Vertex_handle>::iterator currentVertex = (*currentVerticesList)->begin(); currentVertex != (*currentVerticesList)->end(); ++currentVertex) {
			if (!visitedVertices.insert(*currentVertex).second) repeatedVertices.insert(*currentVertex);
		} visitedVertices.clear();
		
		// STEP 6: Cut and join rings in the correct order
        std::list<std::list<Triangulation::Vertex_handle> *> rings;
        std::stack<std::list<Triangulation::Vertex_handle> *> chainsStack;
        std::map<Triangulation::Vertex_handle, std::list<Triangulation::Vertex_handle> *> vertexChainMap;
        std::list<Triangulation::Vertex_handle> *newChain = new std::list<Triangulation::Vertex_handle>();
        for (std::list<Triangulation::Vertex_handle>::iterator currentVertex = (*currentVerticesList)->begin(); currentVertex != (*currentVerticesList)->end(); ++currentVertex) {
            // New chain
            if (repeatedVertices.count(*currentVertex) > 0) {
                // Closed by itself
                if (newChain->front() == *currentVertex) {
                    // Degenerate (insufficient vertices to be valid)
                    if (newChain->size() < 3) delete newChain;
                    else {
                        std::list<Triangulation::Vertex_handle>::iterator secondElement = newChain->begin();
                        ++secondElement;
                        // Degenerate (zero area)
                        if (newChain->back() == *secondElement) delete newChain;
                        // Valid
                        else rings.push_back(newChain);
                    }
                }
                
                // Open by itself
                else {
                    // Closed with others in stack
                    if (vertexChainMap.count(*currentVertex)) {
                        while (chainsStack.top() != vertexChainMap[*currentVertex]) {
                            newChain->splice(newChain->begin(), *chainsStack.top());
                            chainsStack.pop();
                        } newChain->splice(newChain->begin(), *chainsStack.top());
                        chainsStack.pop();
                        vertexChainMap.erase(*currentVertex);
                        // Degenerate (insufficient vertices to be valid)
                        if (newChain->size() < 3) delete newChain;
                        else {
                            std::list<Triangulation::Vertex_handle>::iterator secondElement = newChain->begin();
                            ++secondElement;
                            // Degenerate (zero area)
                            if (newChain->back() == *secondElement) delete newChain;
                            // Valid
                            else rings.push_back(newChain);
                        }
                    }
                    
                    // Open
                    else {
                        // Not first chain
                        if (repeatedVertices.count(newChain->front()) > 0) vertexChainMap[newChain->front()] = newChain;
                        chainsStack.push(newChain);
                    }
                } newChain = new std::list<Triangulation::Vertex_handle>();
            } newChain->push_back(*currentVertex);
        }
        
        // Final ring
        while (chainsStack.size() > 0) {
            newChain->splice(newChain->begin(), *chainsStack.top());
            chainsStack.pop();
        }
        
        // Degenerate (insufficient vertices to be valid)
        if (newChain->size() < 3) delete newChain;
        else {
            std::list<Triangulation::Vertex_handle>::iterator secondElement = newChain->begin();
            ++secondElement;
            // Degenerate (zero area)
            if (newChain->back() == *secondElement) delete newChain;
            // Valid
            else rings.push_back(newChain);
        }
        
        // STEP 7: Make rings from these lists and save them
		for (std::list<std::list<Triangulation::Vertex_handle> *>::iterator currentRing = rings.begin(); currentRing != rings.end(); ++currentRing) {
			Ring *newRing = new Ring();
            for (std::list<Triangulation::Vertex_handle>::iterator currentVertex = (*currentRing)->begin(); currentVertex != (*currentRing)->end(); ++currentVertex) {
				newRing->push_back((*currentVertex)->point());
			} outputRings.push_back(newRing);
		}
		
		// Free memory from the chains
		for (std::list<std::list<Triangulation::Vertex_handle> *>::iterator currentRing = rings.begin(); currentRing != rings.end(); ++currentRing) delete *currentRing;
		
		// Free memory from the vertices lists
		delete *currentVerticesList;
	}
	
	// Free memory from the triangulation
	ringTriangulation.clear();
	
	//std::cout << "\tCreated " << outputRings.size() << " rings." << std::endl;
	return outputRings;
}

void IOWorker::testRings(std::vector<Ring *> &outerRings, std::vector<Ring *> &innerRings, std::vector<std::vector<Ring> > &classification, long fid) 
{
	Triangulation ringsTriangulation;
    startingSearchFaceInRing = Triangulation::Face_handle();
	std::vector<std::vector<Triangulation::Vertex_handle> > ringsToTag;
	std::vector<PolygonHandle *> tags;
	tags.reserve(outerRings.size());
	std::map<PolygonHandle *, unsigned int> tagsMap;
	
	// STEP 1: Put the edges from the outer rings in the triangulation and save them for tagging
	for (std::vector<Ring *>::iterator currentRing = outerRings.begin(); currentRing != outerRings.end(); ++currentRing) {
		ringsToTag.push_back(std::vector<Triangulation::Vertex_handle>());
		tags.push_back(new PolygonHandle());
		tagsMap[tags.back()] = tags.size()-1;
		for (Ring::Edge_const_iterator currentEdge = (*currentRing)->edges_begin(); currentEdge != (*currentRing)->edges_end(); ++currentEdge) {
			Triangulation::Vertex_handle source = ringsTriangulation.insert(currentEdge->source(), startingSearchFaceInRing);
            startingSearchFaceInRing = ringsTriangulation.incident_faces(source);
			Triangulation::Vertex_handle target = ringsTriangulation.insert(currentEdge->target(), startingSearchFaceInRing);
			ringsTriangulation.insert_constraint(source, target);
            startingSearchFaceInRing = ringsTriangulation.incident_faces(target);
			ringsToTag.back().push_back(source);
		} ringsToTag.back().push_back(ringsToTag.back().front());
	}
	
	// STEP 2: Tag triangles
	Triangulation::Face_handle currentFace;
    int incident;
	std::stack<Triangulation::Face_handle> stack;
	for (unsigned int currentRing = 0; currentRing < ringsToTag.size(); ++currentRing) {
		for (unsigned int currentVertex = 1; currentVertex < ringsToTag[currentRing].size(); ++currentVertex) {
			if (!ringsTriangulation.is_edge(ringsToTag[currentRing].at(currentVertex-1), ringsToTag[currentRing].at(currentVertex), currentFace, incident)) {
				std::cout << "\tError: Cannot find adjoining face to an edge from the edge list!" << std::endl;
			} stack.push(currentFace);
		} tagStack(stack, tags[currentRing]);
	}
	
	// Tag the universe
	stack.push(ringsTriangulation.infinite_face());
	tagStack(stack, &universe);
	
	// Free memory
	ringsToTag.clear();
	
	// STEP 3: Check where inner rings belong
	for (std::vector<Ring *>::iterator currentRing = innerRings.begin(); currentRing != innerRings.end(); ++currentRing) {
		std::set<unsigned int> addedTo;
		for (Ring::Edge_const_iterator currentEdge = (*currentRing)->edges_begin(); currentEdge != (*currentRing)->edges_end(); ++currentEdge) {
			Triangulation::Locate_type locateType;
			int locateIndex;
			Triangulation::Face_handle location = ringsTriangulation.locate(currentEdge->source(), locateType, locateIndex, startingSearchFaceInRing);
            startingSearchFaceInRing = location;
			if (locateType == Triangulation::FACE) {
				PolygonHandle *tag = location->info().getTags();
				if (tagsMap.count(tag) > 0) {
					unsigned int tagIndex = tagsMap[tag];
					if (addedTo.count(tagIndex) == 0) {
						addedTo.insert(tagIndex);
						classification[tagIndex].push_back(**currentRing);
					}
				} else {
					std::cout << "\tFeature #" << fid << ": inner boundary vertex outside outer boundary. Using other vertices..." << std::endl;
				}
			}
		} if (addedTo.size() < 1) std::cout << "\tFeature #" << fid << ": inner boundary cannot fit in any outer boundary. Skipped." << std::endl;
		else if (addedTo.size() > 1) std::cout << "\tFeature #" << fid << ": inner boundary fits in more than one OB. Added to all." << std::endl;
	}
}

void IOWorker::copyFields(OGRFeature *ogrfeature, PolygonHandle *handle) 
{
	if (!ogrfeature || !handle) return;
	
	Field *newField = NULL;
	for (int i = 0; i < ogrfeature->GetFieldCount(); i++) {
		switch (ogrfeature->GetFieldDefnRef(i)->GetType()) {
			case OFTString:
				newField = new StringField(ogrfeature->GetFieldAsString(i));
				break;
			case OFTReal:
				newField = new DoubleField(ogrfeature->GetFieldAsDouble(i));
				break;
			case OFTInteger:
				newField = new IntField(ogrfeature->GetFieldAsInteger(i));
				break;
			default:
				std::cout << "\tError: Field type not supported. Skipped." << std::endl; 
				continue;
				break;
		} 
		
		handle->addField(newField);
	}
}

void IOWorker::AddField( PolygonHandle *handle, int value)
{
	IntField* newField = new IntField(value);
	handle->addField(newField);
	  
}

void IOWorker::tagStack(std::stack<Triangulation::Face_handle> &positiveStack, std::stack<Triangulation::Face_handle> &negativeStack, PolygonHandle *positiveHandle, PolygonHandle *negativeHandle) 
{
    //std::cout << "tagStack() Infinite vertex at: "  << std::endl;
	while (!positiveStack.empty() || !negativeStack.empty()) {
        //std::cout << "positiveStack: " << positiveStack.size() << " negativeStack: " << negativeStack.size() << std::endl;
		if (positiveStack.empty()) {
			Triangulation::Face_handle currentFace = negativeStack.top();
            //std::cout << "Triangle: <" << *(currentFace->vertex(0)) << ", " << *(currentFace->vertex(1)) << ", " << *(currentFace->vertex(2)) << ">" << std::endl;
			negativeStack.pop();
			currentFace->info().setTags(negativeHandle);
			if (currentFace->is_constrained(0)) {
				if (currentFace->neighbor(0)->info().getTags() != positiveHandle) {
					currentFace->neighbor(0)->info().setTags(positiveHandle);
					positiveStack.push(currentFace->neighbor(0));
				}
			} else {
				if (currentFace->neighbor(0)->info().getTags() != negativeHandle) {
					currentFace->neighbor(0)->info().setTags(negativeHandle);
					negativeStack.push(currentFace->neighbor(0));
				}
			} if (currentFace->is_constrained(1)) {
				if (currentFace->neighbor(1)->info().getTags() != positiveHandle) {
					currentFace->neighbor(1)->info().setTags(positiveHandle);
					positiveStack.push(currentFace->neighbor(1));
				}
			} else {
				if (currentFace->neighbor(1)->info().getTags() != negativeHandle) {
					currentFace->neighbor(1)->info().setTags(negativeHandle);
					negativeStack.push(currentFace->neighbor(1));
				}
			} if (currentFace->is_constrained(2)) {
				if (currentFace->neighbor(2)->info().getTags() != positiveHandle) {
					currentFace->neighbor(2)->info().setTags(positiveHandle);
					positiveStack.push(currentFace->neighbor(2));
				}
			} else {
				if (currentFace->neighbor(2)->info().getTags() != negativeHandle) {
					currentFace->neighbor(2)->info().setTags(negativeHandle);
					negativeStack.push(currentFace->neighbor(2));
				}
			}
		} else {
			Triangulation::Face_handle currentFace = positiveStack.top();
            //std::cout << "Triangle: <" << *(currentFace->vertex(0)) << ", " << *(currentFace->vertex(1)) << ", " << *(currentFace->vertex(2)) << ">" << std::endl;
			positiveStack.pop();
			currentFace->info().setTags(positiveHandle);
			if (currentFace->is_constrained(0)) {
				if (currentFace->neighbor(0)->info().getTags() != negativeHandle) {
					currentFace->neighbor(0)->info().setTags(negativeHandle);
					negativeStack.push(currentFace->neighbor(0));
				}
			} else {
				if (currentFace->neighbor(0)->info().getTags() != positiveHandle) {
					currentFace->neighbor(0)->info().setTags(positiveHandle);
					positiveStack.push(currentFace->neighbor(0));
				}
			} if (currentFace->is_constrained(1)) {
				if (currentFace->neighbor(1)->info().getTags() != negativeHandle) {
					currentFace->neighbor(1)->info().setTags(negativeHandle);
					negativeStack.push(currentFace->neighbor(1));
				}
			} else {
				if (currentFace->neighbor(1)->info().getTags() != positiveHandle) {
					currentFace->neighbor(1)->info().setTags(positiveHandle);
					positiveStack.push(currentFace->neighbor(1));
				}
			} if (currentFace->is_constrained(2)) {
				if (currentFace->neighbor(2)->info().getTags() != negativeHandle) {
					currentFace->neighbor(2)->info().setTags(negativeHandle);
					negativeStack.push(currentFace->neighbor(2));
				}
			} else {
				if (currentFace->neighbor(2)->info().getTags() != positiveHandle) {
					currentFace->neighbor(2)->info().setTags(positiveHandle);
					positiveStack.push(currentFace->neighbor(2));
				}
			}
		}
	}
}

void IOWorker::tagStack(std::stack<Triangulation::Face_handle> &stack, PolygonHandle *handle) 
{
	Triangulation::Face_handle curFace, neibFace;

	while (!stack.empty()) {
		curFace = stack.top();
		stack.pop();
		if (!curFace->info().hasTag(handle))
			curFace->info().addTag(handle);

		neibFace = curFace->neighbor(0);
		if (!neibFace->info().hasTag(handle) && !curFace->is_constrained(0)) {
			neibFace->info().addTag(handle);
			stack.push(neibFace);
		}
		
		neibFace = curFace->neighbor(1);
		if (!neibFace->info().hasTag(handle) && !curFace->is_constrained(1)) {
			neibFace->info().addTag(handle);
			stack.push(neibFace);
		}
		
		neibFace = curFace->neighbor(2);
		if (!neibFace->info().hasTag(handle) && !curFace->is_constrained(2)) {
			neibFace->info().addTag(handle);
			stack.push(neibFace);
		}
	}
}

/*
std::list<Triangulation::Vertex_handle> * IOWorker::getBoundary(Triangulation::Face_handle face, int edge, PolygonHandle *polygon) 
{
    std::list<Triangulation::Vertex_handle> *vertices = new std::list<Triangulation::Vertex_handle>();
	
	// Check clockwise edge
	Triangulation::Face_handle neibF = face->neighbor(face->cw(edge));
	if (!face->is_constrained(face->cw(edge)) && !neibF->info().hasNoTags()) {
		neibF->info().removeAllTags();
		std::list<Triangulation::Vertex_handle> *v1 = getBoundary(neibF, neibF->index(face), polygon);
        vertices->splice(vertices->end(), *v1);
		delete v1;
	}
	
	// Add central vertex
	vertices->push_back(face->vertex(edge));
	
	// Check counterclockwise edge
	neibF = face->neighbor(face->ccw(edge));
	if (!face->is_constrained(face->ccw(edge)) && !neibF->info().hasNoTags()) {
		neibF->info().removeAllTags();
		std::list<Triangulation::Vertex_handle> *v2 = getBoundary(neibF, neibF->index(face), polygon);
        vertices->splice(vertices->end(), *v2);
		delete v2;
	}
	
	return vertices;
}*/

std::list<Triangulation::Vertex_handle> * IOWorker::getBoundary(Triangulation::Face_handle face, int edge, PolygonHandle *polygon, std::set<Face_handle>* visitedFaces) 
{
	std::list<Triangulation::Vertex_handle> *vertices = new std::list<Triangulation::Vertex_handle>();

	// Check clockwise edge
	Triangulation::Face_handle neibF = face->neighbor(face->cw(edge));
	if (neibF->info()==face->info()&&!visitedFaces->count(neibF)) {
		//neibF->info().removeAllTags();
		visitedFaces->insert(neibF);
		std::list<Triangulation::Vertex_handle> *v1 = getBoundary(neibF, neibF->index(face), polygon, visitedFaces);
		vertices->splice(vertices->end(), *v1);
		delete v1;
	}

	// Add central vertex
	vertices->push_back(face->vertex(edge));

	// Check counterclockwise edge
	neibF = face->neighbor(face->ccw(edge));
	if (neibF->info()==face->info()&&!visitedFaces->count(neibF)) {
		//neibF->info().removeAllTags();
		visitedFaces->insert(neibF);
		std::list<Triangulation::Vertex_handle> *v2 = getBoundary(neibF, neibF->index(face), polygon, visitedFaces);
		vertices->splice(vertices->end(), *v2);
		delete v2;
	}

	return vertices;
}

std::list<Triangulation::Vertex_handle> * IOWorker::getBoundary(Triangulation::Face_handle face, int edge, PolygonHandle *polygon) 
{
	std::list<Triangulation::Vertex_handle> *vertices = new std::list<Triangulation::Vertex_handle>();

	// Check clockwise edge
	if (!face->is_constrained(face->cw(edge)) && !face->neighbor(face->cw(edge))->info().hasNoTags()) {
		face->neighbor(face->cw(edge))->info().removeAllTags();
		std::list<Triangulation::Vertex_handle> *v1 = getBoundary(face->neighbor(face->cw(edge)), face->neighbor(face->cw(edge))->index(face), polygon);
		vertices->splice(vertices->end(), *v1);
		delete v1;
	}

	// Add central vertex
	vertices->push_back(face->vertex(edge));

	// Check counterclockwise edge
	if (!face->is_constrained(face->ccw(edge)) && !face->neighbor(face->ccw(edge))->info().hasNoTags()) {
		face->neighbor(face->ccw(edge))->info().removeAllTags();
		std::list<Triangulation::Vertex_handle> *v2 = getBoundary(face->neighbor(face->ccw(edge)), face->neighbor(face->ccw(edge))->index(face), polygon);
		vertices->splice(vertices->end(), *v2);
		delete v2;
	}

	return vertices;
}

void IOWorker::addtoCount(std::map<PolygonHandle *, unsigned int> &count, PolygonHandle *ph) 
{
	if (ph == NULL) return;
	if (ph->isMultiPolygonHandle()) {
		MultiPolygonHandle *mph = static_cast<MultiPolygonHandle *>(ph);
		for (std::list<PolygonHandle *>::const_iterator currentPolygonHandle = mph->getHandles()->begin(); currentPolygonHandle != mph->getHandles()->end(); ++currentPolygonHandle) {
			if (count.count(*currentPolygonHandle)) count[*currentPolygonHandle]++;
			else count[*currentPolygonHandle] = 1;
		}
	} else {
		if (count.count(ph)) count[ph]++;
		else count[ph] = 1;
	} 
}

void IOWorker::addToLength(std::map<PolygonHandle *, double> &lengths, PolygonHandle *ph, double length) 
{
	if (ph == NULL) return;
	if (ph->isMultiPolygonHandle()) {
		MultiPolygonHandle *mph = static_cast<MultiPolygonHandle *>(ph);
		for (std::list<PolygonHandle *>::const_iterator currentPolygonHandle = mph->getHandles()->begin(); currentPolygonHandle != mph->getHandles()->end(); ++currentPolygonHandle) {
			if (lengths.count(*currentPolygonHandle)) lengths[*currentPolygonHandle] += length;
			else lengths[*currentPolygonHandle] = length;
		}
	} else {
		if (lengths.count(ph)) lengths[ph] += length;
		else lengths[ph] = length;
	}
}

void IOWorker::insertToStream(std::ostream &ostr, OGRFeatureDefn *layerDefinition, unsigned int indentation, int schemaIndex) 
{
    ostr << std::endl;
	for (int currentField = 0; currentField < layerDefinition->GetFieldCount(); currentField++) {
		OGRFieldDefn *fieldDefinition = layerDefinition->GetFieldDefn(currentField);
        if (currentField == schemaIndex) ostr << ">";
		for (unsigned int i = 0; i <= indentation; i++) ostr << "\t";
		insertToStream(ostr, fieldDefinition->GetType());
		ostr << "\t" << fieldDefinition->GetNameRef() << std::endl;
	}
}

void IOWorker::insertToStream(std::ostream &ostr, const OGRFieldType &ft) 
{
	switch (ft) {
		case OFTInteger:
			ostr << "int        ";
			break;
		case OFTIntegerList:
			ostr << "int[]      ";
			break;
		case OFTReal:
			ostr << "double     ";
			break;
		case OFTRealList:
			ostr << "double[]   ";
			break;
		case OFTString:
			ostr << "string     ";
			break;
		case OFTStringList:
			ostr << "string[]   ";
			break;
		case OFTWideString:
			ostr << "deprecated ";
			break;
		case OFTWideStringList:
			ostr << "deprecated ";
			break;
		case OFTBinary:
			ostr << "binary data";
			break;
		case OFTDate:
			ostr << "date       ";
			break;
		case OFTTime:
			ostr << "time       ";
			break;
		case OFTDateTime:
			ostr << "date & time";
			break;
		default:
			ostr << "unknown    ";
			break;
	}
}

void IOWorker::insertToStream(std::ostream &ostr, const OGRwkbGeometryType &gt) 
{
	switch (gt) {
		case wkbUnknown:
			ostr << "unknown";
			break;
		case wkbPoint:
			ostr << "point";
			break;
		case wkbLineString:
			ostr << "line string";
			break;
		case wkbPolygon:
			ostr << "polygon";
			break;
		case wkbMultiPoint:
			ostr << "multi point";
			break;
		case wkbMultiLineString:
			ostr << "multi line string";
			break;
		case wkbMultiPolygon:
			ostr << "multi polygon";
			break;
		case wkbGeometryCollection:
			ostr << "geometry collection";
			break;
		case wkbNone:
			ostr << "none";
			break;
		case wkbLinearRing:
			ostr << "linear ring";
			break;
		case wkbPoint25D:
			ostr << "2.5D point";
			break;
		case wkbLineString25D:
			ostr << "2.5D line string";
			break;
		case wkbPolygon25D:
			ostr << "2.5D polygon";
			break;
		case wkbMultiPoint25D:
			ostr << "2.5D multi point";
			break;
		case wkbMultiLineString25D:
			ostr << "2.5D multi line string";
			break;
		case wkbMultiPolygon25D:
			ostr << "2.5D multi polygon";
			break;
		case wkbGeometryCollection25D:
			ostr << "2.5D geometry collection";
			break;
		default:
			ostr << "other";
			break;
	}
}

void IOWorker::insertTriangulationInfo(std::ostream &ostr, const Triangulation &t) 
{
	// Number of tags
	unsigned int untagged = 0, onetag = 0, multipletags = 0, total;
	for (Triangulation::Finite_faces_iterator currentFace = t.finite_faces_begin(); currentFace != t.finite_faces_end(); ++currentFace) {
		if ((*currentFace).info().hasNoTags()) untagged++;
		else if ((*currentFace).info().hasOneTag()) onetag++;
		else multipletags++;
	} total = onetag + multipletags + untagged;
    ostr << "\tHoles:    " << untagged << " triangles (" << 100.0*untagged/total << " %)" << std::endl << 
            "\tOk:       " << onetag << " triangles (" << 100.0*onetag/total << " %)" << std::endl << 
            "\tOverlaps: " << multipletags << " triangles (" << 100.0*multipletags/total << " %)" << std::endl;
	
	// Other info?
}
