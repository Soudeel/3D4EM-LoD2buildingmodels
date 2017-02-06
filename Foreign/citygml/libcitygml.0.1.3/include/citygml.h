/* -*-c++-*- libcitygml - Copyright (c) 2010 Joachim Pouderoux, BRGM
*
* This file is part of libcitygml library
* http://code.google.com/p/libcitygml
*
* libcitygml is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation, either version 2.1 of the License, or
* (at your option) any later version.
*
* libcitygml is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU Lesser General Public License for more details.
*/

#ifndef __CITYGML_H__
#define __CITYGML_H__

#include <iostream>
#include <string>
#include <vector>
#include <sstream>
#include <map>

#include <stdio.h>
#include <stdlib.h>

#define LIBCITYGML_VERSION_MAJOR 0
#define LIBCITYGML_VERSION_MINOR 1
#define LIBCITYGML_VERSION_REVISION 3

#define LIBCITYGML_VERSIONSTR "0.1.3"

#if defined( _MSC_VER ) && defined( LIBCITYGML_DYNAMIC )
#	ifdef LIBCITYGML_BUILD
#		define LIBCITYGML_EXPORT __declspec( dllexport )
#	else
#		define LIBCITYGML_EXPORT __declspec( dllimport )
#	endif
#else
#	define LIBCITYGML_EXPORT
#endif

#ifdef USE_ELKANO_TYPES
typedef elk::Vec3d TVec3d;
typedef elk::Vec3f TVec3f;
typedef elk::Vec2f TVec2f;
typedef elk::Vec4f TVec4f;
#else
#	include "vecs.h"
#endif


namespace citygml 
{
	class CityModel;

	typedef enum {
		COT_GenericCityObject           = 1 << 0,
		COT_Building                    = 1 << 1,
		COT_Room                        = 1 << 2,
		COT_BuildingInstallation        = 1 << 3,
		COT_BuildingFurniture           = 1 << 4,
		COT_Door                        = 1 << 5,
		COT_Window                      = 1 << 6,
		COT_CityFurniture               = 1 << 7,
		COT_Track                       = 1 << 8,
		COT_Road                        = 1 << 9,
		COT_Railway                     = 1 << 10,
		COT_Square                      = 1 << 11,
		COT_PlantCover                  = 1 << 12,
		COT_SolitaryVegetationObject    = 1 << 13,
		COT_WaterBody                   = 1 << 14,
		COT_TINRelief                   = 1 << 15,
		COT_LandUse                     = 1 << 16,
		COT_Tunnel						= 1 << 17,
		COT_Bridge						= 1 << 18,
		COT_BridgeConstructionElement	= 1 << 19,
		COT_BridgeInstallation			= 1 << 20, 
		COT_BridgePart                  = 1 << 21,
		COT_All                         = 0xFFFFFF
	} CityObjectsType;

	typedef unsigned int CityObjectsTypeMask;

	///////////////////////////////////////////////////////////////////////////////
	// Parsing routines

	// Parameters:
	// objectsMask: a string describing the objects types that must or must not be parsed
	//    examples: "All&~LandUse&~TINRelief" to parse everything but landuses and TIN reliefs
	//              "Road&Railway" to parse only roads & railways
	// minLOD: the minimal LOD that will be parsed
	// maxLOD: the maximal LOD that will be parsed
	// optimize: merge geometries & polygons that share the same appearance in the same object in order to reduce the global hierarchy
	// pruneEmptyObjects: remove the objects which do not contains any geometrical entity
	// tesselate: convert the interior & exteriors polygons to triangles

	class ParserParams {

	public:
		ParserParams( void ) : objectsMask( "All" ), minLOD( 0 ), maxLOD( 4 ), optimize( false ), pruneEmptyObjects( false ), tesselate( true ) { }

	public:
		std::string objectsMask; 
		unsigned int minLOD; 
		unsigned int maxLOD;
		bool optimize; 
		bool pruneEmptyObjects; 
		bool tesselate;
	};

	LIBCITYGML_EXPORT CityModel* load( std::istream& stream, const ParserParams& params );

	LIBCITYGML_EXPORT CityModel* load( const std::string& fileName, const ParserParams& params );

	///////////////////////////////////////////////////////////////////////////////

	class Envelope
	{
		friend class CityGMLHandler;
	public:
		Envelope( void ) {}

		Envelope( const TVec3d& lowerBound, const TVec3d& upperBound )
		{ 
			_lowerBound = lowerBound; 
			_upperBound = upperBound; 
		}

		inline const TVec3d& getLowerBound( void ) const { return _lowerBound; }
		inline const TVec3d& getUpperBound( void ) const { return _upperBound; }

	protected:
		TVec3d _lowerBound;
		TVec3d _upperBound;
	};

	///////////////////////////////////////////////////////////////////////////////
	// Base object associated with an unique id and a set of properties (key-value pairs)
	class Object 
	{
		friend class CityGMLHandler;
		friend std::ostream& operator<<( std::ostream&, const Object & );
	public:
		Object( const std::string& id ) { if ( id != "" ) _id = id; else { std::stringstream ss; ss << "PtrId_" << this; _id = ss.str(); } }

		virtual ~Object( void ) {}

		inline const std::string& getId( void ) const { return _id; }

		inline std::string getProp( const std::string& name ) const
		{
			std::map< std::string, std::string >::const_iterator elt = _properties.find( name );
			return elt != _properties.end() ? elt->second : "";
		}

	protected:
		inline void setProp( const std::string& name, const std::string& value )
		{
			_properties[ name ] = value;
		}

	protected:
		std::string _id;

		std::map< std::string, std::string > _properties;
	};

	///////////////////////////////////////////////////////////////////////////////

	class Appearance : public Object
	{
		friend class CityGMLHandler;
	public:
		Appearance( const std::string& id, const std::string& typeString ) : Object( id ), _typeString( typeString ) {}

		virtual ~Appearance( void ) {}

		inline std::string getType( void ) const { return _typeString; }

		virtual std::string toString( void ) const { return _typeString + " " + _id; }

	protected:
		std::string _typeString;
	};


	///////////////////////////////////////////////////////////////////////////////

	class Texture : virtual public Appearance
	{
		friend class CityGMLHandler;
	public:
		Texture( const std::string& id ) : Appearance( id, "Texture" ), _repeat( false ) {}

		inline std::string getUrl( void ) const { return _url; }

		inline bool getRepeat( void ) const { return _repeat; }

		inline std::string toString( void ) const { return Appearance::toString() + " (url: " + _url + ")"; }

	protected:
		std::string _url;
		bool _repeat;
	};

	///////////////////////////////////////////////////////////////////////////////

	class Material : virtual public Appearance
	{
		friend class CityGMLHandler;
	public:
		Material( const std::string& id ) : Appearance( id, "Material" ), _ambientIntensity( 0.f ), _shininess( 0.f ), _transparency( 0.f ) {}

		inline TVec3f getDiffuse( void ) const { return _diffuse; }
		inline TVec3f getEmissive( void ) const { return _emissive; }
		inline TVec3f getSpecular( void ) const { return _specular; }
		inline float getAmbientIntensity( void ) const { return _ambientIntensity; }
		inline float getShininess( void ) const { return _shininess; }
		inline float getTransparency( void ) const { return _transparency; }

	protected:
		TVec3f _diffuse;
		TVec3f _emissive;
		TVec3f _specular;
		float _ambientIntensity;
		float _shininess;
		float _transparency;
	};

	///////////////////////////////////////////////////////////////////////////////

	typedef std::vector<TVec2f> TexCoords;

	class AppearanceManager 
	{
		friend class CityGMLHandler;
		friend class CityModel;
	public:
		AppearanceManager( void ) {}

		~AppearanceManager( void );

		inline Appearance* getAppearance( const std::string& nodeid ) const
		{
			std::map<std::string, Appearance*>::const_iterator it = _appearanceMap.find( nodeid );
			return ( it != _appearanceMap.end() ) ? it->second : NULL;
		}

		inline bool getTexCoords( const std::string& nodeid, TexCoords &texCoords) const
		{
			texCoords.clear();
			std::map<std::string, TexCoords*>::const_iterator it = _texCoordsMap.find( nodeid );
			if ( it == _texCoordsMap.end() ) return false;
			if ( !it->second ) return false;
			texCoords = *it->second;
			return true;
		}

	protected:
		void addAppearance( Appearance* );
		void assignNode( const std::string& nodeid );
		void assignNode( const std::string& nodeid, Material* );
		void assignTexCoords( const std::string& nodeid, TexCoords* );
		void assignTexCoords( TexCoords* );

		void finish( void ) { _appearanceMap.clear(); _texCoordsMap.clear(); }

	protected:
		std::string _lastId;

		std::vector< Appearance* > _appearances;

		std::map<std::string, Appearance*> _appearanceMap;

		std::map<std::string, TexCoords*> _texCoordsMap;
	};

	///////////////////////////////////////////////////////////////////////////////

	class LinearRing : public Object
	{
		friend class CityGMLHandler;
		friend class Polygon;
	public:
		LinearRing( const std::string& id, bool isExterior ) : Object( id ), _exterior( isExterior ) {}

		bool isExterior( void ) const { return _exterior; }

		inline unsigned int size( void ) const { return _vertices.size(); }

		inline const std::vector<TVec3d>& getVertices( void ) const { return _vertices; }

		inline void addVertex( const TVec3d& v ) { _vertices.push_back( v ); }

		LIBCITYGML_EXPORT TVec3f computeNormal( void ) const;

	protected:
		inline std::vector<TVec3d>& getVertices( void ) { return _vertices; }

		void finish( void );

	protected:		
		bool _exterior;

		std::vector<TVec3d> _vertices;
	};

	///////////////////////////////////////////////////////////////////////////////

	class Polygon : public Object
	{
		friend class CityGMLHandler;
		friend class Geometry;
		friend class Tesseletor;
		friend class CityModel;
	public:
		Polygon( const std::string& id ) : Object( id ), _indices( NULL ),  _appearance( NULL ), _texCoords( NULL ), _exteriorRing( NULL ), _negNormal( false ) {}

		LIBCITYGML_EXPORT ~Polygon( void );

		// Get the vertices
		inline const std::vector<TVec3d>& getVertices( void ) const { return _vertices; }

		// Get the indices
		inline const std::vector<unsigned int>& getIndices( void ) const { return _indices; }

		// Get the normals
		inline const std::vector<TVec3f>& getNormals( void ) const { return _normals; }

		inline const TexCoords& getTexCoords( void ) const { return _texCoords; }

		inline const Appearance* getAppearance( void ) const { return _appearance; }

	protected:
		void finish( bool doTriangulate );
		void finish( AppearanceManager&, Appearance* );

		void addRing( LinearRing* );

		void tesselate( void );
		void mergeRings( void );
		void clearRings( void );

		TVec3f computeNormal( void );

		bool merge( Polygon* );

	protected:
		std::vector<TVec3d> _vertices;
		std::vector<TVec3f> _normals;
		std::vector<unsigned int> _indices;

		Appearance* _appearance;

		TexCoords _texCoords; 

		LinearRing* _exteriorRing;
		std::vector<LinearRing*> _interiorRings;

		bool _negNormal;
	};

	///////////////////////////////////////////////////////////////////////////////

	//typedef enum GeometryType 
	enum GeometryType 
	{
		GT_Unknown = 0,
		GT_Roof,
		GT_Wall,
		GT_Ground,
		GT_Closure,
		GT_Floor,
		GT_InteriorWall,
		GT_Ceiling,
	};

	class Geometry : public Object
	{
		friend class CityGMLHandler;
		friend class CityObject;
		friend std::ostream& operator<<( std::ostream&, const citygml::Geometry& );
	public:
		Geometry( const std::string& id, GeometryType type = GT_Unknown, unsigned int lod = 0 ) : Object( id ), _type( type ), _lod( lod ) {}

		LIBCITYGML_EXPORT ~Geometry();

		// Get the geometry LOD
		inline unsigned int getLOD( void ) const { return _lod; }

		// Get the polygons
		inline unsigned int size( void ) const { return _polygons.size(); }
		inline Polygon* operator[]( unsigned int i ) { return _polygons[i]; }
		inline const Polygon* operator[]( unsigned int i ) const { return _polygons[i]; }

		inline GeometryType getType( void ) const { return _type; }

	protected:
		void addPolygon( Polygon* );

		void finish( AppearanceManager&, Appearance*, bool optimize );

		bool merge( Geometry* );

	protected:
		GeometryType _type;

		unsigned int _lod;

		std::vector< Polygon* > _polygons;
	};

	///////////////////////////////////////////////////////////////////////////////

	std::string getCityObjectsClassName( CityObjectsTypeMask mask );

	CityObjectsTypeMask getCityObjectsTypeMaskFromString( const std::string& stringMask );

	class CityObject : public Object 
	{
		friend class CityGMLHandler;
		friend class CityModel;
		friend std::ostream& operator<<( std::ostream&, const CityObject & );
	public:
		CityObject( const std::string& id, CityObjectsType type ) : Object( id ), _type( type ) {}

		virtual ~CityObject()
		{ 
			std::vector< Geometry* >::const_iterator it = _geometries.begin();
			for ( ; it != _geometries.end(); it++ ) delete *it;
		}

		// Get the object type
		inline CityObjectsType getType( void ) const { return _type; }

		inline std::string getTypeAsString( void ) const { return getCityObjectsClassName( _type ); }

		// Return the envelope (ie. the bounding box) of the object
		inline const Envelope& getEnvelope( void ) const { return _envelope; }

		// Get the default diffuse color of this object class
		virtual TVec4f getDefaultColor( void ) const = 0;

		// Get the number of geometries contains in the object
		inline unsigned int size( void ) const { return _geometries.size(); }

		// Access the geometries
		inline const Geometry* getGeometry( unsigned int i ) const { return _geometries[i]; }

		// Access the children
		inline unsigned int getChildCount( void ) const { return _children.size(); }

		inline const CityObject* getChild( unsigned int i ) const { return _children[i]; }

		inline std::vector< CityObject* >& getChildren( void ) { return _children; }

	protected:
		void finish( AppearanceManager&, bool optimize );

	protected:
		CityObjectsType _type;

		Envelope _envelope;

		std::vector< Geometry* > _geometries;		
		std::vector< CityObject* > _children;
	};

#define MAKE_RGBA( _r_, _g_, _b_, _a_ ) TVec4f( _r_/255.f, _g_/255.f, _b_/255.f, _a_/255.f )
#define MAKE_RGB( _r_, _g_, _b_ ) MAKE_RGBA( _r_, _g_, _b_, 255 )

	// Helper macro to declare a new CityObject type from its name & default color
#define DECLARE_SIMPLE_OBJECT_CLASS( _name_, _defcolor_ ) \
	class _name_ : public CityObject \
	{\
	public:\
	_name_( const std::string& id ) : CityObject( id, COT_ ## _name_ ) {}\
	inline TVec4f getDefaultColor( void ) const { return _defcolor_; }\
	};

	///////////////////////////////////////////////////////////////////////////////

	DECLARE_SIMPLE_OBJECT_CLASS( Building, MAKE_RGB( 186, 184, 135 ) );

	DECLARE_SIMPLE_OBJECT_CLASS( Room, MAKE_RGB( 181, 180, 163 ) );

	DECLARE_SIMPLE_OBJECT_CLASS( Door, MAKE_RGB( 145, 53, 13 ) );

	DECLARE_SIMPLE_OBJECT_CLASS( Window, MAKE_RGBA( 147, 170, 209, 60 ) );

	DECLARE_SIMPLE_OBJECT_CLASS( BuildingInstallation, MAKE_RGB( 186, 186, 177 ) );

	DECLARE_SIMPLE_OBJECT_CLASS( BuildingFurniture, MAKE_RGB( 227, 225, 157 ) );

	DECLARE_SIMPLE_OBJECT_CLASS( CityFurniture, MAKE_RGB( 186, 184, 135 ) );

	DECLARE_SIMPLE_OBJECT_CLASS( WaterBody, MAKE_RGB( 48, 133, 187 ) );

	DECLARE_SIMPLE_OBJECT_CLASS( PlantCover, MAKE_RGB( 0, 184, 0 ) );

	DECLARE_SIMPLE_OBJECT_CLASS( SolitaryVegetationObject, MAKE_RGB( 10, 184, 10 ) );

	DECLARE_SIMPLE_OBJECT_CLASS( Track, MAKE_RGB( 171, 131, 46 ) );

	DECLARE_SIMPLE_OBJECT_CLASS( Road, MAKE_RGB( 159, 159, 159 ) );

	DECLARE_SIMPLE_OBJECT_CLASS( Railway, MAKE_RGB( 180, 180, 180 ) );

	DECLARE_SIMPLE_OBJECT_CLASS( Square, MAKE_RGB( 159, 159, 159 ) );

	DECLARE_SIMPLE_OBJECT_CLASS( TINRelief, MAKE_RGB( 100, 230, 10 ) );

	DECLARE_SIMPLE_OBJECT_CLASS( Tunnel, MAKE_RGB( 180, 180, 150 ) );

	DECLARE_SIMPLE_OBJECT_CLASS( Bridge, MAKE_RGB( 245, 30, 30 ) );

	DECLARE_SIMPLE_OBJECT_CLASS( BridgeConstructionElement, MAKE_RGB( 245, 20, 20 ) );

	DECLARE_SIMPLE_OBJECT_CLASS( BridgeInstallation, MAKE_RGB( 245, 80, 80 ) );

	DECLARE_SIMPLE_OBJECT_CLASS( BridgePart, MAKE_RGB( 245, 50, 50 ) );

	DECLARE_SIMPLE_OBJECT_CLASS( GenericCityObject, MAKE_RGB( 100, 130, 0 ) );

	class LandUse : public CityObject 
	{
	public:
		LandUse( const std::string& id ) : CityObject( id, COT_LandUse ) {}

		inline TVec4f getDefaultColor( void ) const { 

			std::string c = getProp( "class" );
			if ( c != "" )
			{
				int cl = atoi( c.c_str() );
				switch ( cl ) 
				{
				case 1000: return MAKE_RGB( 150, 143, 134 );	// Settlement Area
				case 1100: return MAKE_RGB( 133, 83, 101 );		// Undeveloped Area
				case 2000: return MAKE_RGB( 159, 159, 159 );	// Traffic
				case 3000: return MAKE_RGB( 79, 212, 53 );		// Vegetation
				case 4000: return MAKE_RGB( 67, 109, 247 );		// Water
				}
			}
			return MAKE_RGB( 10, 230, 1 ); 
		}
	};
	///////////////////////////////////////////////////////////////////////////////

	typedef std::vector< CityObject* > CityObjects;
	typedef std::map< CityObjectsType, CityObjects > CityObjectsMap;

	class CityModel : public Object
	{
		friend class CityGMLHandler;
	public:
		CityModel( const std::string& id = "CityModel" ) : Object( id ) {} 

		LIBCITYGML_EXPORT ~CityModel( void );

		// Return the envelope (ie. the bounding box) of the model
		inline const Envelope& getEnvelope( void ) const { return _envelope; }

		// Get the model's number of city objects 
		inline unsigned int size( void ) const
		{ 
			unsigned int count = 0;
			CityObjectsMap::const_iterator it = _cityObjectsMap.begin();
			for ( ; it != _cityObjectsMap.end(); it++ ) count += it->second.size();
			return count; 
		}

		inline const CityObjectsMap& getCityObjectsMap( void ) const { return _cityObjectsMap; }

		inline const CityObjects* getCityObjectsByType( CityObjectsType type ) const
		{ 
			CityObjectsMap::const_iterator it = _cityObjectsMap.find( type );
			if ( it == _cityObjectsMap.end() ) return NULL;
			return &it->second; 
		}

		// Return the roots elements of the model. You can then navigate the hierarchy using object->getChildren().
		inline const CityObjects& getCityObjectsRoots( void ) const { return _roots; }

	protected:
		void addCityObject( CityObject* o );

		inline void addCityObjectAsRoot( CityObject* o ) { if ( o ) _roots.push_back( o ); }

		void finish( bool optimize );

	protected:
		Envelope _envelope;

		CityObjects _roots;

		CityObjectsMap _cityObjectsMap;

		AppearanceManager _appearanceManager;
	};

	///////////////////////////////////////////////////////////////////////////////

	std::ostream& operator<<( std::ostream&, const citygml::Envelope& );
	std::ostream& operator<<( std::ostream&, const citygml::Object& );
	std::ostream& operator<<( std::ostream&, const citygml::Geometry& );
	std::ostream& operator<<( std::ostream&, const citygml::CityObject& );
	std::ostream& operator<<( std::ostream&, const citygml::CityModel & );
}

#endif // __CITYGML_H__
