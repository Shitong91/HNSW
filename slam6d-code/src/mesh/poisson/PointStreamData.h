/*
Copyright (c) 2006, Michael Kazhdan and Matthew Bolitho
All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list of
conditions and the following disclaimer. Redistributions in binary form must reproduce
the above copyright notice, this list of conditions and the following disclaimer
in the documentation and/or other materials provided with the distribution. 

Neither the name of the Johns Hopkins University nor the names of its contributors
may be used to endorse or promote products derived from this software without specific
prior written permission. 

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO THE IMPLIED WARRANTIES 
OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
TO, PROCUREMENT OF SUBSTITUTE  GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
DAMAGE.
*/
#ifndef POINT_STREAM_DATA_INCLUDED
#define POINT_STREAM_DATA_INCLUDED

#include <algorithm>
#include <tuple>
#include "Ply.h"

template< class Real > using Color = Point< Real , 3 >;
template< class Real > void SetColorValues( const Color< Real >& color , unsigned char c[3] ){ for( int i=0 ; i<3 ; i++ ) c[i] = (unsigned char)std::max< int >( 0 , std::min< int >( 255 , (int)( color[i]+0.5 ) ) ); }
template< class Real > void SetColorValues( const Color< Real >& color , RGBColor& c ){ for( int i=0 ; i<3 ; i++ ) c[i] = (unsigned char)std::max< int >( 0 , std::min< int >( 255 , (int)( color[i]+0.5 ) ) ); }

// Should have:
// -- binary operators for vectors
// -- static ReadASCII
// -- static WriteASCII
// -- static ReadBinary
// -- static WriteBinary
// -- static ValidPlyReadProperties( const bool* ) method
// -- static int PlyReadNum
// -- static int PlyWriteNum
// -- static const PlyProperty* PlyReadProperties()
// -- static const PlyProperty* PlyWriteProperties()
// -- a nested class Transform which gets initialized by something and acts on the data
template< typename Real , typename Data >
struct PointStreamData
{
	Data data;

	PointStreamData& operator += ( const PointStreamData& p ){ data += p.data ; return *this; }
	PointStreamData& operator -= ( const PointStreamData& p ){ data -= p.data ; return *this; }
	PointStreamData& operator *= ( Real s )                  { data *= s ; return *this; }
	PointStreamData& operator /= ( Real s )                  { data /= s ; return *this; }
	PointStreamData  operator +  ( const PointStreamData& p ) const { PointStreamData _p = *this ; _p += p ; return _p; }
	PointStreamData  operator -  ( const PointStreamData& p ) const { PointStreamData _p = *this ; _p -= p ; return _p; }
	PointStreamData  operator *  ( Real s )                   const { PointStreamData _p = *this ; _p *= s ; return _p; }
	PointStreamData  operator /  ( Real s )                   const { PointStreamData _p = *this ; _p /= s ; return _p; }

	static const int PlyReadNum;
	static const int PlyWriteNum;
	static const PlyProperty* PlyReadProperties( void );
	static const PlyProperty* PlyWriteProperties( void );
	static bool ValidPlyReadProperties( const bool* flags );
};
template< class Real , unsigned int Dim >
struct PointStreamPosition : public PointStreamData< Real , Point< Real , Dim > >
{
	struct Transform
	{
		Transform( void ){}
		Transform( const XForm< Real , Dim+1 >& xForm ) : _xForm(xForm) { }
		PointStreamPosition operator() ( const PointStreamPosition& p ) const
		{
			PointStreamPosition _p;
			_p.data = _xForm * p.data;
			return _p;
		}
	protected:
		XForm< Real , Dim+1 > _xForm;
	};
	static void readASCII( FILE* fp , PointStreamPosition& p )
	{
		float f;
		for( int i=0 ; i<Dim ; i++ )
			if( fscanf( fp , " %f " , &f )!=1 ) fprintf( stderr , "[ERROR] PointStreamPosition::readASCII: Failed to read color\n" ) , exit( 0 );
			else p.data[i] = (Real)f;
	};
	static void ReadBinary( FILE* fp , PointStreamPosition& p )
	{
		float f;
		for( int i=0 ; i<Dim ; i++ )
			if( fread( &f , sizeof(float) , 1 , fp )!=1 ) fprintf( stderr , "[ERROR] PointStreamPosition::readBINARY: Failed to read color\n" ) , exit( 0 );
			else p.data[i] = (Real)f;
	}
	static void WriteASCII( FILE* fp , const PointStreamPosition& p ){ for( int i=0 ; i<Dim ; i++ ) fprintf( fp , " %f" , (float)p.data[i] ); };
	static void WriteBinary( FILE* fp , const PointStreamPosition& p )
	{
		for( int i=0 ; i<Dim ; i++ )
		{
			float f = (float)p.data[i];
			fwrite( &f , sizeof(float) , 1 , fp );
		}
	}

	static const int PlyReadNum = Dim;
	static const int PlyWriteNum = Dim;
	static const PlyProperty* PlyReadProperties( void ){ return _PlyProperties; }
	static const PlyProperty* PlyWriteProperties( void ){ return _PlyProperties; }
	static bool ValidPlyReadProperties( const bool* flags ){ for( int d=0 ; d<Dim ; d++ ) if( !flags[d] ) return false ; return true ; }
protected:
	static const PlyProperty _PlyProperties[];
};
template<>
const PlyProperty PointStreamPosition< float , 2 >::_PlyProperties[] =
{
	{ "x" , PLY_FLOAT , PLY_FLOAT , int( offsetof( PointStreamPosition , data.coords[0] ) ) , 0 , 0 , 0 , 0 } ,
{ "y" , PLY_FLOAT , PLY_FLOAT , int( offsetof( PointStreamPosition , data.coords[1] ) ) , 0 , 0 , 0 , 0 } ,
};
template<>
const PlyProperty PointStreamPosition< double , 2 >::_PlyProperties[] =
{
	{ "x" , PLY_FLOAT , PLY_DOUBLE , int( offsetof( PointStreamPosition , data.coords[0] ) ) , 0 , 0 , 0 , 0 } ,
{ "y" , PLY_FLOAT , PLY_DOUBLE , int( offsetof( PointStreamPosition , data.coords[1] ) ) , 0 , 0 , 0 , 0 } ,
};
template<>
const PlyProperty PointStreamPosition< float , 3 >::_PlyProperties[] =
{
	{ "x" , PLY_FLOAT , PLY_FLOAT , int( offsetof( PointStreamPosition , data.coords[0] ) ) , 0 , 0 , 0 , 0 } ,
{ "y" , PLY_FLOAT , PLY_FLOAT , int( offsetof( PointStreamPosition , data.coords[1] ) ) , 0 , 0 , 0 , 0 } ,
{ "z" , PLY_FLOAT , PLY_FLOAT , int( offsetof( PointStreamPosition , data.coords[2] ) ) , 0 , 0 , 0 , 0 } ,
};
template<>
const PlyProperty PointStreamPosition< double , 3 >::_PlyProperties[] =
{
	{ "x" , PLY_FLOAT , PLY_DOUBLE , int( offsetof( PointStreamPosition , data.coords[0] ) ) , 0 , 0 , 0 , 0 } ,
{ "y" , PLY_FLOAT , PLY_DOUBLE , int( offsetof( PointStreamPosition , data.coords[1] ) ) , 0 , 0 , 0 , 0 } ,
{ "z" , PLY_FLOAT , PLY_DOUBLE , int( offsetof( PointStreamPosition , data.coords[2] ) ) , 0 , 0 , 0 , 0 } ,
};
template<>
const PlyProperty PointStreamPosition< float , 4 >::_PlyProperties[] =
{
	{ "x" , PLY_FLOAT , PLY_FLOAT , int( offsetof( PointStreamPosition , data.coords[0] ) ) , 0 , 0 , 0 , 0 } ,
{ "y" , PLY_FLOAT , PLY_FLOAT , int( offsetof( PointStreamPosition , data.coords[1] ) ) , 0 , 0 , 0 , 0 } ,
{ "z" , PLY_FLOAT , PLY_FLOAT , int( offsetof( PointStreamPosition , data.coords[2] ) ) , 0 , 0 , 0 , 0 } ,
{ "w" , PLY_FLOAT , PLY_FLOAT , int( offsetof( PointStreamPosition , data.coords[3] ) ) , 0 , 0 , 0 , 0 } ,
};
template<>
const PlyProperty PointStreamPosition< double , 4 >::_PlyProperties[] =
{
	{ "x" , PLY_FLOAT , PLY_DOUBLE , int( offsetof( PointStreamPosition , data.coords[0] ) ) , 0 , 0 , 0 , 0 } ,
{ "y" , PLY_FLOAT , PLY_DOUBLE , int( offsetof( PointStreamPosition , data.coords[1] ) ) , 0 , 0 , 0 , 0 } ,
{ "z" , PLY_FLOAT , PLY_DOUBLE , int( offsetof( PointStreamPosition , data.coords[2] ) ) , 0 , 0 , 0 , 0 } ,
{ "w" , PLY_FLOAT , PLY_DOUBLE , int( offsetof( PointStreamPosition , data.coords[3] ) ) , 0 , 0 , 0 , 0 } ,
};

template< class Real , unsigned int Dim >
struct PointStreamNormal : public PointStreamData< Real , Point< Real , Dim > >
{
	struct Transform
	{
		Transform( void ){}
		Transform( const XForm< Real , Dim+1 >& xForm )
		{
			for( int i=0 ; i<Dim ; i++ ) for( int j=0 ; j<Dim ; j++ ) _xForm(i,j) = xForm(i,j);
			_xForm = _xForm.transpose().inverse();
			_xForm /= (Real)pow( fabs( _xForm.determinant() ) , 1./Dim );
		}
		PointStreamNormal operator() ( const PointStreamNormal& n ) const
		{
			PointStreamNormal _n;
			_n.data = _xForm * n.data;
			return _n;
		}
	protected:
		XForm< Real , Dim > _xForm;
	};
	static void ReadASCII( FILE* fp , PointStreamNormal& p )
	{
		float f;
		for( int i=0 ; i<Dim ; i++ )
			if( fscanf( fp , " %f " , &f )!=1 ) fprintf( stderr , "[ERROR] PointStreamNormal::readASCII: Failed to read normal\n" ) , exit( 0 );
			else p.data[i] = (Real)f;
	};
	static void ReadBinary( FILE* fp , PointStreamNormal& p )
	{
		float f;
		for( int i=0 ; i<Dim ; i++ )
			if( fread( &f , sizeof(float) , 1 , fp )!=1 ) fprintf( stderr , "[ERROR] PointStreamNormal::readBINARY: Failed to read normal\n" ) , exit( 0 );
			else p.data[i] = (Real)f;
	}
	static void WriteASCII( FILE* fp , const PointStreamNormal& p ){ for( int i=0 ; i<Dim ; i++ ) fprintf( fp , " %f" , (float)p.data[i] ); };
	static void WriteBinary( FILE* fp , const PointStreamNormal& p )
	{
		for( int i=0 ; i<Dim ; i++ )
		{
			float f = (float)p.data[i];
			fwrite( &f , sizeof( float) , 1 , fp );
		}
	}
	static const int PlyReadNum = Dim;
	static const int PlyWriteNum = Dim;
	static const PlyProperty* PlyReadProperties( void ){ return _PlyProperties; }
	static const PlyProperty* PlyWriteProperties( void ){ return _PlyProperties; }
	static bool ValidPlyReadProperties( const bool* flags ){ for( int d=0 ; d<Dim ; d++ ) if( !flags[d] ) return false ; return true ; }
protected:
	static const PlyProperty _PlyProperties[];
};
template<>
const PlyProperty PointStreamNormal< float , 2 >::_PlyProperties[] =
{
	{ "nx" , PLY_FLOAT , PLY_FLOAT , int( offsetof( PointStreamNormal , data.coords[0] ) ) , 0 , 0 , 0 , 0 } ,
{ "ny" , PLY_FLOAT , PLY_FLOAT , int( offsetof( PointStreamNormal , data.coords[1] ) ) , 0 , 0 , 0 , 0 } ,
};
template<>
const PlyProperty PointStreamNormal< double , 2 >::_PlyProperties[] =
{
	{ "nx" , PLY_FLOAT , PLY_DOUBLE , int( offsetof( PointStreamNormal , data.coords[0] ) ) , 0 , 0 , 0 , 0 } ,
{ "ny" , PLY_FLOAT , PLY_DOUBLE , int( offsetof( PointStreamNormal , data.coords[1] ) ) , 0 , 0 , 0 , 0 } ,
};
template<>
const PlyProperty PointStreamNormal< float , 3 >::_PlyProperties[] =
{
	{ "nx" , PLY_FLOAT , PLY_FLOAT , int( offsetof( PointStreamNormal , data.coords[0] ) ) , 0 , 0 , 0 , 0 } ,
{ "ny" , PLY_FLOAT , PLY_FLOAT , int( offsetof( PointStreamNormal , data.coords[1] ) ) , 0 , 0 , 0 , 0 } ,
{ "nz" , PLY_FLOAT , PLY_FLOAT , int( offsetof( PointStreamNormal , data.coords[2] ) ) , 0 , 0 , 0 , 0 } ,
};
template<>
const PlyProperty PointStreamNormal< double , 3 >::_PlyProperties[] =
{
	{ "nx" , PLY_FLOAT , PLY_DOUBLE , int( offsetof( PointStreamNormal , data.coords[0] ) ) , 0 , 0 , 0 , 0 } ,
{ "ny" , PLY_FLOAT , PLY_DOUBLE , int( offsetof( PointStreamNormal , data.coords[1] ) ) , 0 , 0 , 0 , 0 } ,
{ "nz" , PLY_FLOAT , PLY_DOUBLE , int( offsetof( PointStreamNormal , data.coords[2] ) ) , 0 , 0 , 0 , 0 } ,
};
template<>
const PlyProperty PointStreamNormal< float , 4 >::_PlyProperties[] =
{
	{ "nx" , PLY_FLOAT , PLY_FLOAT , int( offsetof( PointStreamNormal , data.coords[0] ) ) , 0 , 0 , 0 , 0 } ,
{ "ny" , PLY_FLOAT , PLY_FLOAT , int( offsetof( PointStreamNormal , data.coords[1] ) ) , 0 , 0 , 0 , 0 } ,
{ "nz" , PLY_FLOAT , PLY_FLOAT , int( offsetof( PointStreamNormal , data.coords[2] ) ) , 0 , 0 , 0 , 0 } ,
{ "nw" , PLY_FLOAT , PLY_FLOAT , int( offsetof( PointStreamNormal , data.coords[3] ) ) , 0 , 0 , 0 , 0 } ,
};
template<>
const PlyProperty PointStreamNormal< double , 4 >::_PlyProperties[] =
{
	{ "nx" , PLY_FLOAT , PLY_DOUBLE , int( offsetof( PointStreamNormal , data.coords[0] ) ) , 0 , 0 , 0 , 0 } ,
{ "ny" , PLY_FLOAT , PLY_DOUBLE , int( offsetof( PointStreamNormal , data.coords[1] ) ) , 0 , 0 , 0 , 0 } ,
{ "nz" , PLY_FLOAT , PLY_DOUBLE , int( offsetof( PointStreamNormal , data.coords[2] ) ) , 0 , 0 , 0 , 0 } ,
{ "nw" , PLY_FLOAT , PLY_DOUBLE , int( offsetof( PointStreamNormal , data.coords[3] ) ) , 0 , 0 , 0 , 0 } ,
};

template< class Real >
struct PointStreamColor : public PointStreamData< Real , Color< Real > >
{
	struct Transform
	{
		Transform( void ){}
		template< typename X > Transform( const X& ){}
		PointStreamColor operator() ( const PointStreamColor& c ) const { return c; }
	};
	static void ReadASCII( FILE* fp , PointStreamColor& p )
	{
		unsigned char c[3];
		if( fscanf( fp , " %c %c %c " , &c[0] , &c[1] , &c[2] )!=3 ) fprintf( stderr , "[ERROR] PointStreamColor::readASCII: Failed to read color\n" ) , exit( 0 );
		p.data[0] = (Real)c[0] , p.data[1] = (Real)c[1] , p.data[2] = (Real)c[2];
	};
	static void ReadBinary( FILE* fp , PointStreamColor& p )
	{
		unsigned char c[3];
		if( fread( c , sizeof(unsigned char) , 3 , fp )!=3 ) fprintf( stderr , "[ERROR] PointStreamColor::readBinary: Failed to read color\n" ) , exit( 0 );
		p.data[0] = (Real)c[0] , p.data[1] = (Real)c[1] , p.data[2] = (Real)c[2];
	}
	static void WriteASCII( FILE* fp , const PointStreamColor& p )
	{
		unsigned char c[3];
		SetColorValues( p.data , c );
		fprintf( fp , " %d %d %d " , c[0] , c[1] , c[2] );
	};
	static void WriteBinary( FILE* fp , const PointStreamColor& p )
	{
		unsigned char c[3];
		SetColorValues( p.data , c );
		fwrite( c , sizeof(unsigned char) , 3 , fp );
	}
	static const int PlyReadNum = 6;
	static const int PlyWriteNum = 3;
	static const PlyProperty* PlyReadProperties( void ){ return _PlyProperties; }
	static const PlyProperty* PlyWriteProperties( void ){ return _PlyProperties; }
	static bool ValidPlyReadProperties( const bool* flags ){ for( int d=0 ; d<3 ; d++ ) if( !flags[d] && !flags[d+3] ) return false ; return true ; }
protected:
	static const PlyProperty _PlyProperties[];
};
template<>
const PlyProperty PointStreamColor< float >::_PlyProperties[] =
{
	{ "red"   , PLY_UCHAR , PLY_FLOAT , int( offsetof( PointStreamColor , data.coords[0] ) ) , 0 , 0 , 0 , 0 } ,
{ "green" , PLY_UCHAR , PLY_FLOAT , int( offsetof( PointStreamColor , data.coords[1] ) ) , 0 , 0 , 0 , 0 } ,
{ "blue"  , PLY_UCHAR , PLY_FLOAT , int( offsetof( PointStreamColor , data.coords[2] ) ) , 0 , 0 , 0 , 0 } ,
{ "r"     , PLY_UCHAR , PLY_FLOAT , int( offsetof( PointStreamColor , data.coords[0] ) ) , 0 , 0 , 0 , 0 } ,
{ "g"     , PLY_UCHAR , PLY_FLOAT , int( offsetof( PointStreamColor , data.coords[1] ) ) , 0 , 0 , 0 , 0 } ,
{ "b"     , PLY_UCHAR , PLY_FLOAT , int( offsetof( PointStreamColor , data.coords[2] ) ) , 0 , 0 , 0 , 0 } ,
};
template<>
const PlyProperty PointStreamColor< double >::_PlyProperties[] =
{
	{ "red"   , PLY_UCHAR , PLY_DOUBLE , int( offsetof( PointStreamColor , data.coords[0] ) ) , 0 , 0 , 0 , 0 } , 
{ "green" , PLY_UCHAR , PLY_DOUBLE , int( offsetof( PointStreamColor , data.coords[1] ) ) , 0 , 0 , 0 , 0 } ,
{ "blue"  , PLY_UCHAR , PLY_DOUBLE , int( offsetof( PointStreamColor , data.coords[2] ) ) , 0 , 0 , 0 , 0 } ,
{ "r"     , PLY_UCHAR , PLY_DOUBLE , int( offsetof( PointStreamColor , data.coords[0] ) ) , 0 , 0 , 0 , 0 } , 
{ "g"     , PLY_UCHAR , PLY_DOUBLE , int( offsetof( PointStreamColor , data.coords[1] ) ) , 0 , 0 , 0 , 0 } ,
{ "b"     , PLY_UCHAR , PLY_DOUBLE , int( offsetof( PointStreamColor , data.coords[2] ) ) , 0 , 0 , 0 , 0 } ,
};

template< class Real >
struct PointStreamValue : public PointStreamData< Real , Real >
{
	struct Transform
	{
		Transform( void ){}
		template< typename X > Transform( const X& ){}
		PointStreamValue operator() ( const PointStreamValue& r ) const { return r; }
	};
	static void  ReadASCII ( FILE* fp , PointStreamValue& p ){ float f ; if( fscanf( fp , " %f " , &f )!=1 ) fprintf( stderr , "[ERROR] PointStreamValue::readASCII: Failed to read color\n" ) , exit( 0 ) ; p.data = (Real)f; }
	static void  ReadBinary( FILE* fp , PointStreamValue& p ){ float f ; if( fread( &f , sizeof(float) , 1 , fp )!=1 ) fprintf( stderr , "[ERROR] PointStreamValue::readBinary: Failed to read color\n" ) , exit( 0 ) ; p.data = (Real)f; }
	static void WriteASCII ( FILE* fp , const PointStreamValue& p ){ float f = (float)p.data ; fprintf( fp , " %f " , f ); }
	static void WriteBinary( FILE* fp , const PointStreamValue& p ){ float f = (float)p.data ; fwrite( &f , sizeof(Real) , 1 , fp ); }
	static const int PlyReadNum = 1;
	static const int PlyWriteNum = 1;
	static const PlyProperty* PlyReadProperties( void ){ return _PlyProperties; }
	static const PlyProperty* PlyWriteProperties( void ){ return _PlyProperties; }
	static bool ValidPlyReadProperties( const bool* flags ){ if( !flags[0] ) return false ; return true ; }
public:
	static const PlyProperty _PlyProperties[];
};
template<>
const PlyProperty PointStreamValue< float >::_PlyProperties[] =
{
	{ "value" , PLY_FLOAT , PLY_FLOAT , int( offsetof( PointStreamValue , data ) ) , 0 , 0 , 0 , 0 } , 
};
template<>
const PlyProperty PointStreamValue< double >::_PlyProperties[] =
{
	{ "value" , PLY_FLOAT , PLY_DOUBLE , int( offsetof( PointStreamValue , data ) ) , 0 , 0 , 0 , 0 } , 
};
template< class Real >
struct PointStreamRoughness : public PointStreamData< Real , Real >
{
	struct Transform
	{
		Transform( void ){}
		template< typename X > Transform( const X& ){}
		PointStreamRoughness operator() ( const PointStreamRoughness& r ) const { return r; }
	};
	static void  ReadASCII ( FILE* fp , PointStreamRoughness& p ){ float f ; if( fscanf( fp , " %f " , &f )!=1 ) fprintf( stderr , "[ERROR] PointStreamRoughness::readASCII: Failed to read color\n" ) , exit( 0 ) ; p.data = (Real)f; }
	static void  ReadBinary( FILE* fp , PointStreamRoughness& p ){ float f ; if( fread( &f , sizeof(float) , 1 , fp )!=1 ) fprintf( stderr , "[ERROR] PointStreamRoughness::readBinary: Failed to read color\n" ) , exit( 0 ) ; p.data = (Real)f; }
	static void WriteASCII ( FILE* fp , const PointStreamRoughness& p ){ float f = (float)p.data ; fprintf( fp , " %f " , f ); }
	static void WriteBinary( FILE* fp , const PointStreamRoughness& p ){ float f = (float)p.data ; fwrite( &f , sizeof(Real) , 1 , fp ); }
	static const int PlyReadNum = 1;
	static const int PlyWriteNum = 1;
	static const PlyProperty* PlyReadProperties( void ){ return _PlyProperties; }
	static const PlyProperty* PlyWriteProperties( void ){ return _PlyProperties; }
	static bool ValidPlyReadProperties( const bool* flags ){ if( !flags[0] ) return false ; return true ; }
public:
	static const PlyProperty _PlyProperties[];
};
template<>
const PlyProperty PointStreamRoughness< float >::_PlyProperties[] =
{
	{ "rg" , PLY_FLOAT , PLY_FLOAT , int( offsetof( PointStreamRoughness , data ) ) , 0 , 0 , 0 , 0 } , 
};
template<>
const PlyProperty PointStreamRoughness< double >::_PlyProperties[] =
{
	{ "rg" , PLY_FLOAT , PLY_DOUBLE , int( offsetof( PointStreamRoughness , data ) ) , 0 , 0 , 0 , 0 } , 
};

template< typename Real , typename ... Data >
struct MultiPointStreamData : public PointStreamData< Real , std::tuple< Data ... > >
{
	typedef std::tuple< Data ... > MultiData;
	using PointStreamData< Real , MultiData >::data;
	template< unsigned int I > using DataType = typename std::tuple_element< I , MultiData >::type;

	struct Transform
	{
		Transform( void ){}
		template< typename X >
		Transform( const X& x ){ _initTransforms<0>( x ); }
		MultiPointStreamData operator() ( const MultiPointStreamData& d ) const
		{
			MultiPointStreamData _d;
			_transform<0>( d , _d );
			return _d;
		}
	protected:
		typedef std::tuple< typename Data::Transform ... > Transforms;
		template< unsigned int I > using TransformType = typename std::tuple_element< I , Transforms >::type;
		Transforms _xForms;
	private:
		template< unsigned int I , typename X >
		typename std::enable_if< I!=sizeof...(Data) >::type _initTransforms( const X& x ){ std::get< I >( _xForms ) = TransformType< I >( x ) ; _initTransforms< I+1 >( x ); }
		template< unsigned int I , typename X >
		typename std::enable_if< I==sizeof...(Data) >::type _initTransforms( const X& x ){ }
		template< unsigned int I >
		typename std::enable_if< I!=sizeof...(Data) >::type _transform( const MultiPointStreamData& in , MultiPointStreamData& out ) const { std::get< I >( out.data ) = std::get< I >( _xForms )( std::get< I >( in.data ) ) ; _transform< I+1 >( in , out ); }
		template< unsigned int I >
		typename std::enable_if< I==sizeof...(Data) >::type _transform( const MultiPointStreamData& in , MultiPointStreamData& out ) const { }
	};

	static void  ReadASCII ( FILE* fp , MultiPointStreamData& p ){ p._readASCII <0>( fp ); }
	static void  ReadBinary( FILE* fp , MultiPointStreamData& p ){ p._readBinary<0>( fp ); }
	static void WriteASCII ( FILE* fp , const MultiPointStreamData& p ){ p._writeASCII <0>( fp ); }
	static void WriteBinary( FILE* fp , const MultiPointStreamData& p ){ p._writeBinary<0>( fp ); }

	MultiPointStreamData& operator += ( const MultiPointStreamData& p ){ _add<0>( p ) ; return *this; }
	MultiPointStreamData& operator -= ( const MultiPointStreamData& p ){ _sub<0>( p ) ; return *this; }
	MultiPointStreamData& operator *= ( Real s )                       { _mul<0>( s ) ; return *this; }
	MultiPointStreamData& operator /= ( Real s )                       { _div<0>( s ) ; return *this; }
	MultiPointStreamData  operator +  ( const MultiPointStreamData& p ) const { MultiPointStreamData _p = *this ; _p += p ; return _p; }
	MultiPointStreamData  operator -  ( const MultiPointStreamData& p ) const { MultiPointStreamData _p = *this ; _p -= p ; return _p; }
	MultiPointStreamData  operator *  ( Real s )                        const { MultiPointStreamData _p = *this ; _p *= s ; return _p; }
	MultiPointStreamData  operator /  ( Real s )                        const { MultiPointStreamData _p = *this ; _p /= s ; return _p; }

private:
	template< unsigned int I > static constexpr typename std::enable_if< I!=sizeof...(Data) , int >::type _PlyTotalReadNum( void ){ return DataType< I >::PlyReadNum + _PlyTotalReadNum< I+1 >(); }
	template< unsigned int I > static constexpr typename std::enable_if< I==sizeof...(Data) , int >::type _PlyTotalReadNum( void ){ return 0; }
	template< unsigned int I > static constexpr typename std::enable_if< I!=sizeof...(Data) , int >::type _PlyTotalWriteNum( void ){ return DataType< I >::PlyWriteNum + _PlyTotalWriteNum< I+1 >(); }
	template< unsigned int I > static constexpr typename std::enable_if< I==sizeof...(Data) , int >::type _PlyTotalWriteNum( void ){ return 0; }
public:
	static const int PlyReadNum = _PlyTotalReadNum<0>();
	static const int PlyWriteNum = _PlyTotalWriteNum<0>();
	static PlyProperty* PlyReadProperties( void ){ _SetPlyReadProperties<0>( _PlyReadProperties ) ; return _PlyReadProperties; }
	static PlyProperty* PlyWriteProperties( void ){ _SetPlyWriteProperties<0>( _PlyWriteProperties ) ; return _PlyWriteProperties; }

	static bool ValidPlyReadProperties( const bool* flags ){ return _ValidPlyReadProperties<0>( flags ) ; }
	template< unsigned int I > static bool ValidPlyReadProperties( const bool* flags ){ return DataType< I >::ValidPlyReadProperties( flags + _PlyReadNum< I >() ); }
protected:
	static PlyProperty _PlyReadProperties[];
	static PlyProperty _PlyWriteProperties[];
private:
	template< unsigned int I , unsigned int _I=0 > static typename std::enable_if< I==_I , unsigned int >::type _PlyReadNum( void ){ return 0; }
	template< unsigned int I , unsigned int _I=0 > static typename std::enable_if< I!=_I , unsigned int >::type _PlyReadNum( void ){ return DataType< I >::PlyReadNum + _PlyReadNum< I , _I+1 >(); }

	template< unsigned int I > typename std::enable_if< I!=sizeof...(Data) >::type  _readASCII ( FILE* fp )       { DataType< I >:: ReadASCII ( fp , std::get< I >( data ) ) ;  _readASCII < I+1 >( fp ); }
	template< unsigned int I > typename std::enable_if< I==sizeof...(Data) >::type  _readASCII ( FILE* fp )       { }
	template< unsigned int I > typename std::enable_if< I!=sizeof...(Data) >::type  _readBinary( FILE* fp )       { DataType< I >:: ReadBinary( fp , std::get< I >( data ) ) ;  _readBinary< I+1 >( fp ); }
	template< unsigned int I > typename std::enable_if< I==sizeof...(Data) >::type  _readBinary( FILE* fp )       { }
	template< unsigned int I > typename std::enable_if< I!=sizeof...(Data) >::type _writeASCII ( FILE* fp ) const { DataType< I >::WriteASCII ( fp , std::get< I >( data ) ) ; _writeASCII < I+1 >( fp ); }
	template< unsigned int I > typename std::enable_if< I==sizeof...(Data) >::type _writeASCII ( FILE* fp ) const { }
	template< unsigned int I > typename std::enable_if< I!=sizeof...(Data) >::type _writeBinary( FILE* fp ) const { DataType< I >::WriteBinary( fp , std::get< I >( data ) ) ; _writeBinary< I+1 >( fp ); }
	template< unsigned int I > typename std::enable_if< I==sizeof...(Data) >::type _writeBinary( FILE* fp ) const { }

	template< unsigned int I > typename std::enable_if< I!=sizeof...(Data) >::type _add( const MultiPointStreamData& p ){ std::get< I >( data ) += std::get< I >( p.data ) ; _add< I+1 >( p ); }
	template< unsigned int I > typename std::enable_if< I==sizeof...(Data) >::type _add( const MultiPointStreamData& p ){ }
	template< unsigned int I > typename std::enable_if< I!=sizeof...(Data) >::type _sub( const MultiPointStreamData& p ){ std::get< I >( data ) -= std::get< I >( p.data ) ; _sub< I+1 >( p ); }
	template< unsigned int I > typename std::enable_if< I==sizeof...(Data) >::type _sub( const MultiPointStreamData& p ){ }
	template< unsigned int I > typename std::enable_if< I!=sizeof...(Data) >::type _mul( Real s ){ std::get< I >( data ) *= s ; _mul< I+1 >( s ); }
	template< unsigned int I > typename std::enable_if< I==sizeof...(Data) >::type _mul( Real s ){ }
	template< unsigned int I > typename std::enable_if< I!=sizeof...(Data) >::type _div( Real s ){ std::get< I >( data ) /= s ; _div< I+1 >( s ); }
	template< unsigned int I > typename std::enable_if< I==sizeof...(Data) >::type _div( Real s ){ }

	template< unsigned int I > static typename std::enable_if< I!=sizeof...(Data) >::type _SetPlyReadProperties( PlyProperty* PlyReadProperties )
	{
		for( int d=0 ; d<DataType< I >::PlyReadNum ; d++ )
		{
			PlyReadProperties[d] = DataType< I >::PlyReadProperties()[d];
			MultiPointStreamData temp;
			const typename std::tuple_element< I , MultiData >::type& temp_data = std::get< I >( temp.data );
			PlyReadProperties[d].offset += (int)( (size_t)&temp_data - (size_t)&temp );
		}
		_SetPlyReadProperties< I+1 >( PlyReadProperties + DataType< I >::PlyReadNum );
	}
	template< unsigned int I > static typename std::enable_if< I==sizeof...(Data) >::type _SetPlyReadProperties( PlyProperty* PlyReadProperties ){ }
	template< unsigned int I > static typename std::enable_if< I!=sizeof...(Data) >::type _SetPlyWriteProperties( PlyProperty* PlyWriteProperties )
	{
		for( int d=0 ; d<DataType< I >::PlyWriteNum ; d++ )
		{
			PlyWriteProperties[d] = DataType< I >::PlyWriteProperties()[d];
			MultiPointStreamData temp;
			const typename std::tuple_element< I , MultiData >::type& temp_data = std::get< I >( temp.data );
			PlyWriteProperties[d].offset += (int)( (size_t)&temp_data - (size_t)&temp );
		}
		_SetPlyWriteProperties< I+1 >( PlyWriteProperties + DataType< I >::PlyWriteNum );
	}
	template< unsigned int I > static typename std::enable_if< I==sizeof...(Data) >::type _SetPlyWriteProperties( PlyProperty* PlyWriteProperties ){ }

	template< unsigned int I > static typename std::enable_if< I!=sizeof...(Data) , bool >::type _ValidPlyReadProperties( const bool* flags ){ return DataType< I >::ValidPlyReadProperties( flags ) && _ValidPlyReadProperties< I+1 >( flags + std::tuple_element< I , MultiData >::type::PlyReadNum ); }
	template< unsigned int I > static typename std::enable_if< I==sizeof...(Data) , bool >::type _ValidPlyReadProperties( const bool* flags ){ return true; }
};
template< typename Real , typename ... Data > PlyProperty MultiPointStreamData< Real , Data ... >::_PlyReadProperties[ MultiPointStreamData< Real , Data ... >::PlyReadNum==0 ? 1 : MultiPointStreamData< Real , Data ... >::PlyReadNum ];
template< typename Real , typename ... Data > PlyProperty MultiPointStreamData< Real , Data ... >::_PlyWriteProperties[ MultiPointStreamData< Real , Data ... >::PlyWriteNum==0 ? 1 : MultiPointStreamData< Real , Data ... >::PlyWriteNum ];

#endif // POINT_STREAM_DATA_INCLUDED