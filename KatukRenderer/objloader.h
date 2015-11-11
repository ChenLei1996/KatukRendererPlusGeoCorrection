#ifndef OBJLOADER_H
#define OBJLOADER_H
#include "global.h"
#include "core/geometry.h"
#include "core/shape.h"

void Tokenize( const string& str ,
	           vector<string>& tokens ,
			   const string& delimiters = " " );

void loadObj( const char* file_name ,
	          std::vector<Vector>& vertices ,
			  std::vector<Normal>& normal ,
			  std::vector<Vector>& tex ,
			  std::vector<MeshTriangle>& faces );


#endif