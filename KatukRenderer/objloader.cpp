#include "objloader.h"
#include <sstream>

void Tokenize( const string& str ,
	           vector<string>& tokens ,
			   const string& delimiters )
{
	// ignore the first char if it is delimiter
	string::size_type lastPos = str.find_first_not_of( delimiters , 0 );
	// find the index of the first char which is not delimeter
	string::size_type pos = str.find_first_of( delimiters , lastPos );

	while(string::npos != pos || string::npos != lastPos )
	{
		tokens.push_back( str.substr( lastPos , pos - lastPos ) );
		// jump over delimeter
		lastPos = str.find_first_not_of( delimiters , pos );
		// find the next char not delimeter
		pos = str.find_first_of( delimiters , lastPos );
	}
}

void loadObj( const char* file_name ,
	          vector<Vector>& vertices ,
			  vector<Normal>& normal ,
			  vector<Vector>& tex ,
			  vector<MeshTriangle> & faces )
{
	// read obj file
	std::fstream mObj;
	char line[256];
	unsigned int len = 256;
	mObj.open( file_name );
	string s;
	float v[3];
	Normal N;

	if( mObj.is_open() )
	{
		while( mObj.getline( line , len ) )
		{
			// v: vertex , vn: normal , f: face
			std::stringstream ss(line);
			ss >> s;
			if( s == "v")
			{
				ss >> v[0] >> v[1] >> v[2];
				vertices.push_back( Vector( v[0],v[1],v[2] ) );
			}
			else if( s == "vt")
			{
				ss >> v[1] >> v[2];
				tex.push_back( Vector( v[1] , v[2], 0.f ) );
			}
			else if( s == "vn" )
			{
				ss >> N[0] >> N[1] >> N[2];
				N = Normalize(N);
				normal.push_back( N );
			}
			else if( s == "f" )
			{
				//vector<unsigned int> idx;
				MeshTriangle idx;
				for( int i=0; i<3; i++ )
				{
					ss >> s;
					vector<string> tokens;
					Tokenize( s , tokens , "/" );

					// count # of "/"
					int count = 0;
					int p = s.find_first_of( "/" , 0 );
					while( string::npos != p )
					{
						count++;
						p = s.find_first_of( "/" , p+1 );
					}

					switch( count )
					{
						case 0:

							//vertex index is only specified
							idx.v(i) = atoi(tokens[0].c_str());
							//idx.push_back( atoi( tokens[0].c_str() ) );
							//idx.push_back( 0 );
							//idx.push_back( 0 );
							break;

						case 1:
							//vertex and texture are specified
							idx.v(i) = atoi(tokens[0].c_str());
							//idx.push_back( atoi( tokens[0].c_str() ) );
							//idx.push_back( atoi( tokens[1].c_str() ) );
							//idx.push_back( 0 );
							break;

						case 2:
							if( tokens.size() == 2 ) // vertex and normal are specified
							{
								idx.v(i) = atoi(tokens[0].c_str());
								idx.n(i) = atoi(tokens[1].c_str());
								//idx.push_back( atoi( tokens[0].c_str() ) );
								//idx.push_back( 0 );
								//idx.push_back( atoi( tokens[1].c_str() ) );
							}
							else
							{
								idx.v(i) = atoi(tokens[0].c_str());
								idx.n(i) = atoi(tokens[2].c_str());
								//idx.push_back( atoi( tokens[0].c_str() ) );
								//idx.push_back( atoi( tokens[1].c_str() ) );
								//idx.push_back( atoi( tokens[2].c_str() ) );
							}

							break;						
					}
					
				}
				// push face's indices
				faces.push_back( idx );
			}
		}
	}
	else
	{
		std::cout << "Unable to open file.\n";
		exit(0);
	}

	mObj.close();
}