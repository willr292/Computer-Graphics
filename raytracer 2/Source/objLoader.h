#include <glm/glm.hpp>
#include <vector>
#include <fstream>
#include <string>

using namespace std;
using glm::vec3;
using glm::mat3;
using glm::vec4;
using glm::mat4;

void loadOBJ(
        vector <Triangle> & triangles
        ) {

        //std::vector< unsigned int > vertexIndices, uvIndices, normalIndices;
        std::vector< glm::vec4 > temp_vertices;
        //std::vector< glm::vec2 > temp_uvs;
        //std::vector< glm::vec3 > temp_normals;
        std::vector <glm::vec4> vertices;
        //std::vector <glm::vec3> vertexIndices;
        std::vector <std::vector<int>> vertexIndices;
        printf("Opening file\n");
        std::ifstream file("Source/canoe.obj");
        printf("file open\n");

        while( true ) {

                //char lineHeader[128];
                // read the first word of the line
                //int res = fscanf(file, "%s", lineHeader);
                std::string lineHeader;
                file >> lineHeader;

                if (file.eof())
                        break;

                if ( lineHeader.compare("v") == 0 ) {

                        glm::vec4 vertex;
                        file >> vertex.x >> vertex.y >> vertex.z;
                        temp_vertices.push_back(vertex);

                } else if ( lineHeader.compare("f" ) == 0 ) {

                  unsigned int vertexIndex[3];
                  file >> vertexIndex[0] >> vertexIndex[1] >> vertexIndex[2];
                  std::vector<int> vertexVec;
                  vertexVec.push_back(vertexIndex[0]);
                  vertexVec.push_back(vertexIndex[1]);
                  vertexVec.push_back(vertexIndex[2]);
                  vertexIndices.push_back(vertexVec);
          }
        }

        triangles.reserve( 5000 );

        vec3 cyan(   0.15f, 0.75f, 0.75f );
        for( unsigned int i=0; i<vertexIndices.size(); i++ ){
                triangles.push_back( Triangle(vec4(temp_vertices[vertexIndices[i][0] - 1]), vec4(temp_vertices[vertexIndices[i][1] - 1]), vec4(temp_vertices[vertexIndices[i][2] - 1]), cyan));

        }

}
