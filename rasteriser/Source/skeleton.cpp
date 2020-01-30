#include <iostream>
#include <glm/glm.hpp>
#include <SDL.h>
#include "SDLauxiliary.h"
#include "TestModelH.h"
#include <stdint.h>
#include <glm/gtc/matrix_transform.hpp>
#include "objLoader.h"

using namespace std;
using glm::vec3;
using glm::mat3;
using glm::vec4;
using glm::mat4;
using glm::ivec2;
using glm::vec2;

vector<Triangle> triangles;

#define SCREEN_WIDTH 1280*2
#define SCREEN_HEIGHT 1024*2
//#define SCREEN_WIDTH 320
//#define SCREEN_HEIGHT 256
#define FULLSCREEN_MODE false

float focal_length = 1.0;
glm::vec4 cameraPos( 0, 0, -3.001, 1 );
glm::vec4 OGcameraPos( 0, 0, -3.001, 1 );
glm::mat4 R;
float yaw = 0;
glm::mat4x4 cameraPosMat(0,0,0,0,0,0,0,0,0,0,0,0,0,0,-3.001,1);
glm::mat4x4 cameraPosMat2(0,0,0,0,0,0,0,0,0,0,0,0,0,0,3.001,1);
glm::mat4x4 RR(0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1);
glm::mat4x4 M;
vec3 currentColor;
vec3 currentNormal;
vec3 currentReflectance;
glm::vec3 one(1.0,1.0,1.0);
float depthBuffer[SCREEN_HEIGHT][SCREEN_WIDTH];

struct Pixel
{
    int x;
    int y;
    float zinv;
    vec3 pos3d;
};

struct Vertex
{
    vec4 position;
};

vec4 lightPos(0,-0.5,-0.2,1);
vec4 OGlightPos(0,-0.5,-0.2,1);
vec3 lightPower = 14.0f*vec3( 1, 1, 1 );
vec3 indirectLightPowerPerArea = 0.5f*vec3( 1, 1, 1 );

/* ----------------------------------------------------------------------------*/
/* FUNCTIONS                                                                   */

void Update(mat4 rotationandtranslation);
void Draw(screen* screen,vector<Triangle> triangles, mat4 rotationandtranslation);
void TransformationMatrix(glm::mat4x4 cameraPosMat, glm::mat4 R, glm::mat4x4 cameraPosMat2, glm::mat4x4 finalresult );
void VertexShader( const Vertex& v, Pixel& p, mat4 rotationandtranslation );
void DrawLineSDL( screen *screen, ivec2 a, ivec2 b, vec3 color );
//void DrawPolygonEdges(screen* screen, const vector<vec4>& vertices, mat4 rotationandtranslation);
void ComputePolygonRows(const vector<Pixel>& vertexPixels, vector<Pixel>& leftPixels, vector<Pixel>& rightPixels);
void DrawRows(screen *screen, const vector<Pixel>& leftPixels, const vector<Pixel>& rightPixels, vec3 color );
void DrawPolygon(screen *screen, const vector<Vertex>& vertices, mat4 rotationandtranslation, vec3 currentColor);
void PixelShader(screen* screen, const Pixel& p );

/* ----------------------------------------------------------------------------*/

int main( int argc, char* argv[] )
{
        LoadTestModel(triangles);
        loadOBJ(triangles);
        screen *screen = InitializeSDL( SCREEN_WIDTH, SCREEN_HEIGHT, FULLSCREEN_MODE );

        R[0]=vec4(cos(yaw),0,-sin(yaw),0);
        R[1]=vec4(0,1,0,0);
        R[2]=vec4(sin(yaw),0,cos(yaw),0);
        R[3]=vec4(0,0,0,1);

        while( NoQuitMessageSDL() )
        {
                Update(R);
                Draw(screen,triangles,R);
                SDL_Renderframe(screen);
        }

        SDL_SaveImage( screen, "screenshot.bmp" );

        KillSDL(screen);
        return 0;
}

/*Place your drawing here*/
void Draw(screen* screen,vector<Triangle> triangles, mat4 rotationandtranslation) {
        /* Clear buffer */
        memset(screen->buffer, 0, screen->height*screen->width*sizeof(uint32_t));

        for( int y=0; y<SCREEN_HEIGHT; ++y ){
            for( int x=0; x<SCREEN_WIDTH; ++x ){
              depthBuffer[y][x] = 0;
            }
            }
        for( uint32_t i=0; i<triangles.size(); ++i )
        {
                vector<Vertex> vertices(3);
                vertices[0].position = triangles[i].v0;
                vertices[1].position = triangles[i].v1;
                vertices[2].position = triangles[i].v2;
                currentColor = triangles[i].color;
                currentNormal = vec3(triangles[i].normal);
                currentReflectance = one;
                DrawPolygon(screen, vertices, R, currentColor);
        }
}

/*Place updates of parameters here*/
void Update(mat4 rotationandtranslation)
{
        vec4 right(   rotationandtranslation[0][0], rotationandtranslation[0][1], rotationandtranslation[0][2], 1 );
        vec4 down(    rotationandtranslation[1][0], rotationandtranslation[1][1], rotationandtranslation[1][2], 1 );
        vec4 forward( rotationandtranslation[2][0], rotationandtranslation[2][1], rotationandtranslation[2][2], 1 );
        static int t = SDL_GetTicks();
        /* Compute frame time */
        int t2 = SDL_GetTicks();
        //float dt = float(t2-t);
        t = t2;
        glm::mat4x4 finalresult;

        const uint8_t *keystate = SDL_GetKeyboardState( NULL );
        if( keystate[SDL_SCANCODE_LEFT] ) {
                yaw += 0.01;
        }
        if( keystate[SDL_SCANCODE_RIGHT] ) {
                yaw -=0.01;
        }
        if( keystate[SDL_SCANCODE_UP] ) {
                //cameraPos[1] = cameraPos[1] - 0.1;
                cameraPos = cameraPos + 0.1f*forward;
        }
        if( keystate[SDL_SCANCODE_DOWN] ) {
                //cameraPos[1] = cameraPos[1] + 0.1;
                cameraPos = cameraPos - 0.1f*forward;
        }
        if( keystate[SDL_SCANCODE_S] ) {
                lightPos = lightPos + vec4(0.1f)*down;
        }
        if( keystate[SDL_SCANCODE_W] ) {
                lightPos = lightPos - vec4(0.1f)*down;
        }
        if( keystate[SDL_SCANCODE_A] ) {
                lightPos = lightPos - vec4(0.1f)*right;
        }
        if( keystate[SDL_SCANCODE_D] ) {
                lightPos = lightPos + vec4(0.1f)*right;
        }
        if( keystate[SDL_SCANCODE_Q] ) {
                lightPos = lightPos + vec4(0.1f)*forward;
        }
        if( keystate[SDL_SCANCODE_E] ) {
                lightPos = lightPos - vec4(0.1f)*forward;
        }
        if( keystate[SDL_SCANCODE_O] ) {
                lightPos = OGlightPos;
                cameraPos = OGcameraPos;
                yaw = 0.0;
        }
        R[0]=vec4(cos(yaw),0,-sin(yaw),0);
        R[1]=vec4(0,1,0,0);
        R[2]=vec4(sin(yaw),0,cos(yaw),0);
        R[3]=vec4(0,0,0,1);

        TransformationMatrix(cameraPosMat, R, cameraPosMat2, finalresult );
}


void TransformationMatrix(glm::mat4x4 cameraPosMat, glm::mat4 R, glm::mat4x4 cameraPosMat2, glm::mat4x4 finalresult )
{
        glm::mat4x4 result;
        result = cameraPosMat * R;
        finalresult = result * cameraPosMat2;

}

void VertexShader( const Vertex& v, Pixel& p, mat4 rotationandtranslation )
{
        vec3 newpos = vec3(v.position);
        vec3 vnew2 = newpos - vec3(cameraPos);
        vec4 vRotated = rotationandtranslation*v.position;
        vnew2 = mat3(rotationandtranslation)*vnew2;
        vec4 vnew = vRotated - cameraPos;
        p.x = SCREEN_WIDTH * (focal_length * (vnew[0] / vnew[2])) + (SCREEN_WIDTH / 2.0f);
        p.y = SCREEN_HEIGHT * (focal_length * (vnew[1] / vnew[2])) + (SCREEN_HEIGHT / 2.0f);
        p.zinv = 1/vnew2.z;
        p.pos3d = newpos/vec3(vnew2.z);
        //printf("p.x = %d",p.x);
        //printf("p.y = %d",p.y);
}


void PixelShader(screen* screen, const Pixel& p )
{
    int x = p.x;
    int y = p.y;
    if( p.zinv > depthBuffer[y][x] )
    {
        depthBuffer[y][x] = p.zinv;
        vec3 power = p.pos3d * (1/p.zinv);
        vec3 nhat = currentNormal;
        vec3 rhat = glm::normalize(vec3(lightPos) - power);
        float r = glm::length(rhat);
        rhat = rhat/r;
        //float P = 1/(r * r);
        float A = 4 * 3.14 * (r * r);
        vec3 B = lightPower / A;
        vec3 product = B*max(dot(nhat, rhat),(float)0);
        vec3 Reflected = vec3(1.0,1.0,1.0) * (product + indirectLightPowerPerArea);
        PutPixelSDL( screen, x, y, Reflected*currentColor );
    }
}

/*
void Interpolate( ivec2 a, ivec2 b, vector<ivec2>& result )
{
        int N = result.size();
        glm::vec2 step = glm::vec2(b-a) / float(max(N-1,1));
        glm::vec2 current( a );
        for( int i=0; i<N; ++i )
        {
                result[i] = current;
                current += step;
        }
}
*/
void Interpolate( Pixel a, Pixel b, vector<Pixel>& result )
{
        int N = result.size();
        glm::vec3 change = glm::vec3(b.x-a.x,b.y-a.y,b.zinv - a.zinv) / float(max(N-1,1));
        glm::vec3 position( a.x,a.y,a.zinv );
        glm::vec3 lightInterpolate = glm::vec3(b.pos3d - a.pos3d)/float(max(N-1,1));
        glm::vec3 currentIllumination = a.pos3d;
        for( int i=0; i<N; i++ )
        {
                result[i].x = position.x;
                result[i].y = position.y+0.5;
                result[i].zinv = position.z;
                result[i].pos3d = currentIllumination;
                position += change;
                currentIllumination += lightInterpolate;
        }
}
/*
void DrawLineSDL( screen *screen, ivec2 a, ivec2 b, vec3 color ){
        ivec2 delta = glm::abs( a - b );
        int pixels = glm::max( delta.x, delta.y ) + 1;
        vector<ivec2> line( pixels );
        Interpolate( a, b, line );
        //for(int i = 0; i < line.size(); i++){
        //  printf("x: %d y: %d\n",line[i].x,line[i].y);
        //}
        for(int i = 0; i < line.size(); i++) {
                PutPixelSDL(screen, line[i].x, line[i].y, color);
        }
}
*/
/*
void DrawPolygonEdges(screen* screen, const vector<vec4>& vertices, mat4 rotationandtranslation)
{
        int V = vertices.size();
        // Transform each vertex from 3D world position to 2D image position:
        vector<ivec2> projectedVertices( V );
        for( int i=0; i<V; ++i )
        {
                VertexShader( vertices[i], projectedVertices[i], rotationandtranslation);
        }
        // Loop over all vertices and draw the edge from it to the next vertex:
        for( int i=0; i<V; ++i )
        {
                int j = (i+1)%V; // The next vertex
                vec3 color( 1, 1, 1 );
                DrawLineSDL( screen, projectedVertices[i], projectedVertices[j], color );
        }
}
*/
void ComputePolygonRows(
        const vector<Pixel>& vertexPixels,
        vector<Pixel>& leftPixels,
        vector<Pixel>& rightPixels )
{
        int Ymaxrows = vertexPixels[0].y;
        int Yminrows = vertexPixels[0].y;
        // 1. Find max and min y-value of the polygon
        //    and compute the number of rows it occupies.
        //ivec2 a = vertexPixels[0];
        //ivec2 b = vertexPixels[1];
        //ivec2 c = vertexPixels[2];

        for(int i=1; i< (int)vertexPixels.size(); i++) {
                if(vertexPixels[i].y>Ymaxrows) {
                        Ymaxrows=vertexPixels[i].y;
                }
                if(vertexPixels[i].y<Yminrows) {
                        Yminrows=vertexPixels[i].y;
                }
        }

        int ROWS = Ymaxrows - Yminrows + 1;
        //printf("hi computePolygonrows 1\n");
        // 2. Resize leftPixels and rightPixels
        //    so that they have an element for each row.

        leftPixels.resize(ROWS);
        rightPixels.resize(ROWS);

        //printf("hi computePolygonrows 2\n");
        // 3. Initialize the x-coordinates in leftPixels
        //    to some really large value and the x-coordinates
        //    in rightPixels to some really small value.

        for( int i=0; i<ROWS; ++i )
        {
                leftPixels[i].x  = +numeric_limits<int>::max();
                rightPixels[i].x = -numeric_limits<int>::max();
        }
        for( int i=0; i<(int)vertexPixels.size(); i++ )
        {
                int j = (i+1)%(int)vertexPixels.size();
                vec3 size = glm::abs( vec3(vertexPixels[i].x-vertexPixels[j].x,vertexPixels[i].y-vertexPixels[j].y,vertexPixels[i].zinv-vertexPixels[j].zinv));
                int pixels = glm::max( size.x, size.y) + 1;
                vector<Pixel> line(pixels);
                Interpolate( vertexPixels[i], vertexPixels[j], line );
                for(int k=0; k<pixels; k++) {

                        if( leftPixels[line[k].y-Yminrows].x > line[k].x) {
                                leftPixels[line[k].y-Yminrows] = line[k];
                        }
                        if( rightPixels[line[k].y-Yminrows].x< line[k].x) {
                                rightPixels[line[k].y-Yminrows] = line[k];
                        }
                }
        }
        //printf("hi computePolygonrows 4.2\n");
}

void DrawRows(screen *screen, const vector<Pixel>& leftPixels, const vector<Pixel>& rightPixels, vec3 currentColor ){
        for (int i = 0; i < (int)leftPixels.size(); i++) {
                vec3 size = glm::abs( vec3(leftPixels[i].x-rightPixels[i].x,leftPixels[i].y-rightPixels[i].y, leftPixels[i].zinv-rightPixels[i].zinv));
                int pixels = glm::max( size.x, size.y) + 1;
                if(pixels>1) {
                        //printf("hi 1\n");
                        vector<Pixel> line(pixels);
                        line.resize(pixels);
                        Interpolate( leftPixels[i], rightPixels[i], line );
                        for(int j=0; j<pixels; j++) {
                                if(line[j].x >= 0 && line[j].x <SCREEN_WIDTH && line[j].y>=0 && line[j].y<SCREEN_HEIGHT) {
                                  PixelShader(screen,line[j]);
                                }
                        }
                }
        }
}

void DrawPolygon(screen *screen, const vector<Vertex>& vertices, mat4 rotationandtranslation, vec3 currentColor){
        int V = vertices.size();
        vector<Pixel> vertexPixels( V );
        //printf("hi DrawPolygon 1\n");
        for( int i=0; i<V; i++ ) {
                VertexShader( vertices[i], vertexPixels[i], rotationandtranslation);
        }
        //  printf("hi DrawPolygon 2\n");
        vector<Pixel> leftPixels;
        vector<Pixel> rightPixels;
        ComputePolygonRows( vertexPixels, leftPixels, rightPixels );
        //printf("hi DrawPolygon 3\n");
        DrawRows(screen, leftPixels, rightPixels, currentColor );
        //printf("hi DrawPolygon 4\n");
}
