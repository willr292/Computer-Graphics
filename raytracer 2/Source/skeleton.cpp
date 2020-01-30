#include <iostream>
#include <glm/glm.hpp>
#include "SDL.h"
#include "SDLauxiliary.h"
#include "TestModelH.h"
#include <stdint.h>
#include <limits.h>
#include <math.h>
#include <vector>
#include <random>
#include <omp.h>
#include <time.h>
#include <unistd.h>
#include "objLoader.h"
//#define _USE_MATH_DEFINES
//#include <cmath>

using namespace std;

using glm::vec3;
using glm::mat3;
using glm::vec4;
using glm::mat4;

/* ----------------------------------------------------------------------------*/
/* STRUCTS                                                                   */
struct Intersection
{
        vec3 position;
        float distance;
        int triangleIndex;
        Triangle triangle;
        Intersection() : position(vec3(0,0,0)), distance(0), triangle(Triangle(vec4(0,0,0,0), vec4(0,0,0,0), vec4(0,0,0,0), vec3(0,0,0)))
        {
        }
};
struct Light
{
        vec3 position;
        vec3 color;
};
/* ----------------------------------------------------------------------------*/
/* DEFINES                                                                   */
//#define SCREEN_WIDTH 320
//#define SCREEN_HEIGHT 256
#define SCREEN_WIDTH 640
#define SCREEN_HEIGHT 512
//1280, 1024, 2560, 2048
#define FULLSCREEN_MODE false
/* ----------------------------------------------------------------------------*/
/* VECTORS                                                                   */
vector<Triangle> triangles;
std::vector<int> indexes;
std::vector<int> distances;
vec4 cameraPos(0,0,-2.5,1);
vec4 up(0,0,1,0);
//vec4 Right(   R[0][0], R[0][1], R[0][2], 1 );
//vec4 Down(    R[1][0], R[1][1], R[1][2], 1 );
//vec4 Forward( R[2][0], R[2][1], R[2][2], 1 );
vec4 d;
vec4 d1;
vec4 d2;
vec4 d3;
vec4 d4;
vec4 lightPos( 0, -0.5, -0.7, 1.0 );
//vec4 lightPos( 0.5, 0.4, -1, 1.0 );
vec4 OGlightPos( 0, -0.5, -0.7, 1.0 );
vec4 OGcameraPos(0,0,-2.5,1);
vec3 lightColor = 14.f * vec3( 1, 1, 1 );
vec3 DirectLight( const Intersection& i, mat4 rotationandtranslation);
vec3 indirectLight = 0.05f*vec3( 1, 1, 1 );
glm::mat4x4 cameraPosMat(0,0,0,0,0,0,0,0,0,0,0,0,0,0,-2.5,1);
glm::mat4x4 cameraPosMat2(0,0,0,0,0,0,0,0,0,0,0,0,0,0,-2.5,1);
/* ----------------------------------------------------------------------------*/
/* GLOBAL VARIABLES                                                          */
float m = std::numeric_limits<float>::max();
//float focal_length = SCREEN_WIDTH / 2.0f;
float focal_length = 0.75;
mat4 R;
float yaw = 0;
float greenfloor = 0.5f;
float power = 25;
float refraction_air = 1.0;
float refraction_glass = 1.5;
bool openmpon = false;
//unsigned int microseconds = 1000000;
/* ----------------------------------------------------------------------------*/
/* FUNCTIONS                                                                   */

void Update(vec4 camera, vec4 direction, mat4 rotationandtranslation, vec4 lightposition);
void Draw(screen* screen, mat4 rotationandtranslation);
mat4 RotationMatrix(mat4 matrix, float angle);
vec3 castRay(const Intersection& i);
vec3 castSpecular(const Intersection& i);
vec3 castRefraction(const Intersection& i);
void TransformationMatrix(glm::mat4x4 cameraPosMat, glm::mat4 R, glm::mat4x4 cameraPosMat2, glm::mat4x4 finalresult );
bool ClosestIntersection(vec4 start, vec4 dir, const vector<Triangle>& triangles, Intersection& closestIntersection, mat4 rotationandtranslation );

int main( int argc, char* argv[] )
{
        R[0]=vec4(cos(yaw),0,-sin(yaw),0);
        R[1]=vec4(0,1,0,0);
        R[2]=vec4(sin(yaw),0,cos(yaw),0);
        R[3]=vec4(0,0,0,1);
        LoadTestModel(triangles);
        std::vector < glm::vec3 > out_vertices;
        std::vector < glm::vec2 > out_uvs;
        std::vector < glm::vec3 > out_normals;
        loadOBJ(triangles);
        screen *screen = InitializeSDL( SCREEN_WIDTH, SCREEN_HEIGHT, FULLSCREEN_MODE );
        while( NoQuitMessageSDL() )
        {
                Update(cameraPos,d,R,lightPos);
                Draw(screen,R);
                SDL_Renderframe(screen);
                //usleep(microseconds);
        }
        printf("drawing done");

        SDL_SaveImage( screen, "screenshot.bmp" );

        KillSDL(screen);
        return 0;
}

/*Place your drawing here*/
void Draw(screen* screen, mat4 rotationandtranslation)
{
        srand( unsigned(time(NULL)));
        std::random_device rand_dev;
        std::mt19937 generator(rand_dev());
        std::uniform_real_distribution<double>  distr(0, 0.4);
        vec3 black(0.0f,0.0f,0.0f);
        vec3 colour1(0.0f,0.0f,0.0f);
        vec3 colour2(0.0f,0.0f,0.0f);
        vec3 colour3(0.0f,0.0f,0.0f);
        vec3 colour4(0.0f,0.0f,0.0f);
        if(openmpon == true) {
        #pragma omp parallel for schedule(dynamic,1) collapse(2)
                for(int y = 0; y < SCREEN_HEIGHT; y++) {
                        for(int x = 0; x < SCREEN_WIDTH; x++) {
                                d1[0] = (x+distr(generator) - SCREEN_WIDTH/2.0)/SCREEN_WIDTH;
                                d1[1] = (y+distr(generator) - SCREEN_HEIGHT/2.0)/SCREEN_HEIGHT;
                                d1[2] = focal_length;
                                d2[0] = (x-distr(generator) - SCREEN_WIDTH/2.0)/SCREEN_WIDTH;
                                d2[1] = (y-distr(generator) - SCREEN_HEIGHT/2.0)/SCREEN_HEIGHT;
                                d2[2] = focal_length;
                                d3[0] = (x+distr(generator) - SCREEN_WIDTH/2.0)/SCREEN_WIDTH;
                                d3[1] = (y-distr(generator) - SCREEN_HEIGHT/2.0)/SCREEN_HEIGHT;
                                d3[2] = focal_length;
                                d4[0] = (x-distr(generator) - SCREEN_WIDTH/2.0)/SCREEN_WIDTH;
                                d4[1] = (y+distr(generator) - SCREEN_HEIGHT/2.0)/SCREEN_HEIGHT;
                                d4[2] = focal_length;
                                d1 = glm::normalize(d1);
                                d2 = glm::normalize(d2);
                                d3 = glm::normalize(d3);
                                d4 = glm::normalize(d4);
                                Intersection closestIntersection1;
                                Intersection closestIntersection2;
                                Intersection closestIntersection3;
                                Intersection closestIntersection4;
                                bool intersected1 = ClosestIntersection(cameraPos,R*d1,triangles,closestIntersection1,R);
                                bool intersected2 = ClosestIntersection(cameraPos,R*d2,triangles,closestIntersection2,R);
                                bool intersected3 = ClosestIntersection(cameraPos,R*d3,triangles,closestIntersection3,R);
                                bool intersected4 = ClosestIntersection(cameraPos,R*d4,triangles,closestIntersection4,R);
                                if(intersected1 == false || intersected2 == false || intersected3 == false || intersected4 == false) {
                                        PutPixelSDL(screen, x, y, black);
                                }
                                else{
                                        vec3 colour1 = triangles.at(closestIntersection1.triangleIndex).color;
                                        vec3 colour2 = triangles.at(closestIntersection2.triangleIndex).color;
                                        vec3 colour3 = triangles.at(closestIntersection3.triangleIndex).color;
                                        vec3 colour4 = triangles.at(closestIntersection4.triangleIndex).color;

                                        vec3 illumination1 = DirectLight(closestIntersection1,R);
                                        vec3 illumination2 = DirectLight(closestIntersection2,R);
                                        vec3 illumination3 = DirectLight(closestIntersection3,R);
                                        vec3 illumination4 = DirectLight(closestIntersection4,R);


                                        vec3 directPlusIndirect1 = (illumination1 + indirectLight);
                                        vec3 directPlusIndirect2 = (illumination2 + indirectLight);
                                        vec3 directPlusIndirect3 = (illumination3 + indirectLight);
                                        vec3 directPlusIndirect4 = (illumination4 + indirectLight);

                                        vec3 finalLight1 = directPlusIndirect1 * colour1 * vec3(9.0f);
                                        vec3 finalLight2 = directPlusIndirect2 * colour2 * vec3(9.0f);
                                        vec3 finalLight3 = directPlusIndirect3 * colour3 * vec3(9.0f);
                                        vec3 finalLight4 = directPlusIndirect4 * colour4 * vec3(9.0f);
                                        vec3 finalfinalLight = (finalLight1 + finalLight2 + finalLight3 + finalLight4)/vec3(4.0f);
                                        PutPixelSDL(screen, x, y, finalfinalLight);
                                        if (colour1 == vec3(0.74f, 0.75f, 0.5f) || colour2 == vec3(0.74f, 0.75f, 0.5f) || colour3 == vec3(0.74f, 0.75f, 0.5f) || colour4 == vec3(0.74f, 0.75f, 0.5f)) {
                                                vec3 reflectedColour = castRay(closestIntersection1);
                                                vec3 colour = /*(vec3(greenfloor) * colour1) + */ (vec3(1.0f) * reflectedColour);
                                                vec3 finalLight = illumination1 * colour * vec3(14.0f);
                                                PutPixelSDL(screen, x, y, finalLight);
                                        }
                                        if (colour1 == vec3( 0.63f, 0.06f, 0.04f) || colour2 == vec3( 0.63f, 0.06f, 0.04f) || colour3 == vec3( 0.63f, 0.06f, 0.04f) || colour4 == vec3( 0.63f, 0.06f, 0.04f)) {
                                                vec3 specularColour = castSpecular(closestIntersection1);
                                                vec3 colour = colour1 + specularColour;
                                                vec3 finalLight = illumination1 * colour * vec3(14.0f);
                                                PutPixelSDL(screen, x, y, finalLight);
                                        }
                                        if (colour1 == vec3(   0.15f, 0.15f, 0.75f ) || colour2 == vec3(   0.15f, 0.15f, 0.75f ) || colour3 == vec3(   0.15f, 0.15f, 0.75f ) || colour4 == vec3(   0.15f, 0.15f, 0.75f )) {
                                                vec3 refractionColour = castRefraction(closestIntersection1);
                                                vec3 colour = refractionColour;
                                                vec3 finalLight = illumination1 * colour * vec3(14.0f);
                                                PutPixelSDL(screen, x, y, finalLight);
                                        }
                                }
                        }
                }
        } //openmp loop ending
        else {
                for(int y = 0; y < SCREEN_HEIGHT; y++) {
                        for(int x = 0; x < SCREEN_WIDTH; x++) {
                                d1[0] = (x+distr(generator) - SCREEN_WIDTH/2.0)/SCREEN_WIDTH;
                                d1[1] = (y+distr(generator) - SCREEN_HEIGHT/2.0)/SCREEN_HEIGHT;
                                d1[2] = focal_length;
                                d2[0] = (x-distr(generator) - SCREEN_WIDTH/2.0)/SCREEN_WIDTH;
                                d2[1] = (y-distr(generator) - SCREEN_HEIGHT/2.0)/SCREEN_HEIGHT;
                                d2[2] = focal_length;
                                d3[0] = (x+distr(generator) - SCREEN_WIDTH/2.0)/SCREEN_WIDTH;
                                d3[1] = (y-distr(generator) - SCREEN_HEIGHT/2.0)/SCREEN_HEIGHT;
                                d3[2] = focal_length;
                                d4[0] = (x-distr(generator) - SCREEN_WIDTH/2.0)/SCREEN_WIDTH;
                                d4[1] = (y+distr(generator) - SCREEN_HEIGHT/2.0)/SCREEN_HEIGHT;
                                d4[2] = focal_length;
                                d1 = glm::normalize(d1);
                                d2 = glm::normalize(d2);
                                d3 = glm::normalize(d3);
                                d4 = glm::normalize(d4);
                                Intersection closestIntersection1;
                                Intersection closestIntersection2;
                                Intersection closestIntersection3;
                                Intersection closestIntersection4;
                                bool intersected1 = ClosestIntersection(cameraPos,R*d1,triangles,closestIntersection1,R);
                                bool intersected2 = ClosestIntersection(cameraPos,R*d2,triangles,closestIntersection2,R);
                                bool intersected3 = ClosestIntersection(cameraPos,R*d3,triangles,closestIntersection3,R);
                                bool intersected4 = ClosestIntersection(cameraPos,R*d4,triangles,closestIntersection4,R);
                                if(intersected1 == false || intersected2 == false || intersected3 == false || intersected4 == false) {
                                        PutPixelSDL(screen, x, y, black);
                                }
                                else{
                                        vec3 colour1 = triangles.at(closestIntersection1.triangleIndex).color;
                                        vec3 colour2 = triangles.at(closestIntersection2.triangleIndex).color;
                                        vec3 colour3 = triangles.at(closestIntersection3.triangleIndex).color;
                                        vec3 colour4 = triangles.at(closestIntersection4.triangleIndex).color;

                                        vec3 illumination1 = DirectLight(closestIntersection1,R);
                                        vec3 illumination2 = DirectLight(closestIntersection2,R);
                                        vec3 illumination3 = DirectLight(closestIntersection3,R);
                                        vec3 illumination4 = DirectLight(closestIntersection4,R);


                                        vec3 directPlusIndirect1 = (illumination1 + indirectLight);
                                        vec3 directPlusIndirect2 = (illumination2 + indirectLight);
                                        vec3 directPlusIndirect3 = (illumination3 + indirectLight);
                                        vec3 directPlusIndirect4 = (illumination4 + indirectLight);

                                        vec3 finalLight1 = directPlusIndirect1 * colour1 * vec3(9.0f);
                                        vec3 finalLight2 = directPlusIndirect2 * colour2 * vec3(9.0f);
                                        vec3 finalLight3 = directPlusIndirect3 * colour3 * vec3(9.0f);
                                        vec3 finalLight4 = directPlusIndirect4 * colour4 * vec3(9.0f);
                                        vec3 finalfinalLight = (finalLight1 + finalLight2 + finalLight3 + finalLight4)/vec3(4.0f);
                                        PutPixelSDL(screen, x, y, finalfinalLight);
                                        if (colour1 == vec3(0.74f, 0.75f, 0.5f) || colour2 == vec3(0.74f, 0.75f, 0.5f) || colour3 == vec3(0.74f, 0.75f, 0.5f) || colour4 == vec3(0.74f, 0.75f, 0.5f)) {
                                                vec3 reflectedColour = castRay(closestIntersection1);
                                                vec3 colour = /*(vec3(greenfloor) * colour1) + */ (vec3(1.0f) * reflectedColour);
                                                vec3 finalLight = illumination1 * colour * vec3(14.0f);
                                                PutPixelSDL(screen, x, y, finalLight);
                                        }
                                        if (colour1 == vec3( 0.63f, 0.06f, 0.04f) || colour2 == vec3( 0.63f, 0.06f, 0.04f) || colour3 == vec3( 0.63f, 0.06f, 0.04f) || colour4 == vec3( 0.63f, 0.06f, 0.04f)) {
                                                vec3 specularColour = castSpecular(closestIntersection1);
                                                vec3 colour = colour1 + specularColour;
                                                vec3 finalLight = illumination1 * colour * vec3(14.0f);
                                                PutPixelSDL(screen, x, y, finalLight);
                                        }
                                        /*
                                           if (colour1 == vec3(   0.15f, 0.75f, 0.75f ) || colour2 == vec3(   0.15f, 0.75f, 0.75f ) || colour3 == vec3(   0.15f, 0.75f, 0.75f ) || colour4 == vec3(   0.15f, 0.75f, 0.75f )) {
                                                vec3 cyanColour = castRay(closestIntersection1);
                                                vec3 colour = cyanColour;
                                                vec3 finalLight = illumination1 * colour * vec3(14.0f);
                                                PutPixelSDL(screen, x, y, finalLight);
                                           }
                                         */
                                        if (colour1 == vec3(   0.15f, 0.15f, 0.75f ) || colour2 == vec3(   0.15f, 0.15f, 0.75f ) || colour3 == vec3(   0.15f, 0.15f, 0.75f ) || colour4 == vec3(   0.15f, 0.15f, 0.75f )) {
                                                vec3 refractionColour = castRefraction(closestIntersection1);
                                                vec3 colour = refractionColour;
                                                vec3 finalLight = illumination1 * colour * vec3(14.0f);
                                                PutPixelSDL(screen, x, y, finalLight);
                                        }
                                }
                        }
                }
        }
}

/*Place updates of parameters here*/
void Update(vec4 camera, vec4 dir, mat4 rotationandtranslation, vec4 lightposition)
{
        vec4 right(   rotationandtranslation[0][0], rotationandtranslation[0][1], rotationandtranslation[0][2], 1 );
        vec4 down(    rotationandtranslation[1][0], rotationandtranslation[1][1], rotationandtranslation[1][2], 1 );
        vec4 forward( rotationandtranslation[2][0], rotationandtranslation[2][1], rotationandtranslation[2][2], 1 );
        // Compute frame time:
        //static int t = SDL_GetTicks();
        //int t2 = SDL_GetTicks();
        //float dt = float(t2-t);
        //t = t2;
        glm::mat4x4 finalresult;
        const uint8_t *keystate = SDL_GetKeyboardState( NULL );
        if( keystate[SDL_SCANCODE_UP] ) {
                cameraPos = cameraPos + 0.1f*forward;
        }
        if( keystate[SDL_SCANCODE_DOWN] ) {
                cameraPos = cameraPos - 0.1f*forward;
        }
        if( keystate[SDL_SCANCODE_LEFT] ) {
                yaw += 0.01;
        }
        if( keystate[SDL_SCANCODE_RIGHT] ) {
                yaw -= 0.01;
        }
        if( keystate[SDL_SCANCODE_W] ) {
                lightPos = lightPos + 0.1f*forward;
        }
        if( keystate[SDL_SCANCODE_S] ) {
                lightPos = lightPos - 0.1f*forward;
        }
        if( keystate[SDL_SCANCODE_A] ) {
                lightPos = lightPos - 0.1f*right;
        }
        if( keystate[SDL_SCANCODE_D] ) {
                lightPos = lightPos + 0.1f*right;
        }
        if( keystate[SDL_SCANCODE_Q] ) {
                lightPos = lightPos + 0.1f*down;
        }
        if( keystate[SDL_SCANCODE_E] ) {
                lightPos = lightPos - 0.1f*down;
        }
        if( keystate[SDL_SCANCODE_R] ) {
                lightPos = OGlightPos;
                cameraPos = OGcameraPos;
                greenfloor = 0.5f;
        }
        if( keystate[SDL_SCANCODE_O] ) {
                cameraPos = OGcameraPos;
        }
        if( keystate[SDL_SCANCODE_G] ) {
                greenfloor = greenfloor + 0.1f;
        }
        if( keystate[SDL_SCANCODE_1] ) {
                power = power * 2;
        }
        if( keystate[SDL_SCANCODE_2] ) {
                power = power / 2;
        }
        if( keystate[SDL_SCANCODE_P] ) {
                openmpon = true;
        }
        if( keystate[SDL_SCANCODE_L] ) {
                openmpon = false;
        }
        R[0]=vec4(cos(yaw),0,-sin(yaw),0);
        R[1]=vec4(0,1,0,0);
        R[2]=vec4(sin(yaw),0,cos(yaw),0);
        R[3]=vec4(0,0,0,1);
}

vec3 DirectLight( const Intersection& i, mat4 rotationandtranslation)
{
        std::random_device rand_dev;
        std::mt19937 generator(rand_dev());
        std::uniform_real_distribution<double>  distr(0, 0.01);
        //u = -0.1+ (double)((2* 0.1+ 0.1)* 0.1* rand()/ (RAND_MAX+ 1.));
        vec3 result;
        vec3 result1;
        vec3 result2;
        vec3 result3;
        vec3 result4;
        vec3 result5;
        vec3 result6;
        vec3 result7;
        vec3 result8;
        vec4 D;
        vec3 position = i.position;
        Triangle triangle = i.triangle;
        vec3 normal(triangle.normal);
        float travel = glm::length(vec3(lightPos) - position);
        float A = (4 * 3.14 * (travel * travel));
        vec3 nhat = normal;
        vec3 rhat = glm::normalize(vec3(lightPos) - position);
        float product = max(dot(nhat, rhat),(float)0);
        for ( int i = 0; i < 3; i++) {
                result[i] = (product / A);
                result1[i] = (product / A);
                result2[i] = (product / A);
                result3[i] = (product / A);
                result4[i] = (product / A);
                result5[i] = (product / A);
                result6[i] = (product / A);
                result7[i] = (product / A);
                result8[i] = (product / A);
        }
        Intersection shadowIntersection;
        Intersection shadowIntersection1;
        Intersection shadowIntersection2;
        Intersection shadowIntersection3;
        Intersection shadowIntersection4;
        Intersection shadowIntersection5;
        Intersection shadowIntersection6;
        Intersection shadowIntersection7;
        Intersection shadowIntersection8;
        vec3 shadowray = glm::normalize(position - vec3(lightPos));
        vec3 shadowray1 = glm::normalize(position - (vec3(lightPos) + vec3(0.05,0.0,0.0)));
        vec3 shadowray2 = glm::normalize(position - (vec3(lightPos) + vec3(0.0,0.0,0.05)));
        vec3 shadowray3 = glm::normalize(position - (vec3(lightPos) + vec3(-0.05,0.0,0.0)));
        vec3 shadowray4 = glm::normalize(position - (vec3(lightPos) + vec3(0.05,0.0,0.05)));
        vec3 shadowray5 = glm::normalize(position - (vec3(lightPos) + vec3(0.05,0.0,-0.05)));
        vec3 shadowray6 = glm::normalize(position - (vec3(lightPos) + vec3(-0.05,0.0,0.05)));
        vec3 shadowray7 = glm::normalize(position - (vec3(lightPos) + vec3(-0.05,0.0,-0.05)));
        vec3 shadowray8 = glm::normalize(position - (vec3(lightPos) + vec3(0.0,0.0,-0.05)));
        //float shadowlength = glm::length(vec3(lightPos) - position);
        vec4 positionvec4(position[0],position[1],position[2],1);
        vec4 shadowrayvec4(shadowray[0],shadowray[1],shadowray[2],1);
        vec4 shadowrayvec4_1(shadowray1[0],shadowray1[1],shadowray1[2],1);
        vec4 shadowrayvec4_2(shadowray2[0],shadowray2[1],shadowray2[2],1);
        vec4 shadowrayvec4_3(shadowray3[0],shadowray3[1],shadowray3[2],1);
        vec4 shadowrayvec4_4(shadowray4[0],shadowray4[1],shadowray4[2],1);
        vec4 shadowrayvec4_5(shadowray5[0],shadowray5[1],shadowray5[2],1);
        vec4 shadowrayvec4_6(shadowray6[0],shadowray6[1],shadowray6[2],1);
        vec4 shadowrayvec4_7(shadowray7[0],shadowray7[1],shadowray7[2],1);
        vec4 shadowrayvec4_8(shadowray8[0],shadowray8[1],shadowray8[2],1);
        bool shadowintersected = ClosestIntersection(lightPos,shadowrayvec4,triangles,shadowIntersection,R);
        bool shadowintersected1 = ClosestIntersection(lightPos,shadowrayvec4_1,triangles,shadowIntersection1,R);
        bool shadowintersected2 = ClosestIntersection(lightPos,shadowrayvec4_2,triangles,shadowIntersection2,R);
        bool shadowintersected3 = ClosestIntersection(lightPos,shadowrayvec4_3,triangles,shadowIntersection3,R);
        bool shadowintersected4 = ClosestIntersection(lightPos,shadowrayvec4_4,triangles,shadowIntersection4,R);
        bool shadowintersected5 = ClosestIntersection(lightPos,shadowrayvec4_5,triangles,shadowIntersection5,R);
        bool shadowintersected6 = ClosestIntersection(lightPos,shadowrayvec4_6,triangles,shadowIntersection6,R);
        bool shadowintersected7 = ClosestIntersection(lightPos,shadowrayvec4_7,triangles,shadowIntersection7,R);
        bool shadowintersected8 = ClosestIntersection(lightPos,shadowrayvec4_8,triangles,shadowIntersection8,R);
        if (shadowintersected == true) {
                float shadowdistance = shadowIntersection.distance;
                if (shadowdistance < (travel*0.9)) {
                        result = vec3(0.0f);
                }
        }
        if (shadowintersected1 == true) {
                float shadowdistance = shadowIntersection1.distance;
                if (shadowdistance < (travel*0.9)) {
                        result1 = vec3(0.0f);
                }
        }
        if (shadowintersected2 == true) {
                float shadowdistance = shadowIntersection2.distance;
                if (shadowdistance < (travel*0.9)) {
                        result2 = vec3(0.0f);
                }
        }
        if (shadowintersected3 == true) {
                float shadowdistance = shadowIntersection3.distance;
                if (shadowdistance < (travel*0.9)) {
                        result3 = vec3(0.0f);
                }
        }
        if (shadowintersected4 == true) {
                float shadowdistance = shadowIntersection4.distance;
                if (shadowdistance < (travel*0.9)) {
                        result4 = vec3(0.0f);
                }
        }
        if (shadowintersected5 == true) {
                float shadowdistance = shadowIntersection5.distance;
                if (shadowdistance < (travel*0.9)) {
                        result5 = vec3(0.0f);
                }
        }
        if (shadowintersected6 == true) {
                float shadowdistance = shadowIntersection6.distance;
                if (shadowdistance < (travel*0.9)) {
                        result6 = vec3(0.0f);
                }
        }
        if (shadowintersected7 == true) {
                float shadowdistance = shadowIntersection7.distance;
                if (shadowdistance < (travel*0.9)) {
                        result7 = vec3(0.0f);
                }
        }
        if (shadowintersected8 == true) {
                float shadowdistance = shadowIntersection8.distance;
                if (shadowdistance < (travel*0.9)) {
                        result8 = vec3(0.0f);
                }
        }
        result = (result + result1 + result2 + result3 + result4 + result5 + result6 + result7 + result8) / vec3(9.0f);
        return result;
}


vec3 Reflection(const vec3 incident, const vec3 normal){
        return incident - 2 * dot(incident, normal) * normal;
}


vec3 LightReflection(const vec3 incident, const vec3 normal){
        return (2 * dot(incident, normal) * normal) - incident;
}

float clip(float n, float lower, float upper) {
        return std::max(lower, std::min(n, upper));
}

void Fresnel(const vec3 incident, const vec3 normal, const float &ior, float &kr){
        float cosine = clip(dot(incident, normal),-1,1);
        float etai = 1, etat = ior;
        if (cosine > 0) {
                std::swap(etai, etat);
        }
        // Compute sini using Snell's law
        float sin_T = etai / etat * sqrtf(std::max(0.f, 1 - cosine * cosine));
        // Total internal reflection
        if (sin_T >= 1) {
                kr = 1;
        }
        else {
                float cos_T = sqrtf(std::max(0.f, 1 - sin_T * sin_T));
                cosine = fabsf(cosine);
                float Rs = ((etat * cosine) - (etai * cos_T)) / ((etat * cosine) + (etai * cos_T));
                float Rp = ((etai * cosine) - (etat * cos_T)) / ((etai * cosine) + (etat * cos_T));
                kr = (Rs * Rs + Rp * Rp) / 2;
        }
}


vec3 castRay(const Intersection& i){
        //vec3 hitcolor = i.triangle.color;
        vec3 position = i.position;
        Triangle triangle = i.triangle;
        vec3 normal(triangle.normal);
        vec3 ray = glm::normalize(position - vec3(cameraPos));
        vec3 dir = glm::normalize(Reflection(ray,normal));
        vec4 newposition(position[0],position[1],position[2],1.0f);
        vec4 newdir(dir[0],dir[1],dir[2],1.0f);
        newposition = newposition + (vec4(0.01f)*newdir);
        Intersection reflectedIntersection;
        bool newintersect = ClosestIntersection(newposition,newdir,triangles,reflectedIntersection,R);
        if (newintersect == true) {
                vec3 hitcolor = triangles.at(reflectedIntersection.triangleIndex).color;
                if(i.triangleIndex == reflectedIntersection.triangleIndex) {
                }
                if(hitcolor == vec3(   0.15f, 0.15f, 0.75f )) {
                        hitcolor = castRefraction(reflectedIntersection);
                }
                if(hitcolor == vec3( 0.63f, 0.06f, 0.04f)) {
                        vec3 illumination = DirectLight(reflectedIntersection,R);
                        vec3 OGcolor = hitcolor;
                        vec3 specular = castSpecular(reflectedIntersection);
                        specular = hitcolor + specular;
                        hitcolor = illumination * OGcolor * vec3(14.0f);
                }
                return hitcolor;
        }
        else{
                return vec3(0.0f);
        }
}


vec3 castSpecular(const Intersection& i){
        vec3 position = i.position;
        Triangle triangle = i.triangle;
        vec3 normal(triangle.normal);
        //vec3 ray = glm::normalize(position - vec3(cameraPos));
        vec3 outray = glm::normalize(vec3(lightPos) - position);
        //vec3 lightray = glm::normalize(position - vec3(lightPos));
        vec3 reflectedLight = glm::normalize(LightReflection(outray,normal));
        double specularDouble = dot(outray,reflectedLight);
        vec3 specular(specularDouble,specularDouble,specularDouble);
        specular[0] = pow(specular[0],power);
        specular[1] = pow(specular[1],power);
        specular[2] = pow(specular[2],power);
        return specular;
}


vec3 castRefraction(const Intersection& i){
        Triangle triangle = i.triangle;
        vec3 position = i.position;
        vec4 newposition(position[0],position[1],position[2],1.0f);
        vec3 normal(triangle.normal);
        vec3 n = normal;
        vec3 knew(0.0f);
        vec3 incident = glm::normalize(position - vec3(cameraPos));
        vec4 newincident(incident[0],incident[1],incident[2],1.0f);
        newposition = newposition + (vec4(0.01f)*newincident);
        float cosi = clip(dot(incident, normal),-1,1);
        float ior = refraction_glass;
        float ndoti = dot(n,incident);
        float etai = 1;
        float etat = ior;
        if (ndoti < 0) {
                ndoti = -ndoti;
        }
        else {
                n = -normal;
                std::swap(etai, etat);
        }
        float eta = etai / etat;
        float k = 1 - eta * eta * (1 - cosi * cosi);
        if (k < 0) {
                // total internal reflection. There is no refraction in this case
                // knew = (0.0,0.0,0.0);
        }
        else {
                knew = eta * incident + (eta * ndoti - sqrt(k)) * n;
        }
        vec4 knewvec4(knew[0],knew[1],knew[2],1.0);
        Intersection refractedIntersection;
        //bool newintersect = ClosestIntersection(newposition,newincident,triangles,refractedIntersection,R);
        bool newintersect = ClosestIntersection(newposition,knewvec4,triangles,refractedIntersection,R);
        if (newintersect == true) {
                vec3 hitcolor = triangles.at(refractedIntersection.triangleIndex).color;
                //if(i.triangleIndex == refractedIntersection.triangleIndex) {
                //}
                if (hitcolor == vec3(   0.15f, 0.15f, 0.75f )) {
                        Triangle triangleNEW = refractedIntersection.triangle;
                        vec3 positionNEW = refractedIntersection.position;
                        vec4 newpositionNEW(positionNEW[0],positionNEW[1],positionNEW[2],1.0f);
                        vec3 normalNEW(triangleNEW.normal);
                        vec3 n = normalNEW;
                        vec3 knewNEW(0.0f);
                        vec3 incidentNEW = glm::normalize(positionNEW - position);
                        vec4 newincidentNEW(incidentNEW[0],incidentNEW[1],incidentNEW[2],1.0f);
                        newpositionNEW = newpositionNEW + (vec4(0.01f)*newincidentNEW);
                        float cosi = clip(dot(incidentNEW, normalNEW),-1,1);
                        float ior = refraction_glass;
                        float ndoti = dot(n,incidentNEW);
                        float etai = ior;
                        float etat = 1;
                        if (ndoti < 0) {
                                ndoti = -ndoti;
                        }
                        else {
                                n = -normalNEW;
                                std::swap(etai, etat);
                        }
                        float eta = etai / etat;
                        float k = 1 - eta * eta * (1 - cosi * cosi);
                        if (k < 0) {
                                // total internal reflection. There is no refraction in this case
                                // knew = (0.0,0.0,0.0);
                        }
                        else {
                                knewNEW = eta * incident + (eta * ndoti - sqrt(k)) * n;
                        }
                        vec4 knewNEWvec4(knewNEW[0],knewNEW[1],knewNEW[2],1.0);
                        Intersection refractedIntersectionNEW;
                        //bool newintersectNEW = ClosestIntersection(newpositionNEW,newincidentNEW,triangles,refractedIntersectionNEW,R);
                        bool newintersectNEW = ClosestIntersection(newpositionNEW,knewNEWvec4,triangles,refractedIntersectionNEW,R);
                        if (newintersectNEW == true) {
                                vec3 hitcolor = triangles.at(refractedIntersectionNEW.triangleIndex).color;
                                return hitcolor;
                        }
                        else{
                                return vec3(0.0f);
                        }

                }

                else {
                        return hitcolor;
                }
        }
        else{
                //return hitcolor;
                return vec3(0.0f);
        }

}

mat4 RotationMatrix(mat4 matrix, float angle){
        matrix[0][0] = cos(angle);
        matrix[0][1] = 0;
        matrix[0][2] = -sin(angle);

        matrix[1][0] = 0;
        matrix[1][1] = 1;
        matrix[1][2] = 0;

        matrix[2][0] = sin(angle);
        matrix[2][1] = 0;
        matrix[2][2] = cos(angle);
        return matrix;
}


void TransformationMatrix(glm::mat4x4 cameraPosMat, glm::mat4 R, glm::mat4x4 cameraPosMat2, glm::mat4x4 finalresult )
{
        glm::mat4x4 result;
        finalresult = cameraPosMat * R;
        finalresult = result * cameraPosMat2;
}


bool ClosestIntersection(
        vec4 start,
        vec4 dir,
        const vector<Triangle>& triangles,
        Intersection& closestIntersection,
        mat4 rotationandtranslation )
{
        vec4 right(   rotationandtranslation[0][0], rotationandtranslation[0][1], rotationandtranslation[0][2], 1 );
        vec4 down(    rotationandtranslation[1][0], rotationandtranslation[1][1], rotationandtranslation[1][2], 1 );
        vec4 forward( rotationandtranslation[2][0], rotationandtranslation[2][1], rotationandtranslation[2][2], 1 );
        float minindex = -1;
        float mindist = std::numeric_limits<float>::max();

        //#pragma omp parallel
        //#pragma omp for
        for(unsigned int i=0; i<triangles.size(); i++) {
                vec4 v0 = triangles[i].v0;
                vec4 v1 = triangles[i].v1;
                vec4 v2 = triangles[i].v2;

                vec3 e1 = vec3(v1.x-v0.x,v1.y-v0.y,v1.z-v0.z);
                vec3 e2 = vec3(v2.x-v0.x,v2.y-v0.y,v2.z-v0.z);
                vec3 b = vec3(start.x-v0.x,start.y-v0.y,start.z-v0.z);
                glm::vec3 d(dir);
                mat3 A( -d, e1, e2 );
                vec3 x = glm::inverse( A ) * b;
                vec3 normal(triangles[i].normal);


                if(x[1] >= 0 && x[2] >= 0 && (x[1] + x[2] <= 1) && x[0] >= 0) {
                        if(x[0] < mindist) {
                                mindist = x[0];
                                minindex = i;
                                closestIntersection.distance = x[0];
                                closestIntersection.position = vec3(start) + (x[0] * d);
                                closestIntersection.triangleIndex = i;
                                closestIntersection.triangle = triangles[i];
                        }

                }
        }
        if (minindex == -1) {
                return false;
        }
        else {
                return true;
        }
}
