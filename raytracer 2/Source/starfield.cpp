#include <iostream>
#include <glm/glm.hpp>
#include <SDL.h>
#include "SDLauxiliary.h"
#include "TestModel.h"
#include <stdint.h>

using namespace std;
using glm::vec3;
using glm::mat3;

#define SCREEN_WIDTH 320
#define SCREEN_HEIGHT 256
#define FULLSCREEN_MODE false

  vector<vec3> GenerateStars(int num);
  const float v = 0.0001;

/* ----------------------------------------------------------------------------*/
/* GLOBAL VARIABLES                                                            */
int t;
/* ----------------------------------------------------------------------------*/
/* FUNCTIONS                                                                   */

void Update(vector<vec3>& stars);
void Draw(screen* screen,vector<vec3>& stars);
//void Interpolate( float a, float b, vector<float>& result );
void Interpolate( vec3 a, vec3 b, vector<vec3>& result );
vector<vec3> CreateStars(int num);

// vector<float> result( 10 ); // Create a vector width 10 floats

int main( int argc, char* argv[] )
{

  screen *screen = InitializeSDL( SCREEN_WIDTH, SCREEN_HEIGHT, FULLSCREEN_MODE );
  t = SDL_GetTicks();	/*Set start value for timer.*/


  vector<vec3> stars = CreateStars(1000);
  //cout << "\n";

  while( NoQuitMessageSDL() )
    {
      Draw(screen,stars);
      Update(stars);
      SDL_Renderframe(screen);
    }

  SDL_SaveImage( screen, "screenshot.bmp" );

  KillSDL(screen);
  return 0;
}

vector<vec3> CreateStars(int num) {
    vector<vec3> stars;
    for (int a = 0; a < num; a++) {
        float x = float(rand()) / float(RAND_MAX);
        float y = float(rand()) / float(RAND_MAX);
        float z = float(rand()) / float(RAND_MAX);
        stars.push_back(vec3(x*2-1, y*2-1, z));
    }
    return stars;
}

/*Place your drawing here*/
void Draw(screen* screen,vector<vec3>& stars)
{
  memset(screen->buffer, 0, SCREEN_WIDTH*SCREEN_HEIGHT*sizeof(uint32_t));
  float focal_length = SCREEN_WIDTH / 2.0f;
  for(auto star : stars){
      float u = focal_length * (star.x / star.z) + (SCREEN_WIDTH / 2.0f);
      float v = focal_length * (star.y / star.z) + (SCREEN_HEIGHT / 2.0f);
      vec3 colour = 0.2f * vec3(1, 1, 1) / (star.z * star.z);

      PutPixelSDL(screen, (int)u, (int)v, colour);
  }
}


void Interpolate(vec3 a, vec3 b, vector<vec3> &result) {
    unsigned long size = result.size();

    if (size == 1) {
        result[0] = (a + b) / 2.0f;
    } else {
        vec3 step = (b - a) / (float)(size-1);
        //cout << "step: (" << step.x << "," << step.y << "," << step.z << ")" << endl;
        for (int i = 0; i < result.size(); i++) {
            result[i] = a + (step * (float)i);
        }
    }

}
/*Place updates of parameters here*/
void Update(vector<vec3>& stars){
static int t = SDL_GetTicks();
  int t2 = SDL_GetTicks();
  float dt = float(t2-t);
  t = t2;

  for (int i = 0; i < stars.size(); i++) {
    stars[i].z -= v * dt;
    if (stars[i].z <= 0) {
        stars[i].z += 1;
    }
    if (stars[i].z > 1) {
        stars[i].z -= 1;
    }
}
}
