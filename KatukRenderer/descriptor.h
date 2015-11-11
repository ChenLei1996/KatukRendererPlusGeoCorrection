#ifndef H_SCENE_DESCRIPTOR
#define H_SCENE_DESCRIPTOR
#include <string>

// initilize scene
// load obj files, set lights, camera
void initScene(const std::string& textureName);
void releaseScene();

#endif