/*
 *    Copyright 2016 Anand Muralidhar
 *
 *    Licensed under the Apache License, Version 2.0 (the "License");
 *    you may not use this file except in compliance with the License.
 *    You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 *    Unless required by applicable law or agreed to in writing, software
 *    distributed under the License is distributed on an "AS IS" BASIS,
 *    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *    See the License for the specific language governing permissions and
 *    limitations under the License.
 */

#ifndef BACKTEXTURE_H
#define BACKTEXTURE_H

#include "myGLFunctions.h"
#include <opencv2/opencv.hpp>

class BackTexture{
public:
    BackTexture(int width, int height);
    void Render();
    bool LoadBackImg(cv::Mat backImage);
    int  GetWidth(){return width;}
    int  GetHeight(){return height;}

private:
    int width, height;

    GLuint  vertexBuffer;
    GLuint  vertexAttribute;
    GLuint  shaderProgramID;
    GLint   textureSamplerLocation;
    GLuint  textureNameInGL;

};
#endif //BACKTEXTURE_H
