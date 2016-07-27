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

// shader associated with BackTexture

attribute   vec2 vertexPosition;
varying     vec2 textureCoords;
const float EPS = 10e-3;

void main()
{
    // Z coordinate of gl_Position is chosen so that image lies at the back
    gl_Position     = vec4(vertexPosition.xy, 1.0 - EPS, 1.0);

    // vertexPosition range is [-1,1]. Convert to range [0,1] for reading texture
    textureCoords   = (vertexPosition + 1.) * 0.5;
}
