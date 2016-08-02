SimpleARAndroid
===============
In this post we build a simple AR app that positions a 3D model in a 
particular location in the scene. We create a marker based on the scene’s
contents. In particular we show how to:
- Match feature points with a reference location in the scene
- Construct the camera intrinsic matrix.
- Estimate the pose in each frame
- Display a 3D model at the reference location

You can point at the floor and double-tap to create a marker out of the
scene's contents. This marker image will be continuously tracked and a
3D model will be displayed at the original location. Please ensure that
you choose a reference marker location that has sufficent features.

The project will run on devices with ABI armeabi-v7a.

A blog describing this project:
http://www.anandmuralidhar.com/blog/android/simple-ar

License
-------

Copyright 2016 Anand Muralidhar

Licensed to the Apache Software Foundation (ASF) under one or more contributor
license agreements.  See the NOTICE file distributed with this work for
additional information regarding copyright ownership.  The ASF licenses this
file to you under the Apache License, Version 2.0 (the "License"); you may not
use this file except in compliance with the License.  You may obtain a copy of
the License at

http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
License for the specific language governing permissions and limitations under
the License.