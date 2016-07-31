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

#include <jni.h>
#include "simpleARClass.h"

#ifdef __cplusplus
extern "C" {
#endif

extern SimpleARClass *gSimpleARObject;

/**
 * Create a RGB image from camera's preview data and send it to native class
 */
JNIEXPORT void JNICALL
Java_com_anandmuralidhar_simplearandroid_CameraClass_SendCamImageToNative(JNIEnv *env,
                                                                              jobject instance,
                                                                              jbyteArray data_,
                                                                              jint previewHeight,
                                                                              jint previewWidth) {
    if(gSimpleARObject == NULL) {
        return;
    }

    jbyte *data = env->GetByteArrayElements(data_, NULL);

    // Android returns data in NV21 format, convert it to RGB
    cv::Mat cameraNV21Image(previewHeight * 1.5, previewWidth, CV_8UC1, data);
    cv::Mat cameraRGBImage;
    cv::cvtColor(cameraNV21Image, cameraRGBImage, CV_YUV2RGB_NV21, 3);

    gSimpleARObject->ProcessCameraImage(cameraRGBImage);

    env->ReleaseByteArrayElements(data_, data, 0);
}

#ifdef __cplusplus
}
#endif

