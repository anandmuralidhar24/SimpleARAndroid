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

#include "myShader.h"
#include "simpleARClass.h"
#include <myJNIHelper.h>

/**
 * Class constructor
 */
SimpleARClass::SimpleARClass() {

    MyLOGD("SimpleARClass::SimpleARClass");
    initsDone = false;
    back = NULL;

    cornerDetector = cv::ORB::create(750); // choosing ORB detector with default parameters
    matcher        = cv::DescriptorMatcher::create("BruteForce-Hamming");

    modelObject = NULL;

    gravityMutex.unlock();
    gravity.assign(3, 0);
    gravity[2] = 1.0f;

    doubleTapAction     = false;
    trackingIsOn        = false;
    pnpResultIsValid    = false;
    renderModel         = false;
    newPnpResult        = false;
    previewScaleFactor  = 0.5; // camera image is downscaled to half its original size

    translationVector       = cv::Mat::zeros(3,1,CV_32F);
    translationVectorCopy   = cv::Mat::zeros(3,1,CV_32F);
    rotationVector          = cv::Mat::zeros(3,1,CV_32F);
    rotationVectorCopy      = cv::Mat::zeros(3,1,CV_32F);
}

SimpleARClass::~SimpleARClass() {

    MyLOGD("SimpleARClass::SimpleARClass");
    if(back) {
        delete back;
        back = NULL;
    }
    if (myGLCamera) {
        delete myGLCamera;
    }
    if (modelObject) {
        delete modelObject;
    }
}

/**
 * Perform inits, create objects for detecting corners and rendering image
 */
void SimpleARClass::PerformGLInits() {

    MyLOGD("SimpleARClass::PerformGLInits");

    MyGLInits();

    // create MyGLCamera object and set default position for the object
    myGLCamera = new MyGLCamera(cameraFOV,0);

    back = new BackTexture(cameraPreviewWidth*previewScaleFactor,
                           cameraPreviewHeight*previewScaleFactor);

    modelObject = new AssimpLoader();

    // extract the OBJ and companion files from assets
    // its a long list since ourWorld.obj has 6 textures corresponding to faces of the cube
    std::string objFilename, mtlFilename, texFilename;
    bool isFilesPresent  =
            gHelperObject->ExtractAssetReturnFilename("amenemhat/amenemhat.obj", objFilename) &&
            gHelperObject->ExtractAssetReturnFilename("amenemhat/amenemhat.mtl", mtlFilename) &&
            gHelperObject->ExtractAssetReturnFilename("amenemhat/amenemhat.jpg", texFilename);
    if( !isFilesPresent ) {
        MyLOGE("Model %s does not exist!", objFilename.c_str());
        return;
    }

    modelObject->Load3DModel(objFilename);

    CheckGLError("SimpleARClass::PerformGLInits");
    newCameraImage = false;
    initsDone = true;
}


/**
 * Render to the display
 */
void SimpleARClass::Render() {

    // clear the screen
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // render the camera image as the background texture
    cameraMutex.try_lock();
    if(newCameraImage) {
        back->LoadBackImg(cameraImageForBack);
    }
    newCameraImage = false;
    cameraMutex.unlock();
    back->Render();

    // render the 3D model if we have tracked reference marker in query image
    if(trackingIsOn) {

        pnpMutex.try_lock();
        if(newPnpResult) {
            if (pnpResultIsValid) {

                // make a copy of pnp result, it will be retained till result is updated again
                translationVectorCopy = translationVector.clone();
                rotationVectorCopy = rotationVector.clone();

                // flip OpenCV results to be consistent with OpenGL's coordinate system
                translationVectorCopy.at<double>(2, 0) = -translationVectorCopy.at<double>(2, 0);
                rotationVectorCopy.at<double>(0, 0) = -rotationVectorCopy.at<double>(0, 0);
                rotationVectorCopy.at<double>(1, 0) = -rotationVectorCopy.at<double>(1, 0);
                renderModel = true;

            } else {
                renderModel = false;
            }
            newPnpResult = false;
        }
        pnpMutex.unlock();


        cv::Mat defaultModelPosition = cv::Mat::zeros(3,1,CV_64F);
        defaultModelPosition.at<double>(2,0) = -CAM_HEIGHT_FROM_FLOOR;
        myGLCamera->UpdateModelMat(translationVectorCopy, rotationVectorCopy, defaultModelPosition);

        gravityMutex.lock();
        glm::mat4 mvpMat = myGLCamera->GetMVPAlignedWithGravity(gravity);
        gravityMutex.unlock();

        if (renderModel) {
            modelObject->Render3DModel(&mvpMat);
        } else {
//            MyLOGD("***not rendering model***");
        }
    }

    CheckGLError("SimpleARClass::Render");

}

/**
 * set the viewport, function is also called when user changes device orientation
 */
void SimpleARClass::SetViewport(int width, int height) {

    screenHeight = height;
    screenWidth = width;
    glViewport(0, 0, width, height);
    CheckGLError("Cube::SetViewport");

    myGLCamera->SetAspectRatio((float) 1280 / 720);
}

/**
 * Save camera image, detect feature points in it, highlight them, match them
 */
void SimpleARClass::ProcessCameraImage(cv::Mat cameraRGBImage) {

    cameraMutex.lock();

    cameraImageForBack = cameraRGBImage.clone();

    // resize the camera preview image to a smaller size to speedup processing
    cv::resize(cameraImageForBack,cameraImageForBack, cv::Size(),
               previewScaleFactor, previewScaleFactor);

    // OpenCV image needs to be flipped for OpenGL
    cv::flip(cameraImageForBack, cameraImageForBack, 0);

    if(doubleTapAction) {
        // if we are able to detect required number of feature points, then start tracking
        if(DetectKeypointsInReferenceImage()) {

            trackingIsOn = true;

        } else {

            trackingIsOn = false;

        }
        doubleTapAction = false;

    } else if(trackingIsOn) {

        // if enough feature points are detected in query image, then try to match them
        // else indicate to render loop that matching has failed for this frame
        if(MatchKeypointsInQueryImage()) {

            TrackKeypointsAndUpdatePose();

        } else {
            pnpMutex.lock();
            newPnpResult = true;        // new frame was processed ...
            pnpResultIsValid = false;   // ... but no match.
            pnpMutex.unlock();
        }

    } else {

        // simply highlight corners in the image
        DetectAndHighlightCorners();

    }

    newCameraImage = true; // indicate to Render() that a new image is available

    cameraMutex.unlock();
}

/**
 * Use the corner detector to find feature points and draw small circles around them
 */
void SimpleARClass::DetectAndHighlightCorners(){

    cornerDetector->detect(cameraImageForBack, keyPoints);
    for(int i=0;i<keyPoints.size();i++){

        cv::circle(cameraImageForBack, keyPoints[i].pt, 5, cv::Scalar(255,0,0));

    }
}

/**
 * Camera preview dimensions are saved -- used later to initialize BackTexture object
 */
void SimpleARClass::SetCameraParams(int cameraPreviewWidth, int cameraPreviewHeight,
                                    float cameraFOV) {

    this->cameraPreviewWidth = cameraPreviewWidth;
    this->cameraPreviewHeight = cameraPreviewHeight;
    this->cameraFOV = cameraFOV;
}

/**
 * reset model's position in double-tap
 */
void SimpleARClass::DoubleTapAction() {

    cameraMutex.lock();
    // set a flag to check if a new reference image can be saved
    doubleTapAction = true;
    cameraMutex.unlock();
}

/**
 * Copy gravity vector from sensor into private variable
 */
void SimpleARClass::UpdateGravity(float gx, float gy, float gz) {

    gravityMutex.try_lock();
    gravity[0] = gx;
    gravity[1] = gy;
    gravity[2] = gz;
    gravityMutex.unlock();
    return;
}

/**
 * Use OpenCV's feature detector to compute locations and descriptors of keypoints
 */
bool SimpleARClass::DetectKeypointsInReferenceImage() {

    //Detect feature points and descriptors in reference image
    cornerDetector->detectAndCompute(cameraImageForBack, cv::noArray(),
                                     referenceKeypoints, referenceDescriptors);
    MyLOGD("Number of feature points in source frame %d", (int)referenceKeypoints.size());

    if(referenceKeypoints.size() < MIN_KPS_IN_FRAME){
        return false;
    }

    // source gravity vector used to project keypoints on imaginary floor at certain depth
    gravityMutex.lock();
    sourceGravityVector.x = gravity[0];
    sourceGravityVector.y = gravity[1];
    sourceGravityVector.z = gravity[2];
    gravityMutex.unlock();

    return true;
}

/**
 * Match keypoints in new image with reference frame. Compute homography to determine inliers
 */
    bool SimpleARClass::MatchKeypointsInQueryImage() {

    std::vector<cv::KeyPoint> queryKeypoints;
    cv::Mat queryDescriptors;

    // compute keypoints and their descriptors in the query image
    cameraMutex.lock();
    cornerDetector->detectAndCompute(cameraImageForBack, cv::noArray(), queryKeypoints,
                                     queryDescriptors);
    cameraMutex.unlock();

    MyLOGD("Number of kps in query frame %d", (int) queryKeypoints.size());
    if (queryKeypoints.size() == 0) {
        MyLOGD("Not enough feature points in query image");
        return false;
    }

    std::vector<std::vector<cv::DMatch> > descriptorMatches;
    std::vector<cv::KeyPoint> sourceMatches, queryMatches;
    // knn-match with k = 2
    matcher->knnMatch(referenceDescriptors, queryDescriptors, descriptorMatches, 2);

    // save matches within a certain distance threshold
    for (unsigned i = 0; i < descriptorMatches.size(); i++) {
        if (descriptorMatches[i][0].distance < NN_MATCH_RATIO * descriptorMatches[i][1].distance) {
            sourceMatches.push_back(referenceKeypoints[descriptorMatches[i][0].queryIdx]);
            queryMatches.push_back(queryKeypoints[descriptorMatches[i][0].trainIdx]);
        }
    }
    MyLOGD("Number of kps whose descriptors match = %d", (int) sourceMatches.size());

    // compute homography to further prune outlier keypoints
    cv::Mat homography, inlierMask;
    if (sourceMatches.size() >= MIN_INLIER_COUNT) {
        homography = cv::findHomography(Keypoint2Point(sourceMatches),
                                        Keypoint2Point(queryMatches),
                                        cv::RANSAC, RANSAC_THRESH, inlierMask);
    } else {
        MyLOGD("Very few kps match, cannot proceed further!", (int) sourceMatches.size());
        return false;
    }

    if (homography.empty()) {
        MyLOGD("Could not determine homography!");
        return false;
    }

    sourceInlierKeypoints.clear();
    queryInlierKeypoints.clear();
    // retain inliers after computing homography
    for (unsigned i = 0; i < sourceMatches.size(); i++) {
        if (inlierMask.at<uchar>(i)) {
            sourceInlierKeypoints.push_back(sourceMatches[i]);
            queryInlierKeypoints.push_back(queryMatches[i]);
        }
    }
    MyLOGD("Number of kps match after homography filter = %d", (int) sourceInlierKeypoints.size());

    if (sourceInlierKeypoints.size() < MIN_INLIER_COUNT) {
        MyLOGD("Not enough kps match after homography filter!");
        return false;
    }
    MyLOGD("Success, Number of keypoint matches = %d", (int)sourceInlierKeypoints.size());

    // draw a rectangle marking reference frame in current image
    DrawShiftedCorners(cameraImageForBack, homography);
    return true;
}


/**
 * Call the pnp-based solver to estimate new pose
 */
void SimpleARClass::TrackKeypointsAndUpdatePose() {

    // Use inliers as reference points
    sourceInlierPoints  = Keypoint2Point(sourceInlierKeypoints);
    queryInlierPoints   = Keypoint2Point(queryInlierKeypoints);
    int num_reference_pts = sourceInlierKeypoints.size();

    // Project inlier points onto an imaginary floor to get their 3D locations
    sourceKeypointLocationsIn3D.clear();
    sourceKeypointLocationsIn3D = myGLCamera->GetProjectedPointsOnFloor(sourceInlierPoints,
                                                                        sourceGravityVector,
                                                                        CAM_HEIGHT_FROM_FLOOR,
                                                                        cameraImageForBack.cols,
                                                                        cameraImageForBack.rows);

    // construct the camera intrinsic matrix
    cv::Mat cameraMatrix = myGLCamera->ConstructCameraIntrinsicMatForCV(cameraImageForBack.cols,
                                                                        cameraImageForBack.rows);


    // estimate pose of query frame with solvepnp
    std::vector<float> distCoeffs; //null vector
    pnpMutex.lock();
    pnpResultIsValid = false;
    pnpResultIsValid = cv::solvePnP(sourceKeypointLocationsIn3D, queryInlierPoints,
                                    cameraMatrix, distCoeffs,
                                    rotationVector, translationVector);
    newPnpResult = true; // indicate to render loop that a new result is available
    pnpMutex.unlock();

    if(!pnpResultIsValid) {
        MyLOGD("No solution to pnp!");
    }
    else{
        PrintCVMat(translationVector.t());
        PrintCVMat(rotationVector.t());
    }
    return;
}
