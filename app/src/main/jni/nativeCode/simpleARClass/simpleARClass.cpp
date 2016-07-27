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
    matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");

    // create MyGLCamera object and set default position for the object
    myGLCamera = new MyGLCamera(36,0);
    float pos[]={0.,0.,0.,0.,0.,0.};
    std::copy(&pos[0], &pos[5], std::back_inserter(modelDefaultPosition));
    myGLCamera->SetModelPosition(modelDefaultPosition);

    modelObject = NULL;

    gravityMutex.unlock();
    gravity.assign(3, 0);
    gravity[2] = 1.0f;

    trackingIsOn = false;
    pnpResultIsValid = false;

    translationVector = cv::Mat::zeros(3,1,CV_32F);
    translationVectorCopy = cv::Mat::zeros(3,1,CV_32F);
    rotationVector = cv::Mat::zeros(3,1,CV_32F);
    rotationVectorCopy = cv::Mat::zeros(3,1,CV_32F);
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

    back = new BackTexture(cameraPreviewWidth/2, cameraPreviewHeight/2);

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

    cameraMutex.try_lock();
    if(newCameraImage) {
        back->LoadBackImg(cameraImageForBack);
    }
    newCameraImage = false;
    cameraMutex.unlock();
    back->Render();

    if(trackingIsOn) {


        pnpMutex.try_lock();
        if(pnpResultIsValid) {
            translationVectorCopy = translationVector.clone();
            rotationVectorCopy = rotationVector.clone();
        }
        pnpMutex.unlock();

        translationVectorCopy.at<double>(2,0) = -translationVectorCopy.at<double>(2,0);
        rotationVectorCopy.at<double>(0,0) = -rotationVectorCopy.at<double>(0,0);
        rotationVectorCopy.at<double>(1,0) = -rotationVectorCopy.at<double>(1,0);

        myGLCamera->UpdateModelMat(translationVectorCopy, rotationVectorCopy);

        gravityMutex.lock();
        glm::mat4 mvpMat = myGLCamera->GetMVPAlignedWithGravity(gravity);
        gravityMutex.unlock();

        modelObject->Render3DModel(&mvpMat);

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
 * Save camera image, detect feature points in it, highlight them
 */
void SimpleARClass::ProcessCameraImage(cv::Mat cameraRGBImage, int mPreview_width, int mPreview_height) {

    cameraMutex.lock();

    cameraImageForBack = cameraRGBImage.clone();
    cv::resize(cameraImageForBack,cameraImageForBack, cv::Size(), 0.5, 0.5);

    // OpenCV image needs to be flipped for OpenGL
    cv::flip(cameraImageForBack, cameraImageForBack, 0);

    if(trackingIsOn) {
        if(DetectKeypointsInQueryImage()) {
            TrackKeypointsAndUpdatePose();
        } else {
            pnpResultIsValid = false;
        }

    } else {
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
void SimpleARClass::SetCameraPreviewDims(int cameraPreviewWidth, int cameraPreviewHeight) {

    this->cameraPreviewWidth = cameraPreviewWidth;
    this->cameraPreviewHeight = cameraPreviewHeight;
}

/**
 * reset model's position in double-tap
 */
void SimpleARClass::DoubleTapAction() {

    if(DetectKeypointsInReferenceImage()) {

        trackingIsOn = true;

    } else {

        trackingIsOn = false;

    }
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

bool SimpleARClass::DetectKeypointsInReferenceImage() {

    //Detect feature points and descriptors in reference image
    cameraMutex.lock();
    cornerDetector->detectAndCompute(cameraImageForBack, cv::noArray(),
                                     referenceKeypoints, referenceDescriptors);
    cameraMutex.unlock();
    MyLOGD("Numer of feature points in source frame %d", (int)referenceKeypoints.size());

    if(referenceKeypoints.size() < MIN_KPS_IN_FRAME){
        return false;
    }

    // save source gravity vector
    gravityMutex.lock();
    sourceGravityVector.x = gravity[0];
    sourceGravityVector.y = gravity[1];
    sourceGravityVector.z = gravity[2];
    gravityMutex.unlock();

    return true;
}

std::vector<cv::Point2f> Keypoint2Point(std::vector<cv::KeyPoint> keypoints)
{
    std::vector<cv::Point2f> vectorOfPoints;
    for(unsigned i = 0; i < keypoints.size(); i++) {

        vectorOfPoints.push_back(keypoints[i].pt);

    }
    return vectorOfPoints;
}

void DrawShiftedCorners(cv::Mat image, cv::Mat homography){

    //-- Get the corners from the image_1 ( the object to be "detected" )
    std::vector< cv::Point2f > imageCorners(4);
    imageCorners[0] = cv::Point2f(0, 0);
    imageCorners[1] = cv::Point2f(image.cols, 0 );
    imageCorners[2] = cv::Point2f(image.cols, image.rows );
    imageCorners[3] = cv::Point2f(0, image.rows );
    std::vector< cv::Point2f > scene_corners(4);

    cv::perspectiveTransform(imageCorners, scene_corners, homography);

    //-- Draw lines between the corners (the mapped object in the scene - image_2 )
    cv::line(image, scene_corners[0], scene_corners[1], cv::Scalar(255), 4 );
    cv::line(image, scene_corners[1], scene_corners[2], cv::Scalar(255), 4 );
    cv::line(image, scene_corners[3], scene_corners[0], cv::Scalar(255), 4 );
    cv::line(image, scene_corners[2], scene_corners[3], cv::Scalar(255), 4 );
}

bool SimpleARClass::DetectKeypointsInQueryImage() {

    std::vector<cv::KeyPoint> queryKeypoints;
    cv::Mat queryDescriptors;

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
        homography = cv::findHomography(Keypoint2Point(sourceMatches), Keypoint2Point(queryMatches),
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
    for (unsigned i = 0; i < sourceMatches.size(); i++) {
        if (inlierMask.at<uchar>(i)) {
            sourceInlierKeypoints.push_back(sourceMatches[i]);
            queryInlierKeypoints.push_back(queryMatches[i]);
        }
    }
    MyLOGD("Number of kps match after homography filter = %d", (int) sourceInlierKeypoints.size());

    if (sourceInlierKeypoints.size() < 30) {
        MyLOGD("Not enough kps match after homography filter!");
        return false;
    }

    MyLOGD("Success, Number of keypoint matches = %d", (int)sourceInlierKeypoints.size());
    DrawShiftedCorners(cameraImageForBack, homography);
    return true;
}


// Calls the pnp based solver
void SimpleARClass::TrackKeypointsAndUpdatePose() {

    // Use inliers as reference points
    sourceInlierPoints.clear();
    queryInlierPoints.clear();
    int num_reference_pts = sourceInlierKeypoints.size();
    for(int i=0; i< num_reference_pts;i++){
        sourceInlierPoints.push_back(sourceInlierKeypoints[i].pt);
        queryInlierPoints.push_back(queryInlierKeypoints[i].pt);
    }


    // Project points onto an imaginary floor
    std::vector<cv::Point3f> keypointLocationsIn3D =
            myGLCamera->GetProjectedPointsOnFloor(sourceInlierPoints, sourceGravityVector,
                                                      CAM_HEIGHT_FROM_FLOOR,
                                                  cameraImageForBack.cols, cameraImageForBack.rows);


    // estimate pose of query frame with solvepnp
    glm::mat4 projectionViewMat = myGLCamera->GetProjectionView();
    cv::Mat cameraMatrix = ConstructCameraIntrinsicMatForCV(projectionViewMat,
                                                            cameraImageForBack.cols,
                                                            cameraImageForBack.rows);
    std::vector<float> distCoeffs; //null vector
    pnpMutex.lock();
    pnpResultIsValid = false;
    pnpResultIsValid = cv::solvePnP(keypointLocationsIn3D, queryInlierPoints, cameraMatrix, distCoeffs,
                                    rotationVector,
                                    translationVector);
    pnpMutex.unlock();

    if(!pnpResultIsValid) {
        MyLOGD("No solution to pnp!");
        return;
    }
    else{
        PrintCVMat(translationVector.t());
        PrintCVMat(rotationVector.t());
    }
}


cv::Mat SimpleARClass::ConstructCameraIntrinsicMatForCV(glm::mat4 projectionViewMat,
                                                        float imageWidth, float imageHeight) {

    //derive camera intrinsic mx from GL projection-mx
    cv::Mat cameraIntrinsicMat = cv::Mat::zeros(3, 3, CV_32F);
    // fx, fy, camera centers need to be in pixels for cv
    cameraIntrinsicMat.at<float>(0, 0) = projectionViewMat[0][0] * imageWidth / 2;
    cameraIntrinsicMat.at<float>(0, 2) = imageWidth / 2;
    cameraIntrinsicMat.at<float>(1, 1) = projectionViewMat[1][1] * imageHeight / 2;
    cameraIntrinsicMat.at<float>(1, 2) = imageHeight / 2;
    cameraIntrinsicMat.at<float>(2, 2) = 1.;
    return cameraIntrinsicMat;
}
