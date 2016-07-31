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


#include "myGLCamera.h"
#include "myLogger.h"
#include "math.h"

MyGLCamera::MyGLCamera(
        float FOV,
        float zPosition,
        float nearPlaneDistance,
        float farPlaneDistance) {

    // camera position is fixed
    glm::vec3 cameraPosition = glm::vec3(0, 0, zPosition);
    viewMat = glm::lookAt(cameraPosition,        // Camera location in World Space
                          glm::vec3(0, 0, -1),   // direction in which camera it is pointed
                          glm::vec3(0, 1, 0));   // camera is pointing up
    this->zPosition = zPosition;
    this->nearPlaneDistance = nearPlaneDistance;
    this->farPlaneDistance = farPlaneDistance;
    this->FOV       = FOV;

    // 6DOF describing model's position
    deltaX = deltaY = deltaZ = 0;                  // translations
    modelQuaternion = glm::quat(glm::vec3(0,0,0)); // rotation

    modelMat        = glm::mat4(1.0f);
    translateMat    = glm::mat4(1.0f);
    rotateMat       = glm::mat4(1.0f);
    mvpMat = glm::mat4(1.0f); // projection is not known -> initialize MVP to identity
}

/**
 * Use the display's aspect ratio to compute projection matrix
 */
void MyGLCamera::SetAspectRatio(float aspect) {

    projectionMat = glm::perspective(FOV * float(M_PI / 180), // camera's field-of-view
                                     aspect,                  // camera's aspect ratio
                                     nearPlaneDistance,       // distance to the near plane
                                     farPlaneDistance);       // distance to the far plane
    projectionViewMat = projectionMat * viewMat;
    ComputeMVPMatrix();

}

/**
 * Model's position has 6 degrees-of-freedom: 3 for x-y-z locations and
 * 3 for alpha-beta-gamma Euler angles
 * Convert euler angles to quaternion and update MVP
 */
void MyGLCamera::SetModelPosition(std::vector<float> modelPosition) {

    deltaX = modelPosition[0];
    deltaY = modelPosition[1];
    deltaZ = modelPosition[2];
    float pitchAngle = modelPosition[3];
    float yawAngle   = modelPosition[4];
    float rollAngle  = modelPosition[5];

    modelQuaternion = glm::quat(glm::vec3(pitchAngle, yawAngle, rollAngle));
    rotateMat = glm::toMat4(modelQuaternion);
    ComputeMVPMatrix();
}


/**
 * Compute the translation matrix from x-y-z position and rotation matrix from
 * quaternion describing the rotation
 * MVP = Projection * View * (Translation * Rotation)
 */
void MyGLCamera::ComputeMVPMatrix() {

    translateMat = glm::mat4(1, 0, 0, 0,                  // col0
                             0, 1, 0, 0,	              // col1
                             0, 0, 1, 0,	              // col2
                             deltaX, deltaY, deltaZ, 1);  // col3

    modelMat    = translateMat * rotateMat;
    mvpMat = projectionViewMat * modelMat;
}

/**
 * Simulate change in scale by pushing or pulling the model along Z axis
 */
void MyGLCamera::ScaleModel(float scaleFactor) {

    deltaZ += SCALE_TO_Z_TRANSLATION * (scaleFactor - 1);
    ComputeMVPMatrix();
}

/**
 * Finger drag movements are converted to rotation of model by deriving a
 * quaternion from the drag movement
 */
void MyGLCamera::RotateModel(float distanceX, float distanceY,
                             float endPositionX, float endPositionY) {

    // algo in brief---
    // assume that a sphere with its center at (0,0), i.e., center of screen, and
    // radius 1 is placed on the device.
    // drag movement on the surface is translated to a drag on the imaginary sphere's surface
    // since we know (x,y) coordinates of the drag, we only need to determine z-coordinate
    // corresponding to the height of sphere's surface corresponding to the drag position.
    // begin and end of drag define two points on sphere and we create two vectors joining those
    // points with the origin (0,0).
    // lastly we create a quaternion responsible for rotation between the two vectors.

    // compute ending vector (using endPositionX, endPositionY)
    float dist = sqrt(fmin(1, endPositionX * endPositionX + endPositionY * endPositionY));
    float positionZ = sqrt(1 - dist*dist);
    glm::vec3 endVec = glm::vec3(endPositionX, endPositionY, positionZ);
    endVec = glm::normalize(endVec);

    // compute starting vector by adding (distanceX, distanceY) to ending positions
    endPositionX += distanceX;
    endPositionY += distanceY;
    dist = sqrt(fmin(1, endPositionX * endPositionX + endPositionY * endPositionY));
    positionZ = sqrt(1 - dist*dist);
    glm::vec3 beginVec = glm::vec3(endPositionX, endPositionY, positionZ);
    beginVec = glm::normalize(beginVec);

    // compute cross product of vectors to find axis of rotation
    glm::vec3 rotationAxis = glm::cross(beginVec, endVec);
    rotationAxis = glm::normalize(rotationAxis);

    // compute angle between vectors using the dot product
    float dotProduct = fmax(fmin(glm::dot(beginVec, endVec), 1.), -1.);
    float rotationAngle = TRANSLATION_TO_ANGLE*acos(dotProduct);

    // compute quat using above
    modelQuaternion = glm::angleAxis(rotationAngle, rotationAxis);
    rotateMat = glm::toMat4(modelQuaternion)*rotateMat;

    ComputeMVPMatrix();
}

/**
 * displace model by changing x-y coordinates
 */
void MyGLCamera::TranslateModel(float distanceX, float distanceY) {

    deltaX += XY_TRANSLATION_FACTOR * distanceX;
    deltaY += XY_TRANSLATION_FACTOR * distanceY;
    ComputeMVPMatrix();
}


/**
 * Align MVP matrix along the gravity vector
 */
glm::mat4 MyGLCamera::GetMVPAlignedWithGravity(std::vector<float> gravity) {

    // extract 3x3 rotation mat from model mat
    glm::mat3 currentRotationMat = glm::mat3(modelMat);
    // in every device, Y axis is pointing upwards before applying rotations
    // check the current position of Y axis after rotating it
    glm::vec3 sceneUpVector = currentRotationMat * glm::vec3(0.0, 1.0, 0.0);
    sceneUpVector = glm::normalize(sceneUpVector);

    // normalize the gravity vector
    glm::vec3 gravityVector = glm::vec3(gravity[0], gravity[1], gravity[2]);
    gravityVector = glm::normalize(gravityVector);

    // find the angle between sceneUpVector and gravity vector
    float cosTheta = fmax(fmin(glm::dot(sceneUpVector, gravityVector), 1.), -1.);
    float rotationAngle = acos(cosTheta);
    glm::vec3 rotationAxis = glm::cross(sceneUpVector, gravityVector);
    rotationAxis = glm::normalize(rotationAxis);

    // compute quaternion that will rotate and align sceneUpVector along gravity vector
    glm::quat gravityRotationQuat = glm::angleAxis(rotationAngle, rotationAxis);
    glm::mat4 gravityRotationMat = glm::toMat4(gravityRotationQuat);


    glm::mat4 currentModelMat = modelMat;
    glm::mat3 correctedRotationMat = glm::mat3(gravityRotationMat) * currentRotationMat;
    for(int r=0;r<3;r++){
        for(int c=0;c<3;c++){
            currentModelMat[c][r] = correctedRotationMat[c][r];
        }
    }

    glm::mat4 newMvpMat = projectionViewMat * currentModelMat;
    return newMvpMat;
}

/**
 * Accumulate the rotation from gyro into camera's rotation matrix
 */
void MyGLCamera::AddDeltaRotation(glm::mat4 deltaRotationMat){

    //accumulate gyro delta on right since latest rotation is applied first to model
    rotateMat = rotateMat * deltaRotationMat;
    modelMat    = translateMat * rotateMat;

}


/*
 * Projects the set of 2D points provided onto 3D space at an assumed height from floor
 * ref_points -> 2D points, x,y as per CV
 * height_from_floor -> Vertical height of camera from the floor
 * Returns a vector of back-projected 3D points (w.r.t OpenGL origin)
 */
std::vector<cv::Point3f> MyGLCamera::GetProjectedPointsOnFloor(std::vector<cv::Point2f> sourceKeypoints,
                                                               glm::vec3 sourceGravityVector,
                                                               float heightFromFloor,
                                                               float imageWidth, float imageHeight){

    //Calculate the near plane distances
    float zNearPlane = zPosition - nearPlaneDistance;
    float tangent = tan((FOV * M_PI / 180)/2);   // FOV = display frustum's FOVY
    float nearPlaneHalfHeight = nearPlaneDistance * tangent;
    float aspect = imageWidth / imageHeight;
    float nearPlaneHalfWidth = nearPlaneHalfHeight * aspect;

    glm::vec3 cameraPosition = glm::vec3(0, 0, zPosition);
    glm::vec3 gravity = glm::normalize(-sourceGravityVector);

    std::vector<cv::Point3f> pointLocationsIn3D(sourceKeypoints.size(), cv::Point3f(0, 0, 0));
    for(unsigned int i=0; i < sourceKeypoints.size(); i++){

        // convert point coordinates to normalized device coordinates (ndc) in range [-1,1]
        float xCoordndc = 2 * sourceKeypoints[i].x / imageWidth - 1;
        float xNearPlane = xCoordndc * nearPlaneHalfWidth;
        float yCoordndc = 1 - 2 * sourceKeypoints[i].y / imageHeight;
        float yNearPlane = yCoordndc * nearPlaneHalfHeight;
        glm::vec3 pointOnNearPlane = glm::vec3(xNearPlane, yNearPlane, zNearPlane);

        glm::vec3 unitRay = glm::normalize(pointOnNearPlane - cameraPosition);
        float cosineRayGravity = glm::dot(gravity, unitRay);

        //cameraPosition added to convert point to openGL reference system = worldspace
        glm::vec3 pointOnFloor = unitRay * (heightFromFloor / cosineRayGravity) +
                                 cameraPosition;

        //flip y,z axis for opencv
        pointLocationsIn3D[i].x = pointOnFloor.x;
        pointLocationsIn3D[i].y = -pointOnFloor.y;
        pointLocationsIn3D[i].z = -pointOnFloor.z;

//        MyLOGD("pointLocationsIn3D %f %f %f", pointLocationsIn3D[i].x, pointLocationsIn3D[i].y,
//               pointLocationsIn3D[i].z );

    }

    return pointLocationsIn3D;
}


void MyGLCamera::UpdateModelMat(cv::Mat translationVector,
                                cv::Mat rotationVector,
                                cv::Mat defaultModelPosition) {

    cv::Mat newModelMat = cv::Mat::eye(4, 4, CV_64F);

    //load translation vector in top-last column 3x1
    translationVector.convertTo(translationVector, CV_64F);
    newModelMat.at<double>(0, 3) = translationVector.at<double>(0, 0);
    newModelMat.at<double>(1, 3) = translationVector.at<double>(1, 0);
    newModelMat.at<double>(2, 3) = translationVector.at<double>(2, 0);

    //construct rotation vec
    cv::Mat newRotationMat;
    cv::Rodrigues(rotationVector, newRotationMat);
    newRotationMat.copyTo(newModelMat(cv::Rect(0, 0, 3, 3)));

    newModelMat.convertTo(newModelMat, CV_32F);
    newModelMat = newModelMat.t();
    modelMat = glm::make_mat4((float *) newModelMat.data);

    glm::mat4 translateModel = glm::mat4(1.);
    defaultModelPosition.convertTo(defaultModelPosition, CV_64F);
    translateModel[3][0] = defaultModelPosition.at<double>(0,0);
    translateModel[3][1] = defaultModelPosition.at<double>(1,0);
    translateModel[3][2] = defaultModelPosition.at<double>(2,0);

    modelMat = modelMat * translateModel;

    mvpMat = projectionViewMat * modelMat;
}
