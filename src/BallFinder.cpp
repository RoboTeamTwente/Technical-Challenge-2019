#include <cmath>

#include "BallFinder.h"
#include "ImageProcessor.h"

#include "Camera.h"
#include "Constants.h"
#include "Settings.h"
#include "CircularBuffer.h"

// CONSTRUCTOR
BallFinder::BallFinder() {
    topDownBallPointHistory=nullptr;
}

void BallFinder::findTopDownBallPoint(const ImageProcessor &imageProcessorObject) {

    ballDistanceFromCamera =
            (Constants::REAL_RADIUS * Constants::FOCAL_LENGTH) / imageProcessorObject.cameraImageBallRadius;

    // trigonometry magic from https://math.stackexchange.com/questions/1320285/convert-a-pixel-displacement-to-angular-rotation

    ballPixelsFromCenterX = static_cast<int>(imageProcessorObject.cameraImageBallCenterPoint.x - (0.5 * Settings::IMAGE_WIDTH));

    ballAngleInCameraPlane = static_cast<float>(atan((2 * ballPixelsFromCenterX * tan(0.5 * Constants::HORIZONTAL_FOV_RADIANS))
                                                     / (Settings::IMAGE_WIDTH)));

    ballAngleInCameraPlaneDegrees = static_cast<float>(ballAngleInCameraPlane * (180.0 / M_PIl));

    topDownBallPoint.x = (ballDistanceFromCamera * std::cos(ballAngleInCameraPlane));
    topDownBallPoint.y = (ballDistanceFromCamera * std::sin(ballAngleInCameraPlane));
}


void BallFinder::findMeanOfBallPoints() {

    // START POSITION MEAN CALCULATION //

    if (topDownBallPointHistory == nullptr) {
        topDownBallMeanPoint=topDownBallPoint; // for initializing previousTopDownBallMeanPoint
        
        std::vector<cv::Point2f> inputVector(Settings::DERIVATIVE_BUFFER_SIZE);
        // Set all values to initial value
        std::fill(inputVector.begin(), inputVector.end(), topDownBallPoint);


        topDownBallPointHistory = new CircularBuffer<cv::Point2f>(inputVector);

    } else {
        topDownBallPointHistory->circularPush(topDownBallPoint);
    }

    topDownBallPointHistorySum=cv::Point2f(0,0);

    for (int i=0;i < topDownBallPointHistory->getVector().size(); i++) {
        topDownBallPointHistorySum+=topDownBallPointHistory->get(i);
    }
//    topDownBallPointHistorySum = accumulate(topDownBallPointHistory->getVector().begin(),
//                                            topDownBallPointHistory->getVector().end(), cv::Point2f(0.0f, 0.0f));
    

    previousTopDownBallMeanPoint=topDownBallMeanPoint;
    
    topDownBallMeanPoint = cv::Point_<float>(topDownBallPointHistorySum.x / topDownBallPointHistory->getVector().size(),
                                             topDownBallPointHistorySum.y /
                                             topDownBallPointHistory->getVector().size());

}

void BallFinder::findBallSpeedVector(Camera cameraObject) {
    endFrameTime = std::chrono::steady_clock::now();
    auto frameDuration = (endFrameTime - cameraObject.startFrameTime);

    double frameDurationInSeconds = std::chrono::duration<double>(frameDuration).count(); //convert to seconds
//    std::cout << frameDurationInSeconds*1000 << std::endl;

    // START BALL SPEED CALC  //
    // TODO how does rest of roboteam determine ball pos and speed? I feel like this method is not the best
    // moving average calculation seems like a good idea

    if (Settings::COMPLICATED_DIFFERENCE_CALCULATION && cameraObject.frameCounter >= Settings::DERIVATIVE_BUFFER_SIZE) {

        ballVelocityVectorAsPoint=cv::Point2f(0,0);

        for (int i = 1; i < topDownBallPointHistory->getVector().size(); i++) {
            ballVelocityVectorAsPoint += cv::Point2f((topDownBallPointHistory->get(i).x - topDownBallPointHistory->get(0).x),(topDownBallPointHistory->get(i).y - topDownBallPointHistory->get(0).y));
        }
        ballVelocityVectorAsPoint.x /= topDownBallPointHistory->getVector().size()-1;
        ballVelocityVectorAsPoint.y /= topDownBallPointHistory->getVector().size()-1;
        
        ballVelocityVectorAsPoint /= frameDurationInSeconds;
        
        
    } else {
        cv::Point_<float> pointDifference = previousTopDownBallMeanPoint - topDownBallMeanPoint;
        ballVelocityVectorAsPoint = pointDifference / frameDurationInSeconds;
    }

    ballSpeed = (std::sqrt(ballVelocityVectorAsPoint.x * ballVelocityVectorAsPoint.x +
                         ballVelocityVectorAsPoint.y * ballVelocityVectorAsPoint.y));
    if (ballSpeed < 20){
        ballSpeed = 0;
    }
    // END BALL SPEED CALC //

}

void BallFinder::findBallInterceptionVector() {
    // TODO refactor this (and other methods) for efficiency?
    // TODO fix this (angle is wrong)

    if (ballVelocityVectorAsPoint.x <= 0) {
        interceptPos.x = topDownBallMeanPoint.x;
        interceptPos.y = topDownBallMeanPoint.y;
    } else {
        // This is not stable
//        ballX = topDownBallMeanPoint.x;
//        ballY= topDownBallMeanPoint.y;
//        ballXt = -ballVelocityVectorAsPoint.x;
//        ballYt = -ballVelocityVectorAsPoint.y;
//        robotX = 0;
//        robotY = 0;
//        robotXt = -ballVelocityVectorAsPoint.x;
//        robotYt = ballVelocityVectorAsPoint.x;
        interceptPos.x = topDownBallMeanPoint.x -(ballVelocityVectorAsPoint.x * 0.1);
        interceptPos.y = topDownBallMeanPoint.y -  (ballVelocityVectorAsPoint.y * 0.1);

//        float timeWhereBallXisZero = -topDownBallMeanPoint.x / ballVelocityVectorAsPoint.x;
//        float ballYwhereBallXisZero = timeWhereBallXisZero * ballVelocityVectorAsPoint.y + topDownBallMeanPoint.y;
//
//        if (ballYwhereBallXisZero > 0) {
//            // right hand
//            cv::Point_<float> interceptSpeed = cv::Point_<float>(-ballVelocityVectorAsPoint.y,
//                                                                 ballVelocityVectorAsPoint.x);
//            float intersectTime = topDownBallMeanPoint.x / (interceptSpeed.x - ballVelocityVectorAsPoint.x);
//            interceptPos = cv::Point_<float>(interceptSpeed.x * intersectTime, interceptSpeed.y * intersectTime);
//        } else if (ballYwhereBallXisZero < 0) {
//            // left hand
//            cv::Point_<float> interceptSpeed = cv::Point_<float>(ballVelocityVectorAsPoint.y,
//                                                                 -ballVelocityVectorAsPoint.x);
//            float intersectTime = topDownBallMeanPoint.x / (interceptSpeed.x - ballVelocityVectorAsPoint.x);
//            interceptPos = cv::Point_<float>(interceptSpeed.x * intersectTime, interceptSpeed.y * intersectTime);
//            //determine interceptpos
//        } else if (ballYwhereBallXisZero == 0) {
//            interceptPos = topDownBallMeanPoint;
//        }
    }
}
