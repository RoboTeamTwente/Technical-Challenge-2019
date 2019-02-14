#include "BallFinder.h"
#include "Constants.h"
#include "Settings.h"
#include "ImageProcessor.h"
#include "CircularBuffer.h"
#include "Camera.h"


// CONSTRUCTOR
BallFinder::BallFinder() {
}

void BallFinder::findTopDownBallPoint(ImageProcessor imageProcessorObject) {

    ballDistanceFromCamera =
            (Constants::REAL_RADIUS * Constants::FOCAL_LENGTH) / imageProcessorObject.cameraImageBallRadius;

    // trigonometry magic from https://math.stackexchange.com/questions/1320285/convert-a-pixel-displacement-to-angular-rotation

    ballPixelsFromCenterX = imageProcessorObject.cameraImageBallCenterPoint.x - (0.5 * Settings::IMAGE_WIDTH);

    ballAngleInCameraPlane = atan((2 * ballPixelsFromCenterX * tan(0.5 * Constants::HORIZONTAL_FOV_RADIANS))
                                  / (Settings::IMAGE_WIDTH));

    ballAngleInCameraPlaneDegrees = ballAngleInCameraPlane * (180.0 / M_PIl);

    topDownBallPoint.x = ballDistanceFromCamera * cos(ballAngleInCameraPlane);
    topDownBallPoint.y = ballDistanceFromCamera * sin(ballAngleInCameraPlane);
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


    topDownBallPointHistorySum = accumulate(topDownBallPointHistory->getVector().begin(),
                                            topDownBallPointHistory->getVector().end(), cv::Point2f(0.0f, 0.0f));
    

    previousTopDownBallMeanPoint=topDownBallMeanPoint;
    
    topDownBallMeanPoint = cv::Point_<float>(topDownBallPointHistorySum.x / topDownBallPointHistory->getVector().size(),
                                             topDownBallPointHistorySum.y /
                                             topDownBallPointHistory->getVector().size());

}

void BallFinder::findBallSpeedVector(Camera cameraObject) {
    endFrameTime = std::chrono::steady_clock::now();
    auto frameDuration = (cameraObject.startFrameTime - endFrameTime);
    double frameDurationInSeconds = std::chrono::duration<double>(frameDuration).count(); //convert to seconds

    // START BALL SPEED CALC  //
    // TODO how does rest of roboteam determine ball pos and speed? I feel like this method is not the best

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

    ballSpeed = sqrt(ballVelocityVectorAsPoint.x * ballVelocityVectorAsPoint.x +
                     ballVelocityVectorAsPoint.y * ballVelocityVectorAsPoint.y);
    // END BALL SPEED CALC //

}

void BallFinder::findBallInterceptionVector() {
    // TODO refactor this (and other methods) for efficiency?

    if (ballVelocityVectorAsPoint.x >= 0) {
        interceptPos = topDownBallMeanPoint;
    } else {
        float timeWhereBallXisZero = -topDownBallMeanPoint.x / ballVelocityVectorAsPoint.x;
        float ballYwhereBallXisZero = timeWhereBallXisZero * ballVelocityVectorAsPoint.y + topDownBallMeanPoint.y;

        if (ballYwhereBallXisZero > 0) {
            // right hand
            cv::Point_<float> interceptSpeed = cv::Point_<float>(-ballVelocityVectorAsPoint.y,
                                                                 ballVelocityVectorAsPoint.x);
            float intersectTime = topDownBallMeanPoint.x / (interceptSpeed.x - ballVelocityVectorAsPoint.x);
            interceptPos = cv::Point_<float>(interceptSpeed.x * intersectTime, interceptSpeed.y * intersectTime);
        } else if (ballYwhereBallXisZero < 0) {
            // left hand
            cv::Point_<float> interceptSpeed = cv::Point_<float>(ballVelocityVectorAsPoint.y,
                                                                 -ballVelocityVectorAsPoint.x);
            float intersectTime = topDownBallMeanPoint.x / (interceptSpeed.x - ballVelocityVectorAsPoint.x);
            interceptPos = cv::Point_<float>(interceptSpeed.x * intersectTime, interceptSpeed.y * intersectTime);
            //determine interceptpos
        } else if (ballYwhereBallXisZero == 0) {
            interceptPos = topDownBallMeanPoint;
        }
    }
}
