#include "BallFinder.h"


// CONSTRUCTOR
BallFinder::BallFinder(){
}

void BallFinder::findTopDownBallPoint(ImageProcessor imageProcessorObject) {

    float ballDistanceFromCamera = (Constants::REAL_RADIUS * Constants::FOCAL_LENGTH) / imageProcessorObject.cameraImageBallRadius;

    // trigonometry magic from https://math.stackexchange.com/questions/1320285/convert-a-pixel-displacement-to-angular-rotation

    ballPixelsFromCenterX = imageProcessorObject.cameraImageBallCenterPoint.x - (0.5 * Settings::IMAGE_WIDTH);

    ballAngleInCameraPlane = atan( (2 * ballPixelsFromCenterX * tan(0.5 * Constants::HORIZONTAL_FOV_RADIANS))
            / (Settings::IMAGE_WIDTH));

    ballAngleInCameraPlaneDegrees = ballAngleInCameraPlane * (180.0 / M_PIl);

    topDownBallPoint.x = ballDistanceFromCamera * cos(ballAngleInCameraPlane);
    topDownBallPoint.y = ballDistanceFromCamera * sin(ballAngleInCameraPlane);
}


void BallFinder::findMeanOfBallPoints() {

    // START POSITION MEAN CALCULATION //

    if (topDownBallPointHistory == nullptr){

        std::vector<cv::Point2f> inputVector(Settings::DERIVATIVE_BUFFER_SIZE);
        // Set all values to initial value
        std::fill(inputVector.begin(), inputVector.end(), topDownBallPoint);


        topDownBallPointHistory = new CircularBuffer<cv::Point2f>(inputVector);

    } else {
        topDownBallPointHistory->circularPush(topDownBallPoint);
    }


    topDownBallPointHistorySum = accumulate(topDownBallPointHistory->getVector().begin(), topDownBallPointHistory->getVector().end(), zero);
    topDownBallMeanPoint = cv::Point_<float>(topDownBallPointHistorySum.x / topDownBallPointHistory->getVector().size(), topDownBallPointHistorySum.y / topDownBallPointHistory->getVector().size());

}

void BallFinder::findBallSpeedVector() {
    // atan uses radians

    auto endFrameTime = std::chrono::_V2::system_clock::now();
    auto dT = (cameraObject.startFrameTime - endFrameTime);
    double dTime = std::chrono::duration<double>(dT).count(); //convert to seconds

    // START BALL SPEED CALC  //




    // TODO actually implement time circular buffer
    // TODO store derivatives in vector
    // TODO calculate average derivative
    if (Settings::COMPLICATED_DIFFERENCE_CALCULATION && cameraObject.frameCounter >= Settings::DERIVATIVE_BUFFER_SIZE) {


        std::vector<cv::Point_<float>>
                derivatives;
        for (int firstIndex = 0; firstIndex < pointVector.size() - 1; ++firstIndex) {
            for (int secondIndex = firstIndex + 1; secondIndex < pointVector.size(); ++secondIndex) {

                cv::Point_<float> firstPoint = pointVector[firstIndex];
                cv::Point_<float> secondPoint = pointVector[secondIndex];
                double firstTime = timeVector[firstIndex];
                double secondTime = timeVector[secondIndex];

                cv::Point_<float> derivative = (secondPoint - firstPoint) / (secondTime - firstTime);

            }
        }


        // calculate derivative
        // store in vector


        //calculate mean of vector


    } else {
        cv::Point_<float> pointDifference = prevMean - topDownBallMeanPoint;
        ballVelocityVectorAsPoint = pointDifference / dTime;
    }
    std::cout << "distance=" << distance << ", angle=" << angleDegrees << std::endl;
    std::cout << "x=" << distance << ", y=" << angleDegrees << std::endl;
    ballSpeed = sqrt(ballVelocityVectorAsPoint.x * ballVelocityVectorAsPoint.x +
                     ballVelocityVectorAsPoint.y * ballVelocityVectorAsPoint.y);
    std::cout << "ballspeed in cm/s:" << ballSpeed << std::endl;

    // END BALL SPEED CALC //

}

void BallFinder::findBallInterceptionVector() {

    // START INTERCEPTION CALC //
    cv::Point_<float> interceptPos;

    if (ballVelocityVectorAsPoint.x >= 0) {
    interceptPos = topDownBallMeanPoint;
    } else {
    float timeWhereBallXisZero = -topDownBallMeanPoint.x / ballVelocityVectorAsPoint.x;
    float ballYwhereBallXisZero = timeWhereBallXisZero * ballVelocityVectorAsPoint.y + topDownBallMeanPoint.y;

    if (ballYwhereBallXisZero > 0) {
    // right hand
    cv::Point_<float> interceptSpeed = cv::Point_<float>(-ballVelocityVectorAsPoint.y, ballVelocityVectorAsPoint.x);
    float intersectTime = topDownBallMeanPoint.x / (interceptSpeed.x - ballVelocityVectorAsPoint.x);
    interceptPos = cv::Point_<float>(interceptSpeed.x * intersectTime, interceptSpeed.y * intersectTime);
    } else if (ballYwhereBallXisZero < 0) {
    // left hand
    cv::Point_<float> interceptSpeed = cv::Point_<float>(ballVelocityVectorAsPoint.y, -ballVelocityVectorAsPoint.x);
    float intersectTime = topDownBallMeanPoint.x / (interceptSpeed.x - ballVelocityVectorAsPoint.x);
    interceptPos = cv::Point_<float>(interceptSpeed.x * intersectTime, interceptSpeed.y * intersectTime);
    //determine interceptpos
    } else if (ballYwhereBallXisZero == 0) {
    interceptPos = topDownBallMeanPoint;
    }

    }

    // detemrine topdown interceptpos
    line(topDown, cameraXandY, cameraXandY + (interceptPos * 1), bluegray, 2); //speed line

    // END INTERCEPTION CALC //

}
