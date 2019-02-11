// CONSTRUCTOR
explicit BallFinder::Ballfinder(){
    topDownBallX = -1;
    topDownBallY = -1;
    topDownBallMeanPoint = cv::Point2f(-1, -1);
}


void BallFinder::findBall(int cameraObject.frameCounter, float x, float y, const std::chrono::time_point &cameraObject.startFrameTime, float oneradius,
              const cv::Scalar_<double> &color, int currentX, :cv::Point_<float> &topDownBallMeanPoint, cv::Mat &cameraImageThresholded,
float &ballSpeed, cv::Mat &topDown) {

ballSpeed = sqrt(speedPoint.x * speedPoint.x + speedPoint.y * speedPoint.y);
topDown = cv::Mat::zeros(cameraImageThresholded.size(), CV_8UC3);// START CARTESIAN X,Y CALCULATION //
float distance = (REAL_RADIUS * FOCAL_LENGTH) / oneradius;


// trigonometry magic from https://math.stackexchange.com/questions/1320285/convert-a-pixel-displacement-to-angular-rotation

int pixelsFromCenter = currentX - (0.5 * IMAGE_WIDTH);
float angleRadians = atan((2 * pixelsFromCenter * tan(0.5 * HORIZONTAL_FOV_RADIANS)) / (IMAGE_WIDTH));
float angleDegrees = angleRadians * (180.0 / 3.141592653589793238463);

float prevX = x;
float prevY = y;
x = distance * cos(angleRadians);
y = distance * sin(angleRadians);

// END X,Y CALCULATION //


// TODO find out difference prevX, previousCameraBallX, x, currentX


// START POSITION MEAN CALCULATION //

cv::Point_<float> prevPoint;

prevPoint.x = prevX;
prevPoint.y = prevY;

cv::Point_<float> cartesianPoint;

cartesianPoint.x = x;
cartesianPoint.y = y;

cv::Point_<float> prevMean = topDownBallMeanPoint;


if (cameraObject.frameCounter < BUFFER_SIZE) {
pointVector.push_back(cartesianPoint);
topDownBallMeanPoint = cartesianPoint;

//            auto currentTime= std::chrono::high_resolution_clock::now() - startTime;
//            double currentTimeSeconds = std::chrono::duration<double>(currentTime).count();
//            timeVector.push_back(currentTimeSeconds);
//push back
}
if (cameraObject.frameCounter >= BUFFER_SIZE) {
// circular push
circularPush(pointVector, cartesianPoint);

// calculate mean of points

cv::Point_<float> zero(0.0f, 0.0f);
cv::Point_<float> sum = accumulate(pointVector.begin(), pointVector.end(), zero);

topDownBallMeanPoint = cv::Point_<float>(sum.x / pointVector.size(), sum.y / pointVector.size());

//            auto currentTime= std::chrono::high_resolution_clock::now() - startTime;
//            double currentTimeSeconds = std::chrono::duration<double>(currentTime).count();
//            circularPush(timeVector, currentTimeSeconds);
}

// END POSITION MEAN CALCULATION //

// atan uses radians

auto endFrameTime = std::chrono::_V2::system_clock::now();
auto dT = (cameraObject.startFrameTime - endFrameTime);
double dTime = std::chrono::duration<double>(dT).count(); //convert to seconds

// START BALL SPEED CALC  //

cv::Point_<float> speedPoint;

// TODO actually implement time circular buffer
// TODO store derivatives in vector
// TODO calculate average derivative
if (COMPLICATED_DIFFERENCE_CALCULATION && cameraObject.frameCounter >= BUFFER_SIZE) {


std::vector<cv::Point_ < float>>
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
speedPoint = pointDifference / dTime;
}
std::cout << "distance=" << distance << ", angle=" << angleDegrees << std::endl;
std::cout << "x=" << distance << ", y=" << angleDegrees << std::endl;
std::cout << "ballspeed in cm/s:" << ballSpeed << std::endl;

// END BALL SPEED CALC //

// DRAWING TOP DOWN MAP STUFF //
;

cv::Point_<float> cameraXandY(100, 240);



float line1x = 540 * cos(-0.5 * HORIZONTAL_FOV_RADIANS);
float line1y = 540 * sin(-0.5 * HORIZONTAL_FOV_RADIANS);

float line2x = 540 * cos(0.5 * HORIZONTAL_FOV_RADIANS);
float line2y = 540 * sin(0.5 * HORIZONTAL_FOV_RADIANS);

line(topDown, cv::Point2i(100 + line1x, 240 + line1y), cv::Point2i(cameraXandY.x, cameraXandY.y),
        cv::Scalar<double>(255, 255, 255), 1);
line(topDown, cv::Point2i(100 + line2x, 240 + line2y), cv::Point2i(cameraXandY.x, cameraXandY.y),
        cv::Scalar<double>(255, 255, 255), 1);

Scalar_<double> orange = Scalar_<double>(2, 106, 253);
Scalar_<double> bluegray = Scalar_<double>(255, 120, 120);

cv::Point_<float> topDownBallPos;
topDownBallPos.x = 100 + topDownBallMeanPoint.x * 5;
topDownBallPos.y = 240 + topDownBallMeanPoint.y * 5;

circle(topDown, topDownBallPos, (int) 5, orange, 2, 8, 0); // draw orange ball

line(topDown, topDownBallPos, (topDownBallPos + (speedPoint * 1)), orange, 2); //speed line

// END OF TOP DOWN INIT //

// START INTERCEPTION CALC //
cv::Point_<float> interceptPos;

if (speedPoint.x >= 0) {
interceptPos = topDownBallMeanPoint;
} else {
float timeWhereBallXisZero = -topDownBallMeanPoint.x / speedPoint.x;
float ballYwhereBallXisZero = timeWhereBallXisZero * speedPoint.y + topDownBallMeanPoint.y;

if (ballYwhereBallXisZero > 0) {
// right hand
cv::Point_<float> interceptSpeed = cv::Point_<float>(-speedPoint.y, speedPoint.x);
float intersectTime = topDownBallMeanPoint.x / (interceptSpeed.x - speedPoint.x);
interceptPos = cv::Point_<float>(interceptSpeed.x * intersectTime, interceptSpeed.y * intersectTime);
} else if (ballYwhereBallXisZero < 0) {
// left hand
cv::Point_<float> interceptSpeed = cv::Point_<float>(speedPoint.y, -speedPoint.x);
float intersectTime = topDownBallMeanPoint.x / (interceptSpeed.x - speedPoint.x);
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