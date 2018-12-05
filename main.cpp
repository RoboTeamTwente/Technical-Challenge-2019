#include <iostream>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <numeric>
#include <ctime>
#include <chrono>

#include "constants.cpp"

using realClock = std::chrono::high_resolution_clock;



class circularBufferPoint2f {
private:
    std::vector<cv::Point2f> contents;
public:
    explicit circularBufferPoint2f(std::vector<cv::Point2f> input) {
        contents = input;
    }

    void reset(std::vector<cv::Point2f> input) {
        contents = input;
    }

    int getLength() {
        return contents.size();
    }

    std::vector<cv::Point2f> getVector(){
        return contents;
    };

    void push(cv::Point inputPoint) {
        // simple rotation to the right
        std::rotate(contents.rbegin(), contents.rbegin() + 1, contents.rend());
        // replace first element
        contents[0] = inputPoint;
    }

    cv::Point getElement(int index) {
        return contents[index];
    }
};

template<typename T, typename A>
void circularPush ( std::vector<T,A> vec, T element ) {
    std::rotate(vec.rbegin(), vec.rbegin() + 1, vec.rend());
    // replace first element
    vec[0] = element;
}




int main(int argc, char **argv) {
    auto startTime = realClock::now();



    // INIT CAMERA
    cv::VideoCapture cap(0);
    std::vector<cv::Point2f> defaultBuffer = {cv::Point2f(2,2)};
    circularBufferPoint2f buffer(defaultBuffer);

    if (!cap.isOpened()) {
        std::cout << "webcam failure; is another openCV program running?" << std::endl;
        return -1;
    }


    cv::namedWindow("Control", CV_WINDOW_AUTOSIZE); //create control window

    std::cout << "starting control window" << std::endl;

    int iLowH = LOW_HUE;
    int iHighH = HIGH_HUE;

    int iLowS = LOW_SATURATION;
    int iHighS = HIGH_SATURATION;

    int iLowV = LOW_VALUE;
    int iHighV = HIGH_VALUE;

    int previousX = -1;
    int previousY = -1;
    int frameCounter = 0;


    //Create trackbars in "Control" window

    // TODO find out why this does not require cv:: ?

    cvCreateTrackbar("LowH", "Control", &iLowH, 179); //Hue
    cvCreateTrackbar("HighH", "Control", &iHighH, 179);

    cvCreateTrackbar("LowS", "Control", &iLowS, 255); //Saturation
    cvCreateTrackbar("HighS", "Control", &iHighS, 255);

    cvCreateTrackbar("LowV", "Control", &iLowV, 255); //Value
    cvCreateTrackbar("HighV", "Control", &iHighV, 255);


    cv::Mat imgBGR;
    cv::Mat imgLines;
    float x = -1;
    float y = -1;
    cv::Point2f meanPoint;
    meanPoint = cv::Point2f(-1,-1);

    while (true) {


        std::vector<cv::Point> largest_contour;
        int largest_area = 0;
        int largest_contour_index = 0;
        cv::Rect bounding_rect;

        auto startFrameTime = realClock::now();


        frameCounter++;

        // IMAGE CAPTURE //


        bool captureSuccess = cap.read(imgBGR); // read frame from camera

        if (!captureSuccess) {
            std::cout << "Cannot read a frame from video stream" << std::endl;
            break;
        }

        if (frameCounter % 30 == 1) {
            imgLines = cv::Mat::zeros(imgBGR.size(), CV_8UC3);
        }


        cv::Mat imgHSV;

        cv::cvtColor(imgBGR, imgHSV, cv::COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV

        cv::Mat imgThresholded;

        cv::inRange(imgHSV, cv::Scalar(iLowH, iLowS, iLowV), cv::Scalar(iHighH, iHighS, iHighV), imgThresholded);

        // some filtering

        //morphological opening
        erode(imgThresholded, imgThresholded,
              getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(MORPHOLOGICAL_OPENING_SIZE, MORPHOLOGICAL_OPENING_SIZE)));
        dilate(imgThresholded, imgThresholded,
               getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(MORPHOLOGICAL_OPENING_SIZE, MORPHOLOGICAL_OPENING_SIZE)));

        //morphological closing
//        dilate(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(MORPHOLOGICAL_CLOSING_SIZE, MORPHOLOGICAL_CLOSING_SIZE)));
//        erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(MORPHOLOGICAL_CLOSING_SIZE, MORPHOLOGICAL_CLOSING_SIZE)));


        // CONTOUR FINDING //
        std::vector<std::vector<cv::Point> > contours;
        cv::Mat contourOutput = imgThresholded.clone();
        cv::findContours(contourOutput, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);


        cv::Mat contourImage(contourOutput.size(), CV_8UC3, cv::Scalar(0, 0, 0));
        cv::Scalar colors[3];
        colors[0] = cv::Scalar(255, 0, 0);
        colors[1] = cv::Scalar(0, 255, 0);
        colors[2] = cv::Scalar(0, 0, 255);

        std::vector<std::vector<cv::Point>> contours_poly(contours.size());
        cv::Point2f onecenter;
        float oneradius;

        for (size_t idx = 0; idx < contours.size(); idx++) {

            cv::drawContours(contourImage, contours, idx, colors[idx % 3]);

            double a = contourArea(contours[idx], false);  //  Find the area of contour
            if (a > largest_area) {

                largest_area = a;

                largest_contour = contours[idx];
                largest_contour_index = idx;                //Store the index of largest contour
                cv::approxPolyDP(cv::Mat(contours[idx]), contours_poly[idx], 3, true);

                cv::minEnclosingCircle((cv::Mat) contours_poly[idx], onecenter, oneradius);


            }
        }
        cv::Mat drawing = cv::Mat::zeros(imgThresholded.size(), CV_8UC3);;
        cv::Scalar color = cv::Scalar(255, 255, 255);

        for (int i = 0; i < contours.size(); i++) {
            cv::drawContours(drawing, contours_poly, i, color, 1, 8, std::vector<cv::Vec4i>(), 0, cv::Point());

        }
        cv::circle(contourImage, onecenter, (int) oneradius, color, 2, 8, 0);

        // IMAGE DRAWING //



        int currentX = onecenter.x;
        int currentY = onecenter.y;

        if (previousX >= 0 && previousY >= 0 && currentX >= 0 && currentY >= 0) {

            cv::line(imgLines, cv::Point(currentX, currentY), cv::Point(previousX, previousY), cv::Scalar(255, 0, 0), 10);
        }

        previousX = currentX;
        previousY = currentY;

        // SPEED LINE //


        cv::imshow("Thresholded Image", imgThresholded);
        cv::moveWindow("Thresholded Image", 0, 0);
        imgBGR = imgBGR + imgLines + contourImage;
        cv::imshow("Original", imgBGR);
        cv::moveWindow("Original", 0, 600);




        // DISTANCE AND ANGLE CALCULATION //

        float distance = (REAL_RADIUS * FOCAL_LENGTH) / oneradius;


        // trigonometry magic from https://math.stackexchange.com/questions/1320285/convert-a-pixel-displacement-to-angular-rotation

        int pixelsFromCenter = currentX - (0.5 * IMAGE_WIDTH);
        float angleRadians = atan((2 * pixelsFromCenter * tan(0.5 * HORIZONTAL_FOV_RADIANS)) / (IMAGE_WIDTH));
        float angleDegrees = angleRadians * (180.0 / 3.141592653589793238463);

        float prevX = x;
        float prevY = y;
        x = distance * cos(angleRadians);
        y = distance * sin(angleRadians);





        cv::Point2f prevPoint;

        prevPoint.x = prevX;
        prevPoint.y = prevY;

        cv::Point2f cartesianPoint;

        cartesianPoint.x = x;
        cartesianPoint.y = y;

        cv::Point2f prevMean = meanPoint;
        std::vector<double> timeVector;

        if (frameCounter < BUFFER_SIZE){
            meanPoint = cartesianPoint;

//            auto currentTime= realClock::now() - startTime;
//            double currentTimeSeconds = std::chrono::duration<double>(currentTime).count();
//            timeVector.push_back(currentTimeSeconds);
            //push back
        }

        if (frameCounter == BUFFER_SIZE) {
            std::vector<cv::Point2f> v = {cartesianPoint, prevPoint};
            buffer.reset(v);
        }

        if (frameCounter > BUFFER_SIZE){
            buffer.push(cartesianPoint);
        }

        if (frameCounter >=  BUFFER_SIZE) {
            std::vector<cv::Point2f> points=buffer.getVector();
            cv::Point2f zero(0.0f, 0.0f);
            cv::Point2f sum  = accumulate(points.begin(), points.end(), zero);

            meanPoint = cv::Point2f(sum.x / points.size(), sum.y / points.size());

//            auto currentTime= realClock::now() - startTime;
//            double currentTimeSeconds = std::chrono::duration<double>(currentTime).count();
//            circularPush(timeVector, currentTimeSeconds);


        }
        // atan uses radians

        // BALL SPEED CALC //
        auto endFrameTime = realClock::now();
        auto dT = (startFrameTime - endFrameTime);
        double dTime = std::chrono::duration<double>(dT).count(); //convert to seconds



        cv::Point2f speedPoint;

        // TODO actually implement time circular buffer
        // TODO store derivatives in vector
        // TODO calculate average derivative
        if (COMPLICATED_DIFFERENCE_CALCULATION && frameCounter >=  BUFFER_SIZE){

            std::vector<cv::Point2f> points=buffer.getVector();
            std::vector<cv::Point2f> derivatives;
            for (int firstIndex = 0; firstIndex < points.size()-1; ++firstIndex){
                for (int secondIndex = firstIndex+1; secondIndex < points.size(); ++secondIndex){

                    cv::Point2f firstPoint = buffer.getElement(firstIndex);
                    cv::Point2f secondPoint = buffer.getElement(secondIndex);
                    double firstTime = timeVector[firstIndex];
                    double secondTime = timeVector[secondIndex];

                    cv::Point2f derivative = (secondPoint-firstPoint)/(secondTime-firstTime);

                }
            }


                    // calculate derivative
                    // store in vector


            //calculate mean of vector


        } else {
            cv::Point2f pointDifference = prevMean - meanPoint;
            speedPoint = pointDifference/dTime;
        }





        float ballSpeed = sqrt(speedPoint.x*speedPoint.x + speedPoint.y*speedPoint.y);

        std::cout << "distance=" << distance << ", angle=" << angleDegrees << std::endl;
        std::cout << "x=" << distance << ", y=" << angleDegrees << std::endl;
        std::cout << "ballspeed in cm/s:" << ballSpeed << std::endl;
        // DRAWING FIELD OF VIEW STUFF //


        cv::Mat topDown = cv::Mat::zeros(imgThresholded.size(), CV_8UC3);;

        cv::Point2f cameraXandY(100,240);

        cv::circle(topDown, cameraXandY, (int) 5, color, 2, 8, 0);


        float line1x = 540 * cos(-0.5 * HORIZONTAL_FOV_RADIANS);
        float line1y = 540 * sin(-0.5 * HORIZONTAL_FOV_RADIANS);

        float line2x = 540 * cos(0.5 * HORIZONTAL_FOV_RADIANS);
        float line2y = 540 * sin(0.5 * HORIZONTAL_FOV_RADIANS);

        cv::line(topDown, cv::Point(100 + line1x, 240 + line1y), cv::Point(cameraXandY.x, cameraXandY.y), cv::Scalar(255, 255, 255), 1);
        cv::line(topDown, cv::Point(100 + line2x, 240 + line2y), cv::Point(cameraXandY.x, cameraXandY.y), cv::Scalar(255, 255, 255), 1);

        cv::Scalar orange = cv::Scalar(2, 106, 253);
        cv::Scalar bluegray = cv::Scalar (255,120,120);

        cv::Point2f topDownBallPos;
        topDownBallPos.x = 100 + meanPoint.x * 5;
        topDownBallPos.y = 240 + meanPoint.y * 5;

        cv::circle(topDown, topDownBallPos, (int) 5, orange, 2, 8, 0); // draw orange ball

        cv::line(topDown, topDownBallPos, (topDownBallPos+(speedPoint*1)), orange, 2); //speed line

        // interception line
        cv::Point2f interceptPos;

        if (speedPoint.x >= 0){
            interceptPos = meanPoint;
        } else {
            float timeWhereBallXisZero = -meanPoint.x / speedPoint.x;
            float ballYwhereBallXisZero = timeWhereBallXisZero*speedPoint.y + meanPoint.y;

            if (ballYwhereBallXisZero > 0){
                // right hand
                cv::Point2f interceptSpeed = cv::Point2f(-speedPoint.y, speedPoint.x);
                float intersectTime = meanPoint.x / (interceptSpeed.x - speedPoint.x);
                interceptPos = cv::Point2f(interceptSpeed.x*intersectTime, interceptSpeed.y*intersectTime);
            } else if (ballYwhereBallXisZero < 0) {
                // left hand
                cv::Point2f interceptSpeed = cv::Point2f(speedPoint.y, -speedPoint.x);
                float intersectTime = meanPoint.x / (interceptSpeed.x - speedPoint.x);
                interceptPos = cv::Point2f(interceptSpeed.x*intersectTime, interceptSpeed.y*intersectTime);
                //determine interceptpos
            } else if (ballYwhereBallXisZero == 0) {
                interceptPos = meanPoint;
            }

        }

        // detemrine topdown interceptpos
        cv::line(topDown, cameraXandY, cameraXandY+(interceptPos*1), bluegray, 2); //speed line

        cv::String text1 = "x=" + to_string(meanPoint.x);
        cv::putText(topDown, text1, cv::Point(10, 40), cv::FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv::LINE_AA);

        cv::String text2 = "y=" + to_string(meanPoint.y);
        cv::putText(topDown, text2, cv::Point(10, 40 * 2), cv::FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv::LINE_AA);

        cv::String text3 = "speed=" + to_string(ballSpeed);
        cv::putText(topDown, text3, cv::Point(10, IMAGE_HEIGHT - 40), cv::FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv::LINE_AA);
//
//        String text4  = "y=" + to_string(y);
//        putText(topDown,text2,cv::Point(10,IMAGE_HEIGHT), FONT_HERSHEY_SIMPLEX, 1,(255,255,255),2,LINE_AA);





        cv::imshow("Top down view", topDown);
        cv::moveWindow("Top down view", 800, 600);
        // END OF LOOP //

        if (cv::waitKey(30) == 27) {
            std::cout << "esc key pressed; ending program" << std::endl;
            break;
        }

    }


    return 0;

}






// TODO make code more function / object oriented
// TODO refactor terms to show what is meant when talking about "x" "y" "radius" and "distance" in different contexts (real life vs pixels)
// TODO medium term: optimize code (use less unneeded vectors, stop using namespaces)
// TODO get rid of circularbuffer class and just use a function
// TODO add comments

// the todos below only work when we have a camera on the robot and te robot is on the field
// TODO medium term: use y position in distance estimation because we are close to ground
// TODO medium term: find edge of green field and only look within that, instead of using ypos
// TODO medium term: ask Thijs for very efficient pixel detection code

// TODO long term: correct for weird camera fish eye stuf
// TODO long term: add object tracking so that we don't panic if the ball is partially covered
// TODO long term: combine HSV with laplace or other edge detection for better ball pos
// TODO long term: use other filters instead of moving average

