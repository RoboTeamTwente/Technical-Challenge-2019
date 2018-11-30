#include <iostream>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "constants.cpp"
#include <numeric>
#include <ctime>
#include <chrono>

using namespace cv;
using namespace std;

using realClock = chrono::high_resolution_clock;

class circularBufferPoint2f {
private:
    vector<Point2f> contents;
public:
    explicit circularBufferPoint2f(vector<Point2f> input) {
        contents = input;
    }

    void reset(vector<Point2f> input) {
        contents = input;
    }

    int getLength() {
        return contents.size();
    }

    vector<Point2f> getVector(){
        return contents;
    };

    void push(Point inputPoint) {
        // simple rotation to the right
        std::rotate(contents.rbegin(), contents.rbegin() + 1, contents.rend());
        // replace first element
        contents[0] = inputPoint;
    }

    Point getElement(int index) {
        return contents[index];
    }
};

template<typename T, typename A>
void circularPush ( std::vector<T,A> vector, T element ) {
    std::rotate(vector.rbegin(), vector.rbegin() + 1, vector.rend());
    // replace first element
    vector[0] = element;
}


int main(int argc, char **argv) {
    auto startTime = realClock::now();



    // INIT CAMERA
    VideoCapture cap(0);
    vector<Point2f> defaultBuffer = {Point2f(2,2)};
    circularBufferPoint2f buffer(defaultBuffer);

    if (!cap.isOpened()) {
        cout << "webcam failure; is another openCV program running?" << endl;
        return -1;
    }


    namedWindow("Control", CV_WINDOW_AUTOSIZE); //create control window

    cout << "starting control window" << endl;

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
    cvCreateTrackbar("LowH", "Control", &iLowH, 179); //Hue
    cvCreateTrackbar("HighH", "Control", &iHighH, 179);

    cvCreateTrackbar("LowS", "Control", &iLowS, 255); //Saturation
    cvCreateTrackbar("HighS", "Control", &iHighS, 255);

    cvCreateTrackbar("LowV", "Control", &iLowV, 255); //Value
    cvCreateTrackbar("HighV", "Control", &iHighV, 255);


    Mat imgBGR;
    Mat imgLines;
    float x = -1;
    float y = -1;
    Point2f meanPoint;
    meanPoint = Point2f(-1,-1);

    while (true) {


        vector<Point> largest_contour;
        int largest_area = 0;
        int largest_contour_index = 0;
        Rect bounding_rect;

        auto startFrameTime = realClock::now();


        frameCounter++;

        // IMAGE CAPTURE //


        bool captureSuccess = cap.read(imgBGR); // read frame from camera

        if (!captureSuccess) {
            cout << "Cannot read a frame from video stream" << endl;
            break;
        }

        if (frameCounter % 30 == 1) {
            imgLines = Mat::zeros(imgBGR.size(), CV_8UC3);
        }


        Mat imgHSV;

        cvtColor(imgBGR, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV

        Mat imgThresholded;

        inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded);

        // some filtering

        //morphological opening
        erode(imgThresholded, imgThresholded,
              getStructuringElement(MORPH_ELLIPSE, Size(MORPHOLOGICAL_OPENING_SIZE, MORPHOLOGICAL_OPENING_SIZE)));
        dilate(imgThresholded, imgThresholded,
               getStructuringElement(MORPH_ELLIPSE, Size(MORPHOLOGICAL_OPENING_SIZE, MORPHOLOGICAL_OPENING_SIZE)));

        //morphological closing
//        dilate(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(MORPHOLOGICAL_CLOSING_SIZE, MORPHOLOGICAL_CLOSING_SIZE)));
//        erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(MORPHOLOGICAL_CLOSING_SIZE, MORPHOLOGICAL_CLOSING_SIZE)));


        // CONTOUR FINDING //
        std::vector<std::vector<cv::Point> > contours;
        Mat contourOutput = imgThresholded.clone();
        findContours(contourOutput, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);


        Mat contourImage(contourOutput.size(), CV_8UC3, cv::Scalar(0, 0, 0));
        Scalar colors[3];
        colors[0] = cv::Scalar(255, 0, 0);
        colors[1] = cv::Scalar(0, 255, 0);
        colors[2] = cv::Scalar(0, 0, 255);

        vector<vector<Point> > contours_poly(contours.size());
        Point_<float> onecenter;
        float oneradius;

        for (size_t idx = 0; idx < contours.size(); idx++) {

            cv::drawContours(contourImage, contours, idx, colors[idx % 3]);

            double a = contourArea(contours[idx], false);  //  Find the area of contour
            if (a > largest_area) {

                largest_area = a;

                largest_contour = contours[idx];
                largest_contour_index = idx;                //Store the index of largest contour
                approxPolyDP(Mat(contours[idx]), contours_poly[idx], 3, true);

                minEnclosingCircle((Mat) contours_poly[idx], onecenter, oneradius);


            }
        }
        Mat drawing = Mat::zeros(imgThresholded.size(), CV_8UC3);;
        Scalar color = Scalar(255, 255, 255);

        for (int i = 0; i < contours.size(); i++) {
            drawContours(drawing, contours_poly, i, color, 1, 8, vector<Vec4i>(), 0, Point());

        }
        circle(contourImage, onecenter, (int) oneradius, color, 2, 8, 0);

        // IMAGE DRAWING //



        int currentX = onecenter.x;
        int currentY = onecenter.y;

        if (previousX >= 0 && previousY >= 0 && currentX >= 0 && currentY >= 0) {

            line(imgLines, Point(currentX, currentY), Point(previousX, previousY), Scalar(255, 0, 0), 10);
        }

        previousX = currentX;
        previousY = currentY;

        // SPEED LINE //


        imshow("Thresholded Image", imgThresholded);
        moveWindow("Thresholded Image", 0, 0);
        imgBGR = imgBGR + imgLines + contourImage;
        imshow("Original", imgBGR);
        moveWindow("Original", 0, 600);




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





        Point2f prevPoint;

        prevPoint.x = prevX;
        prevPoint.y = prevY;

        Point2f cartesianPoint;

        cartesianPoint.x = x;
        cartesianPoint.y = y;

        Point2f prevMean = meanPoint;
        vector<double> timeVector;

        if (frameCounter < BUFFER_SIZE){
            meanPoint = cartesianPoint;

//            auto currentTime= realClock::now() - startTime;
//            double currentTimeSeconds = std::chrono::duration<double>(currentTime).count();
//            timeVector.push_back(currentTimeSeconds);
            //push back
        }

        if (frameCounter == BUFFER_SIZE) {
            vector<Point2f> v = {cartesianPoint, prevPoint};
            buffer.reset(v);
        }

        if (frameCounter > BUFFER_SIZE){
            buffer.push(cartesianPoint);
        }

        if (frameCounter >=  BUFFER_SIZE) {
            vector<Point2f> points=buffer.getVector();
            Point2f zero(0.0f, 0.0f);
            Point2f sum  = accumulate(points.begin(), points.end(), zero);

            meanPoint = Point2f(sum.x / points.size(), sum.y / points.size());

//            auto currentTime= realClock::now() - startTime;
//            double currentTimeSeconds = std::chrono::duration<double>(currentTime).count();
//            circularPush(timeVector, currentTimeSeconds);


        }

        // atan uses radians

        // BALL SPEED CALC //
        auto endFrameTime = realClock::now();
        auto dT = (startFrameTime - endFrameTime);
        double dTime = std::chrono::duration<double>(dT).count(); //convert to seconds



        Point2f speedPoint;

        // TODO actually implement time circular buffer
        // TODO store derivatives in vector
        // TODO calculate average derivative
        if (COMPLICATED_DIFFERENCE_CALCULATION && frameCounter >=  BUFFER_SIZE){

            vector<Point2f> points=buffer.getVector();
            vector<Point2f> derivatives;
            for (int firstIndex = 0; firstIndex < points.size()-1; ++firstIndex){
                for (int secondIndex = firstIndex+1; secondIndex < points.size(); ++secondIndex){

                    Point2f firstPoint = buffer.getElement(firstIndex);
                    Point2f secondPoint = buffer.getElement(secondIndex);
                    double firstTime = timeVector[firstIndex];
                    double secondTime = timeVector[secondIndex];

                    Point2f derivative = (secondPoint-firstPoint)/(secondTime-firstTime);

                }
            }


                    // calculate derivative
                    // store in vector


            //calculate mean of vector


        } else {
            Point2f pointDifference = prevMean - meanPoint;
            speedPoint = pointDifference/dTime;
        }





        float ballSpeed = sqrt(speedPoint.x*speedPoint.x + speedPoint.y*speedPoint.y);

        cout << "distance=" << distance << ", angle=" << angleDegrees << endl;
        cout << "x=" << distance << ", y=" << angleDegrees << endl;
        cout << "ballspeed in cm/s:" << ballSpeed << endl;
        // DRAWING FIELD OF VIEW STUFF //


        Mat topDown = Mat::zeros(imgThresholded.size(), CV_8UC3);;

        Point2f cameraXandY(100,240);

        circle(topDown, cameraXandY, (int) 5, color, 2, 8, 0);


        float line1x = 540 * cos(-0.5 * HORIZONTAL_FOV_RADIANS);
        float line1y = 540 * sin(-0.5 * HORIZONTAL_FOV_RADIANS);

        float line2x = 540 * cos(0.5 * HORIZONTAL_FOV_RADIANS);
        float line2y = 540 * sin(0.5 * HORIZONTAL_FOV_RADIANS);

        line(topDown, Point(100 + line1x, 240 + line1y), Point(cameraXandY.x, cameraXandY.y), Scalar(255, 255, 255), 1);
        line(topDown, Point(100 + line2x, 240 + line2y), Point(cameraXandY.x, cameraXandY.y), Scalar(255, 255, 255), 1);

        Scalar orange = Scalar(2, 106, 253);
        Scalar bluegray = Scalar (255,120,120);

        Point2f topDownBallPos;
        topDownBallPos.x = 100 + meanPoint.x * 5;
        topDownBallPos.y = 240 + meanPoint.y * 5;

        circle(topDown, topDownBallPos, (int) 5, orange, 2, 8, 0); // draw orange ball

        line(topDown, topDownBallPos, (topDownBallPos+(speedPoint*1)), orange, 2); //speed line

        // interception line
        Point2f interceptPos;

        if (speedPoint.x >= 0){
            interceptPos = meanPoint;
        } else {
            float timeWhereBallXisZero = -meanPoint.x / speedPoint.x;
            float ballYwhereBallXisZero = timeWhereBallXisZero*speedPoint.y + meanPoint.y;

            if (ballYwhereBallXisZero > 0){
                // right hand
                Point2f interceptSpeed = Point2f(-speedPoint.y, speedPoint.x);
                float intersectTime = meanPoint.x / (interceptSpeed.x - speedPoint.x);
                interceptPos = Point2f(interceptSpeed.x*intersectTime, interceptSpeed.y*intersectTime);
            } else if (ballYwhereBallXisZero < 0) {
                // left hand
                Point2f interceptSpeed = Point2f(speedPoint.y, -speedPoint.x);
                float intersectTime = meanPoint.x / (interceptSpeed.x - speedPoint.x);
                interceptPos = Point2f(interceptSpeed.x*intersectTime, interceptSpeed.y*intersectTime);
                //determine interceptpos
            } else if (ballYwhereBallXisZero == 0) {
                interceptPos = meanPoint;
            }

        }

        // detemrine topdown interceptpos
        line(topDown, cameraXandY, cameraXandY+(interceptPos*1), bluegray, 2); //speed line

        String text1 = "x=" + to_string(meanPoint.x);
        putText(topDown, text1, Point(10, 40), FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, LINE_AA);

        String text2 = "y=" + to_string(meanPoint.y);
        putText(topDown, text2, Point(10, 40 * 2), FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, LINE_AA);

        String text3 = "speed=" + to_string(ballSpeed);
        putText(topDown, text3, Point(10, IMAGE_HEIGHT - 40), FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, LINE_AA);
//
//        String text4  = "y=" + to_string(y);
//        putText(topDown,text2,Point(10,IMAGE_HEIGHT), FONT_HERSHEY_SIMPLEX, 1,(255,255,255),2,LINE_AA);





        imshow("Top down view", topDown);
        moveWindow("Top down view", 800, 600);
        // END OF LOOP //

        if (waitKey(30) == 27) {
            cout << "esc key pressed; ending program" << endl;
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

