#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/tracking/tracking_legacy.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <cstring>

using namespace std;
using namespace cv;

cv::Mat image;

bool selectObject = false;
int trackObject = 0;
cv::Point origin;
cv::Rect selection;

// User draws box around object to track. This triggers tracker to start tracking
static void onMouse(int event, int x, int y, int, void *)
{
    if (selectObject)
    {
        selection.x = MIN(x, origin.x);
        selection.y = MIN(y, origin.y);
        selection.width = std::abs(x - origin.x);
        selection.height = std::abs(y - origin.y);

        selection &= cv::Rect(0, 0, image.cols, image.rows);
    }

    switch (event)
    {
    case cv::EVENT_LBUTTONDOWN:
        origin = cv::Point(x, y);
        selection = cv::Rect(x, y, 0, 0);
        selectObject = true;
        break;
    case cv::EVENT_LBUTTONUP:
        selectObject = false;
        if (selection.width > 0 && selection.height > 0)
            trackObject = -1; // Set up tracker properties in main() loop
        break;
    }
}

int main(int argc, char **argv)
{
    // set input video
    cv::VideoCapture cap(0);
    if (!cap.isOpened())
    {
        cout << "cannot open camera";
    }

    cv::Rect roi; // Region of Interest
    cv::Mat frame;
    cap >> frame;

    // Get width and height
    int width = frame.cols;
    int height = frame.rows;
    cout << "Image Width: " << width/3 << endl;
    cout << "Image Height: " << height/3 << endl;

    // create a tracker object
    // Ptr<Tracker> tracker = TrackerKCF::create();
    // Ptr<Tracker> tracker = cv::legacy::TrackerMedianFlow::create();
    // Ptr<Tracker> tracker = cv::legacy::TrackerMedianFlow::create();
    // Ptr<Tracker> tracker = cv::legacy::TrackerMOSSE:create();
    // Ptr<Tracker> tracker = cv::legacy::Tracker:create();
    // Ptr<Tracker> tracker = cv::legacy::TrackerTLD:create();
    // Ptr<Tracker> tracker = TrackerMIL::create();
    cv::Ptr<cv::Tracker> tracker = cv::TrackerCSRT::create(); // seems the fastest
    // cv::Ptr<cv::Tracker> tracker = cv::TrackerDaSiamRPN::create(); // 

    // perform the tracking process
    printf("Start the tracking process, press ESC to quit.\n");
    namedWindow("tracker", WINDOW_AUTOSIZE);
    setMouseCallback("tracker", onMouse, 0);
    for (;;)
    {
        // get frame from the video
        cap >> frame;
        // stop the program if no more images
        if (frame.empty())
            break;

        frame.copyTo(image);

        // If new object is chosen update roi and initialise tracker
        if (trackObject < 0)
        {
            roi = selection;
            // initialize the tracker
            tracker->init(image, roi);
            trackObject = 1; // Don't set up again, unless user selects new ROI
        }

        // Update tracking if roi is selected
        if (roi.width > 0 && roi.height > 0)
        {
            // update the tracking result
            if (tracker->update(image, roi)){

            } else {
                cout << "Failed to track" << endl;
            }
            // tracker->update(image, roi);

            // get centre of roi
            Point centre = (roi.br() + roi.tl()) / 2;

            // draw the tracked object
            rectangle(image, roi, cv::Scalar(255, 0, 0), 2, 1);
            circle(image, centre, 3, cv::Scalar(255, 0, 0), -1);

            // draw lines
            if (true){
                // Vertical lines
                line(image, cv::Point(width/3, 0), cv::Point(width/3, height), cv::Scalar(255, 0, 0), 1);
                line(image, cv::Point(2*width/3, 0), cv::Point(2*width/3, height), cv::Scalar(255, 0, 0), 1);
                // Horizontal lines
                line(image, cv::Point(0, height/3), cv::Point(width, height/3), cv::Scalar(255, 0, 0), 1);
                line(image, cv::Point(0, 2*height/3), cv::Point(width, 2*height/3), cv::Scalar(255, 0, 0), 1);
            }

            // Left-right check
            if (centre.x < width/3){
                cout << "LEFT" << endl;
            } else if (centre.x > 2*width/3)
            {
                cout << "RIGHT" << endl;
            }

            // up-down check
            if (centre.y < height/3){
                cout << "UP" << endl;
            } else if (centre.y > 2*height/3)
            {
                cout << "DOWN" << endl;
            }
            
        }

        // Invert colours in the selection area
        if (selectObject && selection.width > 0 && selection.height > 0)
        {
            cv::Mat roi(image, selection);
            bitwise_not(roi, roi);
        }

        // show image with the tracked object
        imshow("tracker", image);
        // quit on ESC button
        if (waitKey(1) == 27)
        {
            break;
        }
    }
    return 0;
}