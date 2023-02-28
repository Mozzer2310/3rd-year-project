#include <stdio.h>
#include <unistd.h>
#include <iostream>

#include "ctello.h"
#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/tracking/tracking_legacy.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>

const char *const TELLO_STREAM_URL{"udp://0.0.0.0:11111"};

using namespace std;
using namespace cv;
using namespace ctello;

// The frame size is 960x720, assum drone is at centre
const cv::Point2i DRONE_POSITION(480, 360);
bool doFlight = false; // 0 for flight 1 for testing

// Code for selecting object to track
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

int main()
{
    std::cout << CV_MAJOR_VERSION << "." << CV_MINOR_VERSION << std::endl;
    ctello::Tello tello{};
    if (!tello.Bind())
    {
        return 0;
    }

    // Get battery level and display it
    std::optional<std::string> response;
    tello.SendCommand("battery?");
    while (!(response = tello.ReceiveResponse()))
        ;
    std::cout << "Battery Level: " << *response << std::endl;

    // Get video feed from tello
    tello.SendCommand("streamon");
    while (!(tello.ReceiveResponse()))
        ;
    VideoCapture cap{TELLO_STREAM_URL, CAP_FFMPEG};
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
    cout << "Image Width: " << width << endl;
    cout << "Image Height: " << height << endl;

    // create a tracker object
    // Ptr<Tracker> tracker = TrackerKCF::create(); // doesn't scale
    // Ptr<Tracker> tracker = TrackerMIL::create();
    // Ptr<Tracker> tracker = cv::legacy::TrackerMedianFlow::create();
    // Ptr<Tracker> tracker = cv::legacy::TrackerMOSSE:create();
    // Ptr<Tracker> tracker = cv::legacy::TrackerTLD:create();
    cv::Ptr<cv::Tracker> tracker = cv::TrackerCSRT::create(); // seems the fastest

    // perform the tracking process
    printf("To start the tracking process draw box around ROI, press ESC to quit.\n");
    namedWindow("CTello Stream", WINDOW_AUTOSIZE);
    setMouseCallback("CTello Stream", onMouse, 0);

    if (doFlight)
    {
        tello.SendCommand("takeoff");
        while (!(tello.ReceiveResponse()))
            ;
    }


    bool busy = false;
    while (true)
    {

        // get frame from the video
        cap >> frame;
        // stop the program if no more images
        if (frame.empty())
        {
            break;
        }

        frame.copyTo(image);

        // Listen response
        if (const auto response = tello.ReceiveResponse())
        {
            std::cout << "Tello: " << *response << std::endl;
            busy = false;
        }

        // TODO:
        // take-off of drone and hover (do nothing else) until roi selected
        // Do some flight
        // if (startCounter == 0){
        //     tello.SendCommand("takeoff");
        //     while (!(tello.ReceiveResponse()));
        //     sleep(5);
        //     tello.SendCommand("cw 90");
        //     while (!(tello.ReceiveResponse()));
        //     sleep(5);
        //     tello.SendCommand("left 20");
        //     while (!(tello.ReceiveResponse()));
        //     sleep(5);
        //     tello.SendCommand("land");
        //     while (!(tello.ReceiveResponse()));

        //     startCounter = 1;
        // }

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
            tracker->update(image, roi);

            // get centre of roi
            Point centre = (roi.br() + roi.tl()) / 2;

            // draw the tracked object
            rectangle(image, roi, cv::Scalar(255, 0, 0), 2, 1);
            circle(image, centre, 3, cv::Scalar(255, 0, 0));

            // draw lines
            if (true)
            {
                // Vertical lines
                line(image, cv::Point(width / 3, 0), cv::Point(width / 3, height), cv::Scalar(255, 0, 0), 1);
                line(image, cv::Point(2 * width / 3, 0), cv::Point(2 * width / 3, height), cv::Scalar(255, 0, 0), 1);
                // Horizontal lines
                line(image, cv::Point(0, height / 3), cv::Point(width, height / 3), cv::Scalar(255, 0, 0), 1);
                line(image, cv::Point(0, 2 * height / 3), cv::Point(width, 2 * height / 3), cv::Scalar(255, 0, 0), 1);
            }

            // Left-right check
            if (centre.x < width / 3)
            {
                cout << "LEFT" << endl;
            }
            else if (centre.x > 2 * width / 3)
            {
                cout << "RIGHT" << endl;
            }

            // up-down check
            if (centre.y < height / 3)
            {
                cout << "UP" << endl;
            }
            else if (centre.y > 2 * height / 3)
            {
                cout << "DOWN" << endl;
            }

            // TODO:
            // movement of the drone based off where the roi is in the image
            // look at follow.cpp in ctello GitHub
        }

        // Invert colours in the selection area
        if (selectObject && selection.width > 0 && selection.height > 0)
        {
            cv::Mat roi(image, selection);
            bitwise_not(roi, roi);
        }

        // Display image
        cv::Mat resize_image;
        // cv::resize(image, resize_image, cv::Size(width, height));
        cv::imshow("CTello Stream", image);
        // quit on ESC button
        if (waitKey(1) == 27)
        {
            if (doFlight)
            {
                tello.SendCommand("land");
                while (!(tello.ReceiveResponse()))
                    ;
            }
            break;
        }
    }
}