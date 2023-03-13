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

// The frame size is 960x720, assume drone is at centre
const cv::Point2i DRONE_POSITION(480, 360);
// amount of centimeters to move per pixel
const float CM_PER_PIXEL = 0.3;
// minimum centimeters the drone can move, defined in tello SDK
const int MIN_STEP = 20;
// maximum centimeters the drone can move
const int MAX_STEP = 20;

bool doFlight = false; // 0 for flight 1 for testing

// Code for selecting object to track
cv::Mat image;

bool selectObject = false;
int trackObject = 0;
cv::Point2i origin;
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

// TODO: function
// movement of the drone based off where the roi is in the image
std::string Steer(const Point2i &origin,
                  const Point2i &target,
                  const float cm_per_pixel,
                  const int min_step,
                  const int max_step)
{
    std::string command;
    const Point2i velocity{target - origin};
    if (abs(velocity.x) > abs(velocity.y))
    {
        auto step = static_cast<int>(velocity.x * cm_per_pixel);
        step = std::max(std::min(step, max_step), min_step);
        if (velocity.x > 0)
        {
            command = "right " + std::to_string(step);
        }
        else
        {
            command = "left " + std::to_string(step);
        }
    }
    else
    {
        auto step = static_cast<int>(velocity.y * cm_per_pixel);
        step = std::max(std::min(step, max_step), min_step);
        if (velocity.y < 0)
        {
            command = "up " + std::to_string(step);
        }
        else
        {
            command = "down " + std::to_string(step);
        }
    }
    return command;
}

int main()
{
    ctello::Tello tello{};
    if (!tello.Bind())
    {
        return 0;
    }

    // Get video feed from tello
    tello.SendCommand("streamon");
    while (!(tello.ReceiveResponse()))
        ;
    VideoCapture cap{TELLO_STREAM_URL, CAP_FFMPEG};
    if (!cap.isOpened())
    {
        cout << "cannot open camera";
        return 0;
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

            // TODO:
            // Scaling of roi based on previous roi(s), using interpolation
            // - If there is a large change we wouldn't really expect that so this becomes smaller
            // - Hopefully improve performance for less accurately tracked objects
            // - Smooth variations in bounding box
            // Notes:
            // Scaling seems good with object with unique colour and shape, i.e. mclaren hat

            // get centre of roi
            Point2i centre = (roi.br() + roi.tl()) / 2;

            // draw the tracked object
            rectangle(image, roi, cv::Scalar(255, 0, 0), 2, 1);
            circle(image, centre, 3, cv::Scalar(255, 0, 0));

            // TODO:
            // movement of the drone based off where the roi is in the image
            // Planar movement
            // - Consider drone at centre point (DRONE_POSITION)
            // - Calculate pixel difference horizontally and vertically
            //      - Using trig? maths
            // - Move drone accorindingly (+/- horizontal and vertical)
            //      - Min difference in order to move (testing with drone)
            // Notes:
            // - look at follow.cpp in ctello GitHub for similar
            // - Set a cm value based on number of pixels (use follow.cpp for base value)
            // - do movement of whichever is most different
            const std::string command = Steer(DRONE_POSITION, centre, CM_PER_PIXEL, MIN_STEP, MAX_STEP);
            if (!command.empty())
            {
                if (!busy)
                {
                    if (doFlight)
                    {
                        tello.SendCommand(command);
                    }
                    std::cout << "Command: " << command << std::endl;
                    busy = true;
                }
            }

            // TODO:
            // Forwards/backwards movement
            // - Drone wants to keep ROI roughly the same size
            // - Base size will be initial size defined +/- padding, to account for small variations
            // - Move drone backwards if ROI gets larger
            // - Move drone forwards if ROI gets smaller
            // Notes:
            // - Set a cm value based on scaling factor difference
            // - Will need min value to move from so drone isn't jittery
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