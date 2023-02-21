#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/tracking.hpp>
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
    // create a tracker object
    // Ptr<Tracker> tracker = TrackerKCF::create();
    // Ptr<Tracker> tracker = TrackerMIL::create();
    cv::Ptr<cv::Tracker> tracker = cv::TrackerCSRT::create(); // seems the fastest

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
            tracker->update(image, roi);
            // draw the tracked object
            rectangle(image, roi, cv::Scalar(255, 0, 0), 2, 1);
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