// #include <stdio.h>
// #include <unistd.h>
#include <iostream>
#include <algorithm>
#include <optional>

#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/tracking/tracking_legacy.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>

using namespace cv;

// The frame size is 960x720, assume drone is at centre
const cv::Point2i DRONE_POSITION(480, 360);
// amount of centimeters to move per pixel
const float CM_PER_PIXEL = 0.3;
// minimum centimeters the drone can move, defined in tello SDK
const int MIN_STEP = 20;
// maximum centimeters the drone can move
const int MAX_STEP = 60;

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

// movement of the drone based off where the roi is in the image
std::pair<std::string, Point2i> Steer(const Point2i &origin,
                                      const Point2i &target,
                                      const float cm_per_pixel,
                                      const int min_step,
                                      const int max_step)
{
    std::string command;
    const Point2i velocity{target - origin};
    if (abs(velocity.x) > abs(velocity.y))
    {
        auto step = abs(static_cast<int>(velocity.x * cm_per_pixel));
        if (step <= min_step)
        {
            return {"", velocity};
        }
        step = std::min(step, max_step);
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
        auto step = abs(static_cast<int>(velocity.y * cm_per_pixel));
        if (step <= min_step)
        {
            return {"", velocity};
        }
        step = std::min(step, max_step);
        if (velocity.y < 0)
        {
            command = "up " + std::to_string(step);
        }
        else
        {
            command = "down " + std::to_string(step);
        }
    }
    return {command, velocity};
}

void drawMovement(Mat &image, const Point2i &drone_pos, const Point2i &velocity)
{
    const cv::Point2i x_pos(drone_pos.x + velocity.x, drone_pos.y);
    const cv::Point2i y_pos(drone_pos.x, drone_pos.y + velocity.y);
    if (abs(velocity.x) > abs(velocity.y))
    {
        cv::arrowedLine(image, drone_pos, x_pos, {0, 255, 0});
        cv::arrowedLine(image, drone_pos, y_pos, {0, 0, 255});
    }
    else
    {
        cv::arrowedLine(image, drone_pos, x_pos, {0, 0, 255});
        cv::arrowedLine(image, drone_pos, y_pos, {0, 255, 0});
    }
}

int main()
{
    // set input video
    cv::VideoCapture cap(0);
    if (!cap.isOpened())
    {
        std::cout << "cannot open camera"  << std::endl;
    }

    cv::Rect roi; // Region of Interest
    cv::Mat frame;

    cap >> frame;
    // Get width and height
    int width = frame.cols;
    int height = frame.rows;
    std::cout << "Image Width: " << width << std::endl;
    std::cout << "Image Height: " << height << std::endl;

    // Output video
    double fps = cap.get(cv::CAP_PROP_FPS);
    // Define the codec and video writer objects
    VideoWriter clean_video("video-output/clean_out.avi",
                            cv::VideoWriter::fourcc('M', 'J', 'P', 'G'),
                            fps,
                            cv::Size(960, 720));
    VideoWriter video("video-output/out.avi",
                      cv::VideoWriter::fourcc('M', 'J', 'P', 'G'),
                      fps,
                      cv::Size(960, 720));

    // create a tracker object
    // Ptr<Tracker> tracker = TrackerKCF::create(); // doesn't scale
    // Ptr<Tracker> tracker = TrackerMIL::create();
    // Ptr<Tracker> tracker = cv::legacy::TrackerMedianFlow::create();
    // Ptr<Tracker> tracker = cv::legacy::TrackerMOSSE:create();
    // Ptr<Tracker> tracker = cv::legacy::TrackerTLD:create();
    cv::Ptr<cv::Tracker> tracker = cv::TrackerCSRT::create(); // seems the fastest

    // perform the tracking process
    std::cout << "To start the tracking process draw box around ROI, press ESC to quit." << std::endl;
    namedWindow("Video Stream", WINDOW_AUTOSIZE);
    setMouseCallback("Video Stream", onMouse, 0);

    cv::Mat frame1;
    while (true)
    {

        // get frame from the video
        cap >> frame1;
        // stop the program if no more images
        if (frame1.empty())
        {
            break;
        }

        // Resize the webcam to match drone video size
        cv::resize(frame1, frame, cv::Size(960, 720));
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
            const auto steer = Steer(DRONE_POSITION, centre, CM_PER_PIXEL, MIN_STEP, MAX_STEP);
            const std::string command{steer.first};
            if (!command.empty())
            {
                std::cout << "Command: " << command << std::endl;

                // draw velocity lines (green for selected red for not selected)
                drawMovement(image, DRONE_POSITION, steer.second);
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

        // Write the frame (unedited image) into output file
        clean_video.write(frame);
        // Write the image (edited image) into output file
        video.write(image);

        // cv::resize(image, resize_image, cv::Size(width, height));
        cv::imshow("Video Stream", image);
        // quit on ESC button
        if (waitKey(1) == 27)
        {
            cap.release();
            clean_video.release();
            video.release();
            break;
        }
    }
}