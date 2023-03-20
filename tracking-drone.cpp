#include <algorithm>
#include <iostream>
#include <optional>

#include "ctello.h"
#include <opencv2/core/utility.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/tracking/tracking_legacy.hpp>
#include <opencv2/videoio.hpp>

const char *const TELLO_STREAM_URL{"udp://0.0.0.0:11111"};

using namespace cv;
using namespace ctello;

// Variable instantiation

// 0 for flight 1 for testing
bool doFlight = false;

// Used in drawing the ROI around the object
bool selectObject = false;
// Used to start object tracking or select new ROI
int trackObject = 0;

cv::Mat image;
cv::Point2i origin;
cv::Rect selection;
// The frame size is 960x720, assume drone is at centre
const cv::Point2i DRONE_POSITION(480, 360);
// Amount of centimeters to move per pixel
const float CM_PER_PIXEL = 0.3;
// Minimum centimeters the drone can move, defined in tello SDK
const int MIN_STEP = 20;
// Maximum centimeters the drone can move
const int MAX_STEP = 60;
// default output filenames
const std::string CLEAN = "video-output/out.avi";
const std::string DIRTY = "video-output/out_dirty.avi";

/**
 * @brief User draws box around object to track. This triggers tracker to start
 * tracking.
 *
 * @param   event   The event to decide what action to take
 * @param   x       The x coordinate
 * @param   y       The y coordinate
 */
static void onMouse(int event, int x, int y, int, void *) {
    if (selectObject) {
        selection.x = MIN(x, origin.x);
        selection.y = MIN(y, origin.y);
        selection.width = std::abs(x - origin.x);
        selection.height = std::abs(y - origin.y);

        selection &= cv::Rect(0, 0, image.cols, image.rows);
    }

    switch (event) {
    case cv::EVENT_LBUTTONDOWN:
        origin = cv::Point(x, y);
        selection = cv::Rect(x, y, 0, 0);
        selectObject = true;
        break;
    case cv::EVENT_LBUTTONUP:
        selectObject = false;
        if (selection.width > 0 && selection.height > 0) {
            // Set up tracker properties in main() loop
            trackObject = -1;
        }
        break;
    }
}

/**
 * @brief Generates a command as a string based off the drone position and the
 * centre of the ROI around the object being tracked.
 *
 * @param   origin          The position of the drone
 * @param   target          The centre of the ROI around the object being
 * tracked
 * @param   cm_per_pixel    The number of cms to move per pixel
 * @param   min_step        The minimum number of cms the drone can move
 * @param   max_step        The maximum number of cms the drone can move
 * @return                  A pair containing the `command` as a string and the
 * `velocity` as a point
 */
std::pair<std::string, Point2i> Steer(const Point2i &origin,
                                      const Point2i &target,
                                      const float cm_per_pixel,
                                      const int min_step, const int max_step) {
    std::string command;
    const Point2i velocity{target - origin};
    // Horizontal difference larger than vertical difference
    if (abs(velocity.x) > abs(velocity.y)) {
        // Convert pixel velocity to cm velocity and absolute the value
        int step = abs(static_cast<int>(velocity.x * cm_per_pixel));
        if (step <= min_step) {
            // Return an empty command if movement is less than minimum step
            return {"", velocity};
        }
        step = std::min(step, max_step);
        // Return right or left depending on sign of velocity
        if (velocity.x > 0) {
            command = "right " + std::to_string(step);
        } else {
            command = "left " + std::to_string(step);
        }
    } else {
        // Convert pixel velocity to cm velocity and absolute the value
        int step = abs(static_cast<int>(velocity.y * cm_per_pixel));
        if (step <= min_step) {
            // Return an empty command if movement is less than minimum step
            return {"", velocity};
        }
        step = std::min(step, max_step);
        // Return up or down depending on sign of velocity
        if (velocity.y < 0) {
            command = "up " + std::to_string(step);
        } else {
            command = "down " + std::to_string(step);
        }
    }
    return {command, velocity};
}

/**
 * Draws arrows to represent the movement of the drone.
 * Green arrow is the movement the drone is making,
 * the red arrow is movement the drone is not making.
 *
 * @param   image       The current image
 * @param   drone_pos   The position of the drone
 * @param   velocity    The movement needed for drone_pos to match the object's
 * position
 */
void drawMovement(Mat &image, const Point2i &drone_pos,
                  const Point2i &velocity) {
    // Define two points for the horizontal and vertical velocity values
    const cv::Point2i x_pos(drone_pos.x + velocity.x, drone_pos.y);
    const cv::Point2i y_pos(drone_pos.x, drone_pos.y + velocity.y);
    /// Draw the arrows, colour depends on which value is larger
    // Green for larger (movement drone has selected)
    // Red for smaller (movement not selected)
    if (abs(velocity.x) > abs(velocity.y)) {
        cv::arrowedLine(image, drone_pos, x_pos, {0, 255, 0});
        cv::arrowedLine(image, drone_pos, y_pos, {0, 0, 255});
    } else {
        cv::arrowedLine(image, drone_pos, x_pos, {0, 0, 255});
        cv::arrowedLine(image, drone_pos, y_pos, {0, 255, 0});
    }
}

/**
 * @brief Renames the output files to a user specified name.
 *
 * @param clean_default The default name for the clean output video file
 * @param dirty_default The default name for the dirty output video file
 */
void renameOutputs(const std::string clean_default,
                   const std::string dirty_default) {
    std::string output_name;
    std::cout << "\nSpecify output filename for video, if none specified then "
                 "default will be used, this will overwrite anything saved to "
                 "the same filename"
              << std::endl;
    std::cout << "Output filename: ";
    getline(std::cin, output_name);
    if (!output_name.empty()) {
        std::string clean_name = "video-output/" + output_name + ".avi";
        std::string dirty_name = "video-output/" + output_name + "_dirty.avi";
        if (rename(clean_default.c_str(), clean_name.c_str()) != 0) {
            std::cout << "Error moving file" << std::endl;
        } else {
            std::cout << "File saved successfully" << std::endl;
        }
        if (rename(dirty_default.c_str(), dirty_name.c_str()) != 0) {
            std::cout << "Error moving file" << std::endl;
        } else {
            std::cout << "File saved successfully" << std::endl;
        }
    }
}

int main() {
    // Instantiate tello object and bind to connected drone
    ctello::Tello tello{};
    if (!tello.Bind()) {
        return 0;
    }

    // Get video feed from tello drone
    tello.SendCommand("streamon");
    while (!(tello.ReceiveResponse()))
        ;
    VideoCapture cap{TELLO_STREAM_URL, CAP_FFMPEG};
    if (!cap.isOpened()) {
        std::cout << "cannot open camera" << std::endl;
        return 0;
    }

    cv::Rect roi; // Region of Interest
    cv::Mat frame;

    // Get the first frame in order to determine width and height of image
    cap >> frame;
    int width = frame.cols;
    int height = frame.rows;
    std::cout << "Image Width: " << width << std::endl;
    std::cout << "Image Height: " << height << std::endl;

    // Output video
    double fps = cap.get(cv::CAP_PROP_FPS);
    /// Define the codec and video writer objects
    // `clean_video` - will save the original frame
    // `video` - will save the frame with bounding boxes and other items drawn,
    // for evaluation
    VideoWriter clean_video("video-output/clean_out.avi",
                            cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), fps,
                            cv::Size(width, height));
    VideoWriter video("video-output/out.avi",
                      cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), fps,
                      cv::Size(width, height));

    // create a tracker object
    // Ptr<Tracker> tracker = TrackerKCF::create(); // doesn't scale
    // Ptr<Tracker> tracker = TrackerMIL::create();
    // Ptr<Tracker> tracker = cv::legacy::TrackerMedianFlow::create();
    // Ptr<Tracker> tracker = cv::legacy::TrackerMOSSE:create();
    // Ptr<Tracker> tracker = cv::legacy::TrackerTLD:create();
    cv::Ptr<cv::Tracker> tracker =
        cv::TrackerCSRT::create(); // seems the fastest

    // Show information
    std::cout << "To start the tracking process draw box around ROI, press ESC "
                 "to quit."
              << std::endl;

    // Create window and mouse callback for ROI selection
    namedWindow("CTello Stream", WINDOW_AUTOSIZE);
    setMouseCallback("CTello Stream", onMouse, 0);

    if (doFlight) {
        tello.SendCommand("takeoff");
        while (!(tello.ReceiveResponse()))
            ;
    }

    bool busy = false;
    while (true) {
        // Get frame from the video
        cap >> frame;
        // Stop the program if no more images
        if (frame.empty()) {
            break;
        }

        // Copy frame so it isn't edited
        frame.copyTo(image);

        // Listen for drone response, the drone can only move once it has
        // completed its previous command
        if (const auto response = tello.ReceiveResponse()) {
            std::cout << "Tello: " << *response << std::endl;
            busy = false;
        }

        // If new object is chosen update roi and initialise tracker
        if (trackObject < 0) {
            roi = selection;
            // initialize the tracker
            tracker->init(image, roi);
            trackObject = 1; // Don't set up again, unless user selects new ROI
        }

        // Update tracking if roi is selected
        if (roi.width > 0 && roi.height > 0) {
            // update the tracking result
            tracker->update(image, roi);

            // TODO:
            // Scaling of roi based on previous roi(s), using interpolation
            // - If there is a large change we wouldn't really expect that so
            // this becomes smaller
            // - Hopefully improve performance for less accurately tracked
            // objects
            // - Smooth variations in bounding box
            // Notes:
            // Scaling seems good with object with unique colour and shape, i.e.
            // mclaren hat

            // Get centre of roi
            Point2i object_centre = (roi.br() + roi.tl()) / 2;

            // Draw the tracked object
            rectangle(image, roi, cv::Scalar(255, 0, 0), 2, 1);
            circle(image, object_centre, 3, cv::Scalar(255, 0, 0));

            // Call Steer and store the returned pair object {command, velocity}
            const auto steer = Steer(DRONE_POSITION, object_centre,
                                     CM_PER_PIXEL, MIN_STEP, MAX_STEP);
            // Get the command to send to the drone, returned by Steer
            const std::string command{steer.first};
            if (!command.empty()) {
                if (!busy) {
                    if (doFlight) {
                        // Send the command to the drone if it is not busy and
                        // program is in flight mode
                        tello.SendCommand(command);
                    }
                    // Output command and set drone as busy
                    std::cout << "Command: " << command << std::endl;
                    busy = true;
                }

                // Draw velocity lines (green for selected red for not selected)
                drawMovement(image, DRONE_POSITION, steer.second);
            }

            // TODO:
            // Forwards/backwards movement
            // - Drone wants to keep ROI roughly the same size
            // - Base size will be initial size defined +/- padding, to account
            // for small variations
            // - Move drone backwards if ROI gets larger
            // - Move drone forwards if ROI gets smaller
            // Notes:
            // - Set a cm value based on scaling factor difference
            // - Will need min value to move from so drone isn't jittery
        }

        // Invert colours in the selection area
        if (selectObject && selection.width > 0 && selection.height > 0) {
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
        cv::imshow("CTello Stream", image);
        // Quit on ESC button
        if (waitKey(1) == 27) {
            cv::destroyAllWindows();
            if (doFlight) {
                tello.SendCommand("land");
                while (!(tello.ReceiveResponse()))
                    ;
            }
            renameOutputs(CLEAN, DIRTY);
            cap.release();
            clean_video.release();
            video.release();
            break;
        }
    }
}