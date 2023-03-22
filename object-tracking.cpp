#include <algorithm>
#include <deque>
#include <iostream>
#include <list>
#include <optional>

#include <opencv2/core/utility.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/tracking/tracking_legacy.hpp>
#include <opencv2/videoio.hpp>

using namespace cv;

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
// starting size of roi
cv::Size roi_size;
// Multiplier for max size of roi
const float ROI_MAX = 0.7;
// Multiplier for min size of roi
const float ROI_MIN = 0.05;
// Acceptable range multiplier of roi size
const float ROI_SCALE = 0.2;
// Datastructure that holds the previous roi sizes
std::deque<int> prevs_roi_size;

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
        if (selection.width > 0 && selection.height > 0)
            // Set up tracker properties in main() loop
            trackObject = -1;
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
 * @brief Generates a command to move the drone longitudinally, based on the
 * size of the ROI compared to the initial size of the ROI
 *
 * @param   original_size   The size of the ROI when it was initialised
 * @param   target_size     The size of the target object in the current frame
 * @param   min_step        The minimum number of cms the drone can move
 * @param   roi_scale       The value to define the acceptable scale 1 +/-
 * `roi_scale`
 * @return                  A `string` containing the command to give the drone
 */
std::string LongitudinalMove(const cv::Size &original_size,
                             const cv::Size &target_size, const int min_step,
                             const float roi_scale) {
    std::string command;
    // The average ratio of height and width of the two Size objects
    float ratio =
        ((static_cast<float>(target_size.width) / original_size.width) +
         ((static_cast<float>(target_size.height)) / original_size.height)) /
        2;

    /// Move backwards if target is > 1.2 times the initial size
    // Move forwards if target is < 0.8 times the initial size
    // Don't move longitudinally
    if (ratio > 1 + roi_scale) {
        command = "back " + std::to_string(min_step);
    } else if (ratio < 1 - roi_scale) {
        command = "forward " + std::to_string(min_step);
    } else {
        command = "";
    }
    return command;
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

/**
 * @brief Check if the defined ROI is within the allowed size range.
 *
 * @param   roi_size      The size of the current defined ROI
 * @param   frame_width   The width of the frame
 * @param   frame_height  The height of the frame
 * @param   roi_min       The multiplier to calculate minimum roi size
 * @param   roi_max       The multiplier to calculate maximum roi size
 * @return                `true` - When the defined ROI is of an acceptable size
 * @return                `false` - When the defined ROI is not of an acceptable
 * size
 */
bool checkROI(cv::Size roi_size, int frame_width, int frame_height,
              const float roi_min, const float roi_max) {
    if (roi_size.width > 0.7 * frame_width or
        roi_size.height > 0.7 * frame_height) {
        std::cout << "ROI too large, define area again" << std::endl;
        return false;
    } else if (roi_size.width < 0.05 * frame_width or
               roi_size.height < 0.05 * frame_height) {
        std::cout << "ROI too small, define area again" << std::endl;
        return false;
    }
    return true;
}

/**
 * @brief Measures the rate of change of the roi, to determine if sudden change
 * occurs.
 *
 * @param   roi     `cv::Rect` the current roi
 * @return          `true` - When the rate of change is safe
 * @return          `false` - When the rate of change is unsafe
 */
bool rocCheck(cv::Rect roi) {
    prevs_roi_size.push_front(roi.area());
    if (prevs_roi_size.size() > 20) {
        prevs_roi_size.pop_back();
        float ratios[19];
        for (int i = 0; i < prevs_roi_size.size() - 1; i++) {
            ratios[i] = static_cast<float>(prevs_roi_size.at(i)) /
                        prevs_roi_size.at(i + 1);
        }
        float ratio_avg = 0;
        for (float n : ratios) {
            ratio_avg = ratio_avg + n;
        }
        ratio_avg = ratio_avg / 19;
        if (!(ratio_avg < 1.1 and ratio_avg > 0.9)) {
            std::cout << "Rate of Change is UNSAFE" << std::endl;
            return false;
        }
    }
    return true;
}

/**
 * @brief Function to safely close windows and release OpenCV objects
 *
 * @param   cap             `cv::VideoCapture` object
 * @param   video_writers   List of `cv::VideoWriter` objects
 */
void exitSafe(cv::VideoCapture cap, std::list<cv::VideoWriter> video_writers) {
    cv::destroyAllWindows();
    renameOutputs(CLEAN, DIRTY);
    cap.release();
    for (auto cap : video_writers) {
        cap.release();
    }
}

int main() {
    // Set input video
    cv::VideoCapture cap(0);
    if (!cap.isOpened()) {
        std::cout << "cannot open camera" << std::endl;
    }

    cv::Rect roi; // Region of Interest
    cv::Mat frame;

    // Get the first frame in order to determine width and height of image
    cap >> frame;
    int width = 960;
    int height = 720;
    std::cout << "Image Width: " << width << std::endl;
    std::cout << "Image Height: " << height << std::endl;

    // Output video
    double fps = cap.get(cv::CAP_PROP_FPS);
    /// Define the codec and video writer objects
    // `clean_video` - will save the original frame
    // `video` - will save the frame with bounding boxes and other items drawn,
    // for evaluation
    VideoWriter clean_video(CLEAN, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'),
                            fps, cv::Size(960, 720));
    VideoWriter video(DIRTY, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), fps,
                      cv::Size(960, 720));
    std::list<cv::VideoWriter> videoWriters = {clean_video, video};

    // create a CSRT tracker object
    cv::Ptr<cv::Tracker> tracker = cv::TrackerCSRT::create();

    // Show information
    std::cout << "To start the tracking process draw box around ROI, press ESC "
                 "to quit."
              << std::endl;

    // Create window and mouse callback for ROI selection
    namedWindow("Video Stream", WINDOW_AUTOSIZE);
    setMouseCallback("Video Stream", onMouse, 0);

    cv::Mat frame1;
    while (true) {

        // Get frame from the video
        cap >> frame1;
        // Stop the program if no more images
        if (frame1.empty()) {
            exitSafe(cap, videoWriters);
            break;
        }

        // Resize the webcam to match drone video size
        cv::resize(frame1, frame, cv::Size(960, 720));
        // Copy frame so it isn't edited
        frame.copyTo(image);

        // If new object is chosen update roi and initialise tracker
        if (trackObject < 0) {
            roi = selection;
            // store the initial roi size
            roi_size = roi.size();

            // Initialize the tracker if ROI is an acceptable size, otherwise
            // reset values
            if (checkROI(roi_size, width, height, ROI_MIN, ROI_MAX)) {
                // initialize the tracker
                tracker->init(image, roi);
                // Don't set up again, unless user selects new ROI
                trackObject = 1;
                // Clear the roi size queue
                prevs_roi_size.clear();
            } else {
                trackObject = 0;
                roi = cv::Rect();
            }
        }

        // Update tracking if roi is selected
        if (roi.width > 0 && roi.height > 0) {
            // update the tracking result
            tracker->update(image, roi);

            if (!rocCheck(roi)) {
                exitSafe(cap, videoWriters);
                break;
            }

            // Get centre of roi
            Point2i object_centre = (roi.br() + roi.tl()) / 2;

            // Draw the tracked object
            rectangle(image, roi, cv::Scalar(255, 0, 0), 2, 1);
            circle(image, object_centre, 3, cv::Scalar(255, 0, 0));

            // Call Steer and store the returned pair object {command, velocity}
            const auto steer = Steer(DRONE_POSITION, object_centre,
                                     CM_PER_PIXEL, MIN_STEP, MAX_STEP);
            // Get the command to send to the drone, returned by Steer
            std::string command{steer.first};
            if (!command.empty()) {
                std::cout << "Command: " << command << std::endl;

                // Draw velocity lines (green for selected red for not selected)
                drawMovement(image, DRONE_POSITION, steer.second);
            } else {
                // If no planar movement needed check for longitudinal
                command =
                    LongitudinalMove(roi_size, roi.size(), MIN_STEP, ROI_SCALE);
                if (!command.empty()) {
                    std::cout << "Command: " << command << std::endl;
                }
            }
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
        cv::imshow("Video Stream", image);
        // Quit on ESC button
        if (waitKey(1) == 27) {
            exitSafe(cap, videoWriters);
            break;
        }
    }
}