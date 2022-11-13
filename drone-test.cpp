#include <stdio.h>
#include <unistd.h>
#include <iostream>

#include "ctello.h"
#include "opencv2/opencv.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>

const char* const TELLO_STREAM_URL{"udp://0.0.0.0:11111"};

using ctello::Tello;
using cv::CAP_FFMPEG;
using cv::imshow;
using cv::VideoCapture;
using cv::waitKey;
using cv::resize;

int width = 320; // width of image
int height = 240; // height of image
int startCounter = 1; // 0 for flight 1 for testing

int main(){
    std::cout << "test" << std::endl;
    Tello tello{};
    if (!tello.Bind())
    {
        return 0;
    }

    // Get battery level and display it
    std::optional<std::string> response;
    tello.SendCommand("battery?");
    while(!(response = tello.ReceiveResponse()));
    std::cout << "Battery Level: " << *response << std::endl;

    // Get video feed from tello
    tello.SendCommand("streamon");
    while (!(tello.ReceiveResponse()));
    VideoCapture capture{TELLO_STREAM_URL, CAP_FFMPEG};

    while (true){

        // Get the image from the video
        cv::Mat frame;
        capture >> frame;

        // Do some flight
        if (startCounter == 0){
            tello.SendCommand("takeoff");
            while (!(tello.ReceiveResponse()));
            sleep(5);
            tello.SendCommand("cw 90");
            while (!(tello.ReceiveResponse()));
            sleep(5);
            tello.SendCommand("left 20");
            while (!(tello.ReceiveResponse()));
            sleep(5);
            tello.SendCommand("land");
            while (!(tello.ReceiveResponse()));

            startCounter = 1;
        }

        // Display image
        cv::Mat resize_frame;
        cv::resize(frame, resize_frame, cv::Size(width, height));
        cv::imshow("CTello Stream", resize_frame);
        if (waitKey(1) == 27)
        {
            break;
        }
    }
}