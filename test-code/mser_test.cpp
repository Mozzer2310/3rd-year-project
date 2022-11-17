#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>

using namespace cv;
using namespace std;

int main(int argc, char *argv[]){
    Mat image = imread(argv[1], 1);
    
    Mat img;
    image.copyTo(img);
    Ptr<MSER> mser = MSER::create();
    cout << mser->getDelta() << endl;
    vector<vector<Point> > regions;
    vector<Rect> mser_bbox;
    mser->detectRegions(img, regions, mser_bbox);

    for (int i = 0; i < regions.size(); i++){
        rectangle(img, mser_bbox[i], CV_RGB(0, 255, 0));
    }

    Mat regionsMask;
    image.copyTo(regionsMask);
    for (auto v : regions){
        for (auto p : v){
            circle(regionsMask, p, 1, CV_RGB(0, 255, 0));
        }
    }

    imshow("Display Window", img);
    waitKey(0);

    imshow("Display Window", regionsMask);
    waitKey(0);
}