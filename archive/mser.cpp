#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

int main(int argc, char *argv[]){
    Mat image;

    VideoCapture cap(0);

    if (!cap.isOpened()){
        cout << "cannot open camera";
    }

    while (true){
        cap >> image;

        Mat vis;
        image.copyTo(vis);

        Ptr<MSER> mser = MSER::create();
        mser->setDelta(10);
        vector<vector<Point> > regions;
        vector<Rect> mser_bbox;
        mser->detectRegions(vis, regions, mser_bbox);

        for (int i = 0; i < regions.size(); i++){
            rectangle(vis, mser_bbox[i], CV_RGB(0, 255, 0));
        }
        
        // cout << regions[0].size() << endl;

        // for (auto vec : regions){
        //     for (auto v : regions){
        //         cout << v << endl;
        //     }
        // }

        imshow("Display Window", vis);
        if (waitKey(1) == 27)
        {
            break;
        }
    }
}