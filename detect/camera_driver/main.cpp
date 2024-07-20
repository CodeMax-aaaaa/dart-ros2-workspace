#include <iostream>
#include <opencv2/opencv.hpp>
#include <ctime>

#include "framecapture.h"

using namespace cv;
using namespace std;
using namespace rm;

int main( )
{
    FrameCapture cap(0,1);
    Frame frame;
    cap.setExp(5);

    time_t nowtime;
    struct tm* p;
    time(&nowtime);
    p = localtime(&nowtime);
    string time = to_string(p->tm_hour) + ":" + to_string(p->tm_min);

    VideoWriter vw;
    vw.open("/home/amov/record2024/"+time+".avi", CV_FOURCC('M', 'J', 'P', 'G'), 25, Size(1280, 1024));

    while(1)
    {
        if (1)
        {
            VideoCapture vc;
            vc.open("/home/amov/record2024/2:20.avi");
            vc.read(frame.srcImg);
        }
        else
        {
            cap.getFrame(frame);
            cap.showResults();
    //        cap.getSerial().print(frame.fb);
    //        cap.save(frame);
            vw << frame.srcImg.clone();
        }
        imshow("src", frame.srcImg);


        int key = waitKey(1);
        switch (key)
        {
        case 27:
        {
            vw.release();
            exit(0);
        }
            break;
        case 's':
        {

        }
            break;
        }
    }
}
