/*************************
 *       OPENCV 4
 *************************/
#include <iostream>
#include <opencv2/opencv.hpp>
using namespace cv;

int main(int argc, char** argv )
{
    std::string path = "/home/yrsn/Dev/ProyectSlam/opencv/build/";
    Mat image;
    image = imread(path+"lena.jpg", 1 );
    if ( !image.data )
    {
        printf("No image data \n");
        return -1;
    }
    namedWindow("Display Image", WINDOW_AUTOSIZE );
    imshow("Display Image", image);

    std::cout << "OpenCV version is: " << CV_VERSION << std::endl;
    waitKey(0);
    return 0;
}

//To build from shell:
//g++ hello_cv.cpp -o hello_world `pkg-config --cflags --libs opencv`
