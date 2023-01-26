#include <stdio.h>
#include <iostream>
#include <iomanip>
#include <string>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>

using namespace cv;
using namespace std;


bool equal(const Mat& a, const Mat& b){
    if ( (a.rows != b.rows) || (a.cols != b.cols) )
        return false;
    Scalar s = sum( a - b );
    return (s[0]==0) && (s[1]==0) && (s[2]==0);
}

int main(int, char** argv) {
    string filename(argv[1]);
    bool skip_frames = false;
    if(filename == "-s") {
        skip_frames = true;
        filename = argv[2];
    }
    VideoCapture streamer(filename);
    VideoCapture seeker(filename);

    float nb_frames = (float)streamer.get(CAP_PROP_FRAME_COUNT);
    cout << "Total nb of frames: " << nb_frames << endl;

    Mat streamer_frame;
    Mat seeker_frame;
    while(1) {
        int next_frame = (int)streamer.get(CV_CAP_PROP_POS_FRAMES);
        //cout << "Streamer read" << endl;
        if(!streamer.read(streamer_frame)) {
            break;
        }
//        imshow("dude", streamer_frame);
//        waitKey();

        if(skip_frames && (next_frame % 3) != 0) {
            continue;
        }

        //cout << "Seeker moved" << endl;
        bool success = seeker.set(CV_CAP_PROP_POS_FRAMES, next_frame);
        if (!success) {
            cout << endl << "Fail, coudln't seek: " << next_frame << endl;
        }
        //cout << "Seeker read" << endl;
        seeker.read(seeker_frame);

        if(!equal(streamer_frame,seeker_frame)){
            cerr << endl << "Frames are different: " << next_frame << endl;
            imwrite("streamer.png", streamer_frame);
            imwrite("seeker.png", seeker_frame);
            return 1;
        }

        cout << "\r" << fixed << setprecision(2) << (float)next_frame*100.0 / nb_frames << '%' << flush;
    }
    streamer.release();
    seeker.release();

    cout << "Seeking works!" << endl;
    return 0;
}
