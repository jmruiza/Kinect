#ifndef HEIGHTS_H
#define HEIGHTS_H

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

class Heights{

private:
    int threshold;
    int alpha;  // Contrast
    int beta;   // Brightness

    cv::Mat image;
    cv::Mat binary;
    cv::Mat temp;

public:
    Heights(cv::Mat img):
        image(img)
    {
        this->run();
    }

    void run(){
        int keypressed;
        alpha = 1;
        beta = 10;
        threshold = 75;
        cv::namedWindow("Binary");
        cv::createTrackbar("Threshold", "Binary", &threshold, 255);
        cv::createTrackbar("Contrast", "Binary", &alpha, 30);
        cv::createTrackbar("Brightness", "Binary", &beta, 100);

        do{
            process();
            cv::imshow("Binary", binary);
            keypressed = cv::waitKey(500);
        }while( keypressed != 113 && keypressed != 27);
    }

    void process(){
        image.copyTo(temp);
        for(int y=0; y<image.rows; y++){
            for(int x=0; x<image.cols; x++){
                temp.at<uchar>(y,x) = cv::saturate_cast<uchar>( (0.1 * alpha) *(temp.at<uchar>(y,x)) + beta );
            }
        }

        cv::threshold(temp,binary, threshold, 255.0, cv::THRESH_BINARY);
    }

/*
    Parameters:
    trackbarname – Name of the created trackbar.
    winname – Name of the window that will be used as a parent of the created trackbar.
    value – Optional pointer to an integer variable whose value reflects the position of the slider.
            Upon creation, the slider position is defined by this variable.
    count – Maximal position of the slider. The minimal position is always 0.
    onChange – Pointer to the function to be called every time the slider changes position.
        This function should be prototyped as void Foo(int,void*); , where the first parameter is the
        trackbar position and the second parameter is the user data (see the next parameter).
        If the callback is the NULL pointer, no callbacks are called, but only value is updated.
    userdata – User data that is passed as is to the callback. It can be used to handle trackbar
        events without using global variables.

    The function createTrackbar creates a trackbar (a slider or range control) with the specified name
    and range, assigns a variable value to be a position synchronized with the trackbar and specifies
    the callback function onChange to be called on the trackbar position change. The created trackbar
    is displayed in the specified window winname.
*/
};

#endif // HEIGHTS_H
