#ifndef HEIGHTS_H
#define HEIGHTS_H

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "morphofeatures.h"
#include "blob_detector.h"

class Heights{

private:    
    std::string filename;
    std::string filenameJPG;
    std::string filenamePCD;

    cv::Mat image;
    cv::Mat binary;
    cv::Mat temp;

    void loadFiles(){
        std::stringstream tmp1, tmp2;

        // Set filename for jpg file
        tmp1 << filename << ".jpg";
        filenameJPG = tmp1.str();
        // Set filename for pcd file
        tmp2 << filename << ".pcd";
        filenamePCD = tmp2.str();

        // std::cout << filename << "\nJPG:\t" << filenameJPG << "\nPCD:\t" << filenamePCD << std::endl;
        image = cv::imread(filenameJPG);
        if(!image.data){
            std::cout << " Error: Can't open the given parameter: \"" << filenameJPG << "\"" << std::endl;
        }
    }

    int threshold, threshold2;
    int alpha;  // Contrast
    int beta;   // Brightness

    /** Keypoints **/
    std::vector<cv::KeyPoint> keypoints;
    /** Heihgts **/
    std::vector<cv::Point> d;



    MorphoFeatures morp;
    BlobDetector bdetect;

public:
    Heights (std::string filename):
        filename(filename)
    {
        loadFiles();
    }

//    Heights(cv::Mat img):
//        image(img)
//    {
//        this->run();
//    }

    void run(){
        int keypressed;
        alpha = 9;
        beta = 0;
        threshold = 0;
        threshold2=13;
        //cv::Mat element5(5, 5, CV_8U, cv::Scalar(1));

        bdetect.addFilter_Area();

        cv::namedWindow("Binary");
        cv::createTrackbar("Threshold", "Binary", &threshold, 255);
        cv::createTrackbar("Threshold2", "Binary", &threshold2, 255);
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

        morp.setThreshold(threshold);
        binary = morp.getEdges(temp);

        cv::morphologyEx(binary, binary, cv::MORPH_OPEN, cv::Mat());
        cv::morphologyEx(binary, binary, cv::MORPH_CLOSE, cv::Mat());
        cv::threshold(binary, binary, threshold2, 255, CV_THRESH_BINARY);
        bdetect.setInputImage(binary);
        bdetect.process();
        binary = bdetect.getOutput();

        //cv::threshold(temp,binary, threshold, 255.0, cv::THRESH_BINARY);
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
