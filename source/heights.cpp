#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

int main(){
    cv::Mat image;
    cv::Mat bin;
    cv::Mat mask;

    image = cv::imread("../../../../imagenes/mech_1.jpg", 0);
    cv::threshold(image, bin, 128.0, 255.0, cv::THRESH_BINARY);
    cv::imshow("Binary", bin);

//// Eroding and dilating images using morphological filters
//    // Specifying a structuring element off size
//    cv::Mat element(7,7,CV_8U, cv::Scalar(1));

//    // Erode image
//    cv::Mat eroded;
//    // cv::erode(bin, eroded, cv::Mat()); // cv::Mat() Default structuring element
//    // The same result if use a bigger kernell or repetitively apply
//    // cv::erode(bin, eroded, element); // With 7x7 kernell
//    cv::erode(bin, eroded, cv::Mat(), cv::Point(-1,-1), 3); // Repetitively apply the same structuring element
//    cv::imshow("Eroded", eroded);

//    // Dilate image
//    cv::Mat dilated;
//    // cv::dilate(bin, dilated, cv::Mat()); // cv::Mat() Default structuring element
//    // The same result if use a bigger kernell or repetitively apply
//    // cv::dilate(bin, dilated, element); // With 7x7 kernell
//    cv::dilate(bin, dilated, cv::Mat(), cv::Point(-1,-1), 3); // Repetitively apply the same structuring element
//    cv::imshow("Dilated", dilated);

// Opening and closing images using mophological filters
    // Use a 5x5 structuring element to make the effect of filter more apparent.
    cv::Mat element5(5, 5, CV_8U, cv::Scalar(1));

    // Closing is defined as the erosion of the dilation of an image
    cv::Mat closed;
    cv::morphologyEx(bin, closed, cv::MORPH_CLOSE, element5);
    cv::imshow("Closed", closed);

    // Closing is defined as the dilation of the erosion of an image
    cv::Mat opened;
    cv::morphologyEx(bin, opened, cv::MORPH_OPEN, element5);
    cv::imshow("Opened", opened);


    cv::waitKey();
    return 0;
}
