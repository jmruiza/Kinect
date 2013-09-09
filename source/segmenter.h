#if !defined SEGMENTER
#define SEGMENTER

#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

class Segmenter {

private:
    // Input image
    cv::Mat image_in;
    // Output image
    cv::Mat image_segmented;

    // Auxiliary images
    cv::Mat image_markers;
    cv::Mat image_bin;
    cv::Mat image_foreground;
    cv::Mat image_background;

    int threshold;
    int threshold_background;
    int it_dilate;
    int it_erode;

public:
    // Constructor
    Segmenter(cv::Mat img, int threshold=125, int threshold_background=1,
              int it_dilate=9, int it_erode=9):
        image_in(img),
        threshold(threshold),
        threshold_background(threshold_background),
        it_dilate(threshold_background),
        it_erode(threshold_background)
    {
        this->process();
    }


    // Setters and getters
    void set_image_in(const cv::Mat &img){
        img.copyTo(image_in);
    }

    cv::Mat get_image_segmented(){
        return image_segmented;
    }

    void setMarkers(const cv::Mat& markerImage){
        // Convert to image of ints
        markerImage.convertTo(image_markers,CV_32S);
    }

    // Return result in the form of an image
    cv::Mat get_Segmentation(){
        cv::Mat tmp;
        // all segment with label higher than 255
        // will be assigned value 255
        image_markers.convertTo(tmp, CV_8U);
        return tmp;
    }

    // Return watershed in the form of an image
    cv::Mat get_Watersheds(){
        cv::Mat tmp;
        image_markers.convertTo(tmp, CV_8U, 255, 255);
        return tmp;
    }

    cv::Mat get_Binary(){
        return image_bin;
    }

    void process(){
        // Convert image to grayscale
        cv::cvtColor(image_in, image_bin, CV_RGB2GRAY);
        // Get binary map (35-50)
        cv::threshold(image_bin, image_bin, threshold, 255.0, cv::THRESH_BINARY_INV);

        // Eliminate noise and smaller objects (foreground)
        cv::erode(image_bin, image_foreground, cv::Mat(), cv::Point(-1,-1), 9);

        // Identify image pixels without objects (background)
        cv::dilate(image_bin, image_background, cv::Mat(), cv::Point(-1,-1), it_dilate);
        cv::threshold(image_background, image_background, 1, 128, cv::THRESH_BINARY_INV);

        // Markers image
        image_markers = image_foreground + image_background;
        image_markers.convertTo(image_markers, CV_32S);

        // Apply watershed
        cv::watershed(image_in, image_markers);

        // Apply segmentation in the image
        cv::Mat tmp;
        image_markers.convertTo(tmp, CV_8U, 255, 255);
        image_in.copyTo(image_segmented);

        for(int j=1; j<tmp.rows-1; j++){
            for(int i=1; i<tmp.cols-1; i++){
                if(tmp.at<uchar>(j,i) != 255){
                image_segmented.at<cv::Vec3b>(j,i)[0] = 0;
                image_segmented.at<cv::Vec3b>(j,i)[1] = 0;
                image_segmented.at<cv::Vec3b>(j,i)[2] = 255;
                }
            }
        }
    }

    void run(){
        int keypress;
        cv::namedWindow("Binary", cv::WINDOW_NORMAL);
        cv::namedWindow("Segmented");

        cv::createTrackbar("Threshold: ", "Binary", &threshold, 255);
        cv::createTrackbar("T Back: ", "Binary", &threshold_background, 255);
        cv::createTrackbar("Erode: ", "Binary", &it_erode, 100);
        cv::createTrackbar("Dilate: ", "Binary", &it_dilate, 100);


        do{
            process();
            cv::imshow("Segmented", image_segmented);
            cv::imshow("Binary", get_Segmentation());

            keypress = cv::waitKey(250);
            // Enter - 13

            // std::cout << threshold << std::endl;
        }while( keypress != 113 && keypress != 27);
    }
};


#endif
