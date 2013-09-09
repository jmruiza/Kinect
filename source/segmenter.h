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

public:

    // Constructor
    Segmenter(cv::Mat img):
        image_in(img)
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

    void process(){
        // Convert image to grayscale
        cv::cvtColor(image_in, image_bin, CV_RGB2GRAY);
        // Get binary map (35-50)
        cv::threshold(image_bin, image_bin, 85.0, 255.0, cv::THRESH_BINARY_INV);
        //cv::adaptiveThreshold(img, binary, 255.0, CV_ADAPTIVE_THRESH_MEAN_C, CV_THRESH_BINARY, 5, 1);

        // Eliminate noise and smaller objects (foreground)
        cv::erode(image_bin, image_foreground, cv::Mat(), cv::Point(-1,-1), 9);

        // Identify image pixels without objects (background)
        cv::dilate(image_bin, image_background, cv::Mat(), cv::Point(-1,-1), 9);
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
                image_segmented.at<cv::Vec3b>(j,i)[1] = 255;
                image_segmented.at<cv::Vec3b>(j,i)[2] = 255;
                }
            }
        }
    }
};


#endif
