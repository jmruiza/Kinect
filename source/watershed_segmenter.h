#if !defined WATERSHS
#define WATERSHS

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

class WatershedSegmenter {

private:
    cv::Mat image;
    cv::Mat binary;
    cv::Mat markers;
    cv::Mat segmented;
    cv::Mat foreground;
    cv::Mat background;

public:
    // Constructor
    WatershedSegmenter(){}

    WatershedSegmenter(cv::Mat &img){
        process(img);
    }

    // Process function
    void process(const cv::Mat &img){
        img.copyTo(image);

        // Convert image to grayscale
        cv::cvtColor(image, binary, CV_RGB2GRAY);
        // Get binary map (35-50)
        cv::threshold(binary, binary, 85.0, 255.0, cv::THRESH_BINARY_INV);
        //cv::adaptiveThreshold(img, binary, 255.0, CV_ADAPTIVE_THRESH_MEAN_C, CV_THRESH_BINARY, 5, 1);

        // Eliminate noise and smaller objects (foreground)
        cv::erode(binary, foreground, cv::Mat(), cv::Point(-1,-1), 9);

        // Identify image pixels without objects (background)
        cv::dilate(binary, background, cv::Mat(), cv::Point(-1,-1), 9);
        cv::threshold(background, background, 1, 128, cv::THRESH_BINARY_INV);

        // Markers image
        markers = foreground + background;
        markers.convertTo(markers,CV_32S);

        // Apply watershed
        cv::watershed(image, markers);
    }

    /* Getters and Setters */
    void setMarkers(const cv::Mat& markerImage){
		// Convert to image of ints
        markerImage.convertTo(markers,CV_32S);
    }

    // Return result in the form of an image
    cv::Mat getSegmentation(){
        cv::Mat tmp;
		// all segment with label higher than 255
		// will be assigned value 255
        markers.convertTo(tmp, CV_8U);
		return tmp;
    }

    // Return watershed in the form of an image
    cv::Mat getWatersheds(){
        cv::Mat tmp;
        markers.convertTo(tmp, CV_8U, 255, 255);
		return tmp;
    }

    cv::Mat getSegmentedImage(){
        cv::Mat tmp;
        markers.convertTo(tmp, CV_8U, 255, 255);
        image.copyTo(segmented);

        for(int j=1; j<tmp.rows-1; j++){
            for(int i=1; i<tmp.cols-1; i++){
                if(tmp.at<uchar>(j,i) != 255){
                segmented.at<cv::Vec3b>(j,i)[0] = 0;
                segmented.at<cv::Vec3b>(j,i)[1] = 255;
                segmented.at<cv::Vec3b>(j,i)[2] = 255;
                }
            }
        }
        return segmented;
    }
};


#endif
