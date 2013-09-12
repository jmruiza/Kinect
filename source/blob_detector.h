#if !defined BLOBDETECTOR
#define BLOBDETECTOR

#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>

/**
  Class that implements the SimpleBlobDetector
  @author Juan Manuel Ruiz Aranda
  **/
class BlobDetector{

private:

    /** Flag selected filter **/
    bool filter;
    /** Flag filtered **/
    bool filtered;

    /** Input image **/
    cv::Mat image_in;
    /** Output image **/
    cv::Mat image_out;

    /** Keypoints detected **/
    std::vector<cv::KeyPoint> keypoints;

    /** Parameters for SimpleBlobDetector **/
    cv::SimpleBlobDetector::Params params;

    void printNoFiltered(){
        std::cout << "No filter has been applied" << std::endl;
    };

public:

    /** Void Constructor **/
    BlobDetector (){
        filter = filtered = false;
        params.filterByArea=false;
        params.filterByCircularity=false;
        params.filterByColor=false;
        params.filterByConvexity=false;
        params.filterByInertia=false;
    }

    /** Constructor with params
      @param image Input image
    **/
    BlobDetector (cv::Mat input):
        image_in(input),
        filter(false),
        filtered(false)
    {
        this->params.filterByArea=false;
        this->params.filterByCircularity=false;
        this->params.filterByColor=false;
        this->params.filterByConvexity=false;
        this->params.filterByInertia=false;
    }

// Setters =====================================================================

    /** Setter for input image
      @param image Input image
    **/
    void setInputImage (const cv::Mat &image){
        image.copyTo(image_in);
    }

    /** Setter fot threshold step. Defaults in opencv's code (blobdetector.cpp)
        @param thresholdStep
    **/
    void setThresholdStep (float thresholdStep = 10.0){
        params.thresholdStep = thresholdStep;
    }

    /** Setter fot threshold. Defaults in opencv's code (blobdetector.cpp)
        @param min value for threshold
        @param max value for threshold
    **/
    void setThreshold (float min = 50.0, float max = 220.0){
        params.minThreshold = min;
        params.maxThreshold = max;
    }


    /** Setter fot minRepeatability. Defaults in opencv's code (blobdetector.cpp)
        @param min value for Repeatability
    **/
    void setMinRepeatability (size_t minRepeatability = 2){
        params.minRepeatability = minRepeatability;
    }

    /** Setter fot minDistBetweenBlobs. Defaults in opencv's code (blobdetector.cpp)
        @param min value for Distance Between Blobs
    **/
    void setMinDistBetweenBlobs (float minDistBetweenBlobs = 10.0){
        params.minDistBetweenBlobs = minDistBetweenBlobs;
    }

    /** Add filter: Color. Defaults in opencv's code (blobdetector.cpp)
        @param color value for Blob Color
    **/

    // Filters ========================================================================

    void addFilter_Color (uchar color = 0){
        // params.filterByColor = true;
        params.blobColor = color;
        std::cout << "Filter deactivated (Posible bug). Choose another..." << std::endl;
        filter = false;
    }

    /** Add filter: Area. Defaults in opencv's code (blobdetector.cpp)
        @param min value Area
        @param max value Area
    **/
    void addFilter_Area (float min = 25.0, float max=5000.0){
        params.filterByArea = true;
        params.minArea = min;
        params.maxArea = max;
        filter = true;
    }

    /** Add filter: Circularity. Defaults in opencv's code (blobdetector.cpp)
        @param min value Circularity
        @param max value Circularity
    **/
    void addFilter_Circularity (float min = 0.8f, float max=std::numeric_limits<float>::max()){
        params.filterByCircularity = false;
        params.minCircularity = min;
        params.maxCircularity = max;
        filter = true;
    }

    /** Add filter: Inertia Ratio. Defaults in opencv's code (blobdetector.cpp)
        @param min value Inertia Ratio
        @param max value Inertia Ratio
    **/
    void addFilter_Inertia (float min = 0.1f, float max=std::numeric_limits<float>::max()){
        params.filterByInertia = true;
        params.minInertiaRatio = min;
        params.maxInertiaRatio = max;
        filter = true;
    }

    /** Add filter: Convexity. Defaults in opencv's code (blobdetector.cpp)
        @param min value Convexity
        @param max value Convexity
    **/
    void addFilter_Convexity(float min = 0.95f, float max=std::numeric_limits<float>::max()){
        params.filterByConvexity = true;
        params.minConvexity = min;
        params.maxConvexity = max;
        filter = true;
    }

// Getters =====================================================================

    /** Getter of output image
        @return Output image
    **/
    cv::Mat getOutput (){
        if(filtered){
            drawKeypoints(false);
            return image_out;
        }
        std::cout << "Error: BlobDetector::getOutput(): ";
        printNoFiltered();
        return image_in;
    }

    /** Prints the detected points in input image **/
    void printKeypoints (){
        if(filtered){
            std::cout << "Keypoints: " << keypoints.size() << std::endl;
            for (int i=0; i<keypoints.size(); i++)
                std::cout << "Keypoint [" << i << "]: (" << round(keypoints[i].pt.x) << ", " << round(keypoints[i].pt.y) << ")" << std::endl;
        }
        else{
            std::cout << "Error: BlobDetector::printKeypoints(): ";
            printNoFiltered();
        }
    }

    /** Draw the keypoints in output image, with option to show output image
        @param show_image Option to show keypoints in output image (Default: true)
    **/
    void drawKeypoints (bool show_image=true){
        if(filtered){
            // Drawing KeyPoints
            cv::drawKeypoints(image_in, keypoints, image_out, cv::Scalar(0, 0, 255));
        }
        else{
            image_in.copyTo(image_out);
            std::cout << "Error: BlobDetector::drawKeypoints(): ";
            printNoFiltered();
        }

        if(show_image){
            cv::namedWindow("Blob Detector");
            cv::imshow("Blob Detector", image_out);
            cv::waitKey();
        }

    }

// Processing functions ========================================================

    /** Process function, apply blob detector in input image **/
    void process (){
        if(filter){
            // Set up and create the detector using the parameters
            cv::Ptr<cv::FeatureDetector> blob_detector = new cv::SimpleBlobDetector(params);
            blob_detector->create("SimpleBlob");

            // Detect
            blob_detector->detect(image_in, keypoints);
            filtered = true;
        }
        else
            std::cout << "Error: BlobDetector::process(): No filter has been selected" << std::endl;
    }
};


#endif
