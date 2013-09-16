#ifndef MORPHOFEATURES_H
#define MORPHOFEATURES_H

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

class MorphoFeatures
{
private:
    // Threshold to produce binary image
    int threshold;
    // Structuring elements used un coner detection
    cv::Mat cross;
    cv::Mat diamond;
    cv::Mat square;
    cv::Mat x;

    void applyThreshold(cv::Mat& result){
        // Apply threshold on result
        if (threshold>0)
            cv::threshold(result, result, threshold, 255, cv::THRESH_BINARY);
    }

public:
    MorphoFeatures(){}

    void setThreshold(int value){
        threshold = value;
    }

    cv::Mat getEdges(const cv::Mat &image){
        // Get the gradient image
        cv::Mat result;
        cv::morphologyEx(image, result, cv::MORPH_GRADIENT, cv::Mat());
        // Apply threshold to obtain a binary image
        applyThreshold(result);

        return result;
    }

};

#endif // MORPHOFEATURES_H
