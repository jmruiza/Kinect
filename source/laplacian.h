#ifndef LAPLACIAN_H
#define LAPLACIAN_H

#include <opencv2/imgproc/imgproc.hpp>

class LaplacianZC{
private:
    // Original Image
    cv::Mat img;

    // 32-bit float image containing the Laplacian
    cv::Mat laplace;
    // Aperture size of laplacian kernel
    int aperture;

public:
    LaplacianZC() : aperture(3) {}

    // Set the aperture size of kernel
    void setAperture(int a){
        aperture = a;
    }

    // Compute the floating point Laplacian
    cv::Mat computeLaplacian(const cv::Mat& image){
        // Compute Laplacian
        cv::Laplacian(image, laplace, CV_32F, aperture);

        // Keep local copy of the image
        // (used for zero-crossings)
        img = image.clone();
        return laplace;
    }

    // Get the Laplacian result in 8-bit image
    // zero correponds to gray level 128
    // if no scale is provided, then the max value will be
    // scaled to intensity 255
    // You must call computeLaplacian before calling this
    cv::Mat getLaplacianImage(double scale=-1.0){
        if(scale<0){
            double lapmin, lapmax;
            cv::minMaxLoc(laplace, &lapmin, &lapmax);
            scale = 127/std::max(-lapmin, lapmax);
        }
        cv::Mat laplaceImage;
        laplace.convertTo(laplaceImage, CV_8U, scale, 128);
        return laplaceImage;
    }

    // Get a binary image of the zero-crossings
    // if the product of the two adjascent pixels is
    // less than threshold then this zero-crossing
    // will be ignored
    cv::Mat getZeroCrossings(float threshold=1.0){
        // Create the iterators
        cv::Mat_<float>::const_iterator it = laplace.begin<float>() + laplace.step1();
        cv::Mat_<float>::const_iterator itend = laplace.end<float>();
        cv::Mat_<float>::const_iterator itup = laplace.begin<float>();

        // Binary image initialize to white
        cv::Mat binary(laplace.size(), CV_8U, cv::Scalar(255));
        cv::Mat_<uchar>::iterator itout = binary.begin<uchar>() +  binary.step1();

        // Negate the input threshold value
        threshold *= -1.0;

        for( ; it!=itend; ++it, ++itup, ++itout){
            // if the product of two adjascent pixel is negative
            // then there is a sign change
            if( *it * *(it-1) < threshold )
                *itout = 0; // Horizontal zero-crossing
            else if( *it * *itup < threshold)
                *itout = 0; // Vertical zero-crossing
        }

        return binary;
    }

};

#endif // LAPLACIAN_H
