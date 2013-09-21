#ifndef HEIGHTS_H
#define HEIGHTS_H

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>

#include "morphofeatures.h"
#include "blob_detector.h"

class Heights{

    // Define new types for existent types: Code easier
    typedef pcl::PointCloud<pcl::PointXYZ> Cloud;
    typedef typename Cloud::Ptr CloudPtr;

private:    

    /** Strings to handle filenames **/
    std::string filename;
    std::string filenameJPG;
    std::string filenamePCD;

    /** Handling Cloud **/
    CloudPtr cloud_;
    std::vector<int> index_;
    float x_min, x_max;
    float y_min, y_max;

    /** Handling Image **/
    cv::Mat image;
    cv::Mat binary;
    cv::Mat temp;
    int threshold, threshold2;
    int alpha;  // Contrast
    int beta;   // Brightness

    /** Flags **/
    bool files_exists;    

    /** Load jpeg and pcd files, and checks if exists **/
    void loadFiles(){
        std::stringstream tmp1, tmp2;
        // Set filename for jpg file
        tmp1 << filename << ".jpg";
        filenameJPG = tmp1.str();
        // Set filename for pcd file
        tmp2 << filename << ".pcd";
        filenamePCD = tmp2.str();

        // Load RGB image from jpg file
        image = cv::imread(filenameJPG);
        if(!image.data){
            std::cout << " Error: Can't open the given parameter: \"" << filenameJPG << "\"" << std::endl;
            files_exists = false;
        }

        // Load Point Cloud from pcd file
        pcl::PCDReader reader;
        if( !reader.read<pcl::PointXYZ> (filenamePCD, *cloud_) ){
            files_exists = false;
        }
    }

    /** Get the Point Cloud Dimensions **/
    void getCloudDimensions(){
        x_min = x_max = cloud_->points[0].x;
        y_min = y_max = cloud_->points[0].y;

        for (size_t i = 0; i < cloud_->points.size (); ++i){
          if( cloud_->points[i].x < x_min )
              x_min = cloud_->points[i].x;

          if( cloud_->points[i].x > x_max )
              x_max = cloud_->points[i].x;

          if( cloud_->points[i].y < y_min )
              y_min = cloud_->points[i].y;

          if( cloud_->points[i].y > y_max )
              y_max = cloud_->points[i].y;
        }

        std::cout << "Point Cloud size: " << cloud_->size() << std::endl;
        std::cout << "Size in x (width): " << cloud_->width << std::endl;
        std::cout << "Size in y (height): " << cloud_->height << std::endl;
        std::cout << "x: " << x_min << " - " << x_max << std::endl;
        std::cout << "y: " << y_min << " - " << y_max << std::endl;
    }

    /** Mapping coordinates (x,y) in the image to the coordinates of the cloud **/
    cv::Point3f getCloudCordinates(int x, int y){
        cv::Point3f mapping_(0, 0, 0);
        // Mapping x value
        mapping_.x = mapping(x_min, x_max, x, 0, image.cols-1);
        // Mapping y value
        mapping_.y = mapping(y_min, y_max, y, 0, image.rows-1);
        return mapping_;
    }

    /** Mapping function image to cloud **/
    float mapping(float yo, float yf, int x, int xo, int xf){
        float m, y;
        m =  ( (yf-yo) / (float)(xf-xo) );
        y = m * (x-xo) + yo;
        return y;
    }

    /** Mapping function cloud to image **/
    int mapping(int yo, int yf, float x, float xo, float xf){
        float m, y;
        m =  ( (yf-yo) / (float)(xf-xo) );
        y = m * (x-xo) + yo;
        return (int)(y);
    }

    std::vector<cv::Point> kinect_points;

    /** Get the Point Cloud Mapping points **/
    void getPoints(){
        cv::Point temp(0,0);

        for (size_t i = 0; i < cloud_->points.size (); ++i){
            if( !isnan(cloud_->points[i].x) && !isnan(cloud_->points[i].y) ){
                temp.x = mapping(0, image.cols-1, cloud_->points[i].x, x_min, x_max) - 22;
                temp.y = mapping(0, image.rows-1, cloud_->points[i].y, y_min, y_max) + 19;
                kinect_points.push_back(temp);
            }
        }
    }

    void drawKinectPoints( std::vector<cv::Point> &points, cv::Mat &img, bool show=true){
        for (size_t i = 0; i < points.size(); ++i){
            if( points[i].x < img.cols && points[i].y < img.rows )
                img.at<cv::Vec3b>(points[i].y,points[i].x)[2]=255;
        }

        if(show){
            cv::imshow("Points", img);
            cv::waitKey();
        }
    }


    /** Keypoints **/
    std::vector<cv::KeyPoint> keypoints;
    /** Heihgts **/

    MorphoFeatures morp;
    BlobDetector bdetect;

public:


    Heights (std::string filename):
        filename(filename),
        cloud_ (new Cloud)
    {
        loadFiles();
        getCloudDimensions();

        getPoints();
        drawKinectPoints(kinect_points, image);
        int x = 0;
        int y = 0;
        cv::Point3f point = getCloudCordinates(x,y);
        std::cout << "(" << x << "," << y << ") -> "<< "(" << point.x << ", " << point.y << ", " << point.z << ")" << std::endl;
    }

    Heights(cv::Mat img):
        image(img)
    {
        this->run();
    }

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
