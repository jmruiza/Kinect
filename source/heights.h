#ifndef HEIGHTS_H
#define HEIGHTS_H

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>

#include "laplacian.h"

class Heights{

    // Define new types for existent types: Code easier
    typedef pcl::PointCloud<pcl::PointXYZ> Cloud;
    typedef typename Cloud::Ptr CloudPtr;

private:    

    /** Units **/
    bool in_meters;

    /** Strings to handle filenames **/
    std::string filename;
    std::string filenameJPG;
    std::string filenamePCD;

    /** Handling Cloud **/
    CloudPtr cloud_;
    std::vector<cv::Point3f> kinect_points;
    float x_min, x_max;
    float y_min, y_max;
    float z_ref;

    /** Handling Image **/
    cv::Mat image;
    cv::Mat binary;
    cv::Mat image_copy;

    /** Depth Maps **/
    cv::Mat depth_map;
    cv::Mat depth_map_filtered;

    /** Contours and content detection **/
    cv::Mat image_colored;
    std::vector<cv::Scalar> colors;
    std::vector<cv::Vec4i> hierarchy;
    std::vector<std::vector<cv::Point> > contours;

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
        z_ref = cloud_->points[0].z;

        for (size_t i = 1; i < cloud_->points.size (); ++i){
          if( cloud_->points[i].x < x_min )
              x_min = cloud_->points[i].x;

          if( cloud_->points[i].x > x_max )
              x_max = cloud_->points[i].x;

          if( cloud_->points[i].y < y_min )
              y_min = cloud_->points[i].y;

          if( cloud_->points[i].y > y_max )
              y_max = cloud_->points[i].y;

          if( cloud_->points[i].z > z_ref && cloud_->points[i].z > 0 )
              z_ref = cloud_->points[i].z;
        }
//         std::cout << "Point Cloud size: " << cloud_->size() << std::endl;
//         std::cout << "Size in x (width): " << cloud_->width << std::endl;
//         std::cout << "Size in y (height): " << cloud_->height << std::endl;
//         std::cout << "x: " << x_min << " - " << x_max << std::endl;
//         std::cout << "y: " << y_min << " - " << y_max << std::endl;
//         std::cout << "Referencia (z): "<< z_ref << std::endl;
    }

    /** Mapping coordinates (x,y) in the image to the coordinates of the cloud
        @param x (int)
        @param y (int)
    **/
    cv::Point3f getCloudCordinates(int x, int y){
        cv::Point3f mapping_(0, 0, 0);
        // Mapping x value
        mapping_.x = mapping(x_min, x_max, x, 0, image.cols-1);
        // Mapping y value
        mapping_.y = mapping(y_min, y_max, y, 0, image.rows-1);
        return mapping_;
    }

    /** Mapping function image to cloud
        @param yo (float)
        @param yf (float)
        @param x (int)
        @param xo (int)
        @param xf (int)
    **/
    float mapping(float yo, float yf, int x, int xo, int xf){
        float m, y;
        m =  ( (yf-yo) / (float)(xf-xo) );
        y = m * (x-xo) + yo;
        return y;
    }

    /** Mapping function cloud to image
        @param yo (int)
        @param yf (int)
        @param x (float)
        @param xo (float)
        @param xf (float)
    **/
    int mapping(int yo, int yf, float x, float xo, float xf){
        float m, y;
        m =  ( (yf-yo) / (float)(xf-xo) );
        y = m * (x-xo) + yo;
        return (int)(y);
    }

    /** Get cloud correspondences in the image **/
    void getPointCloudCorrespondences(){
        cv::Point3f temp;

        for (size_t i = 0; i < cloud_->points.size (); ++i){
            if( !isnan(cloud_->points[i].x) && !isnan(cloud_->points[i].y) ){
                temp.x = mapping(0, image.cols-1, cloud_->points[i].x, x_min, x_max) - 22;
                temp.y = mapping(0, image.rows-1, cloud_->points[i].y, y_min, y_max) + 19;
                temp.z = cloud_->points[i].z;

                if(temp.x > 0 && temp.y > 0)
                    kinect_points.push_back(temp);
            }
        }
    }

    /** cloud correspondences in the image (DEMO)
        @param points (std::vector<cv::Point3f>&)
        @param img (cv::Mat&)
        @param show (bool)
    **/
    void drawKinectPoints( std::vector<cv::Point3f> &points, cv::Mat &img, bool show=true){
        for (size_t i = 0; i < points.size(); ++i){
            if( points[i].x < img.cols && points[i].y < img.rows )
                img.at<cv::Vec3b>(points[i].y,points[i].x)[2]=255;
        }

        if(show){
            cv::imshow("Points", img);
            cv::waitKey();
        }
    }

    /** Generate a float image (Matrix) with values for cloud **/
    void generateCloudImage(){
        depth_map = cv::Mat(image.rows, image.cols, CV_32F, cv::Scalar(0));

        // Filter image (Eliminate 0's)
        for(int j=0; j<depth_map.rows; j++)
            for(int i=0; i<depth_map.cols; i++)
                depth_map.at<float>(j,i) = z_ref;

        for (size_t i = 0; i < kinect_points.size(); ++i){
            depth_map.at<float>(kinect_points[i].y,kinect_points[i].x) = kinect_points[i].z;
        }
    }

    /** Mouse Event Callback **/
    static void mouseEvent(int event, int x, int y, int flags, void* param){
        cv::Point* pnt = (cv::Point*) param;

        if( event != cv::EVENT_MOUSEMOVE ) // cv::EVENT_LBUTTONDOWN
            return;

        pnt->x = x;
        pnt->y = y;
    }

    /** Add text to image Mouse Event Callback
      @param pnt is the point cloid to label
     **/
    void addLabel(cv::Point pnt){
        cv::Point ptmp = pnt;
        image.copyTo(image_copy);
        std::stringstream tmp;

        if(in_meters)
            tmp << std::fixed << std::setprecision(3) << getHeight(pnt) << "m";
        else
            tmp << std::fixed << std::setprecision(2) << 100 * getHeight(pnt) << "cm";

        cv::circle(image_copy, pnt, 1, cv::Scalar(0, 0, 255), 2);

        if(image.cols - pnt.x < 99)
            ptmp.x = image.cols-99;
        if(pnt.y < 20)
            ptmp.y = 20;

        cv::putText(image_copy, tmp.str(), ptmp, cv::FONT_HERSHEY_COMPLEX, 0.8, cv::Scalar(0,0,255));
    }

    /** Get Height of a point
    @param pnt (cv::Point)
    **/
    float getHeight(cv::Point pnt){
        //return depth_map.at<float>(pnt.y, pnt.x);
//        return z_ref - depth_map.at<float>(pnt.y, pnt.x);
        return z_ref - depth_map_filtered.at<float>(pnt.y, pnt.x);
//        return depth_map_filtered.at<float>(pnt.y, pnt.x);
    }

    /** Filter RGB image to get "regions" **/
    void filterRGBMap(){
        // Convert to grayscale
        cv::cvtColor(image, binary, CV_BGR2GRAY);

        // Filter 2D (Use a specific kernel)
        cv::Mat kernel(3, 3, CV_32F, cv::Scalar(0));
        kernel.at<float>(1,1) = 5.0;
        kernel.at<float>(0,1) = -1.0;
        kernel.at<float>(2,1) = -1.0;
        kernel.at<float>(1,0) = -1.0;
        kernel.at<float>(1,2) = -1.0;

        cv::filter2D(binary, binary, image.depth(), kernel);

        // Get edges
        LaplacianZC laplacian;
        laplacian.setAperture(11);
        cv::Mat flap = laplacian.computeLaplacian(binary);
        binary = laplacian.getLaplacianImage();

        cv::threshold(binary, binary, 123, 255, cv::THRESH_BINARY);

        image_colored = cv::Mat::zeros(image.rows, image.cols, CV_8UC3);

        // Find Contours with hierarchy
        cv::findContours(binary, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);

        // iterate through all the top-level contours, draw each connected component with its own random color
        int idx = 0;
        for( ; idx >= 0; idx = hierarchy[idx][0] ){
            cv::Scalar color( rand()&255, rand()&255, rand()&255 );
            cv::drawContours( image_colored, contours, idx, color, CV_FILLED, 8, hierarchy );
            colors.push_back(color);
        }
    }

    /** Using the colored RGB image, apply a selective filter on depth map **/
    void filterDepthMap(){

        depth_map_filtered =  cv::Mat(image.rows, image.cols, CV_32F, cv::Scalar(0));

        // Filtering content
        std::vector<cv::Scalar>::iterator it = colors.begin();
        std::vector<cv::Scalar>::iterator itend = colors.end();

        while (it != itend) {
            cv::Scalar color = (*it);
            std::vector<cv::Point> points;

            // Scanning colored image
            for(int j=0; j<image_colored.rows; j++)
                for(int i=0; i<image_colored.cols; i++){
                    if( image_colored.at<cv::Vec3b>(j,i)[0] == color[0] &&
                        image_colored.at<cv::Vec3b>(j,i)[1] == color[1] &&
                        image_colored.at<cv::Vec3b>(j,i)[2] == color[2])
                        points.push_back(cv::Point(i,j));
                }

            // Getting average
            int npoints = 0;
            float averange;

            for(int i=0; i<points.size(); i++){
                if(depth_map.at<float>(points[i].y, points[i].x) != 0){
                    averange += depth_map.at<float>(points[i].y, points[i].x);
                    npoints++;
                }
            }

            averange = averange/(float)(npoints);

            // Apply filter
            for(int i=0; i<points.size(); i++)
                depth_map_filtered.at<float>(points[i].y, points[i].x) = averange;

            ++it;
        }

        // Filtering Contours
        std::vector< std::vector <cv::Point> >::iterator c_it = contours.begin();
        std::vector< std::vector <cv::Point> >::iterator c_itend = contours.end();

        while (c_it != c_itend) {
            std::vector<cv::Point> c_points = (*c_it);

            // Getting average
            int c_npoints = 0;
            float c_averange;

            for(int i=0; i<c_points.size(); i++){
                if(depth_map.at<float>(c_points[i].y, c_points[i].x) != 0){
                    c_averange += depth_map.at<float>(c_points[i].y, c_points[i].x);
                    c_npoints++;
                }
            }

            c_averange = c_averange/(float)(c_npoints);

            // Apply filter
            for(int i=0; i<c_points.size(); i++)
                depth_map_filtered.at<float>(c_points[i].y, c_points[i].x) = c_averange;

            ++c_it;
        }
        eliminateZeroValues(depth_map_filtered);
    }

    /** Eliminate the zero values in image
        @param image (cv::Mat&)
    **/
    void eliminateZeroValues(cv::Mat &image){
        int npoints;
        float averange;

        cv::Mat_<float> nozeros = cv::Mat(image.rows, image.cols, CV_32F, cv::Scalar(0));
        image.copyTo(nozeros);

        for(int j=0; j<nozeros.rows; j++)
            for(int i=0; i<nozeros.cols; i++)
                if( round(nozeros(j,i)) <= 0 ){
                    npoints = 0;
                    averange = 0.0;
                    // Left-Up Corner
                    if(i==0 && j==0){
                        if( round(nozeros(j,i+1)) > 0 ){ averange += nozeros(j,i+1); npoints++; }
                        if( round(nozeros(j,i+2)) > 0 ){ averange += nozeros(j,i+2); npoints++; }
                        if( round(nozeros(j+1,i)) > 0 ){ averange += nozeros(j+1,i); npoints++; }
                        if( round(nozeros(j+1,i+1)) > 0 ){ averange += nozeros(j+1,i+1); npoints++; }
                        if( round(nozeros(j+1,i+2)) > 0 ){ averange += nozeros(j+1,i+2); npoints++; }
                        if( round(nozeros(j+2,i)) > 0 ){ averange += nozeros(j+2,i); npoints++; }
                        if( round(nozeros(j+2,i+1)) > 0 ){ averange += nozeros(j+2,i+1); npoints++; }
                        if( round(nozeros(j+2,i+2)) > 0 ){ averange += nozeros(j+2,i+2); npoints++; }
                    }
                    // Right-Up Corner
                    else if(i==nozeros.cols-1 && j==0){
                        if( round(nozeros(j,i-1)) > 0 ){ averange += nozeros(j,i-1); npoints++; }
                        if( round(nozeros(j,i-2)) > 0 ){ averange += nozeros(j,i-2); npoints++; }
                        if( round(nozeros(j+1,i)) > 0 ){ averange += nozeros(j+1,i); npoints++; }
                        if( round(nozeros(j+1,i-1)) > 0 ){ averange += nozeros(j+1,i-1); npoints++; }
                        if( round(nozeros(j+1,i-2)) > 0 ){ averange += nozeros(j+1,i-2); npoints++; }
                        if( round(nozeros(j+2,i)) > 0 ){ averange += nozeros(j+2,i); npoints++; }
                        if( round(nozeros(j+2,i-1)) > 0 ){ averange += nozeros(j+2,i-1); npoints++; }
                        if( round(nozeros(j+2,i-2)) > 0 ){ averange += nozeros(j+2,i-2); npoints++; }
                    }
                    // Left-Down Corner
                    else if(i==0 && j==nozeros.rows-1){
                        if( round(nozeros(j,i+1)) > 0 ){ averange += nozeros(j,i+1); npoints++; }
                        if( round(nozeros(j,i+2)) > 0 ){ averange += nozeros(j,i+2); npoints++; }
                        if( round(nozeros(j-1,i)) > 0 ){ averange += nozeros(j-1,i); npoints++; }
                        if( round(nozeros(j-1,i+1)) > 0 ){ averange += nozeros(j-1,i+1); npoints++; }
                        if( round(nozeros(j-1,i+2)) > 0 ){ averange += nozeros(j-1,i+2); npoints++; }
                        if( round(nozeros(j-2,i)) > 0 ){ averange += nozeros(j-2,i); npoints++; }
                        if( round(nozeros(j-2,i+1)) > 0 ){ averange += nozeros(j-2,i+1); npoints++; }
                        if( round(nozeros(j-2,i+2)) > 0 ){ averange += nozeros(j-2,i+2); npoints++; }
                    }
                    // Right-Down Corner
                    else if(i==nozeros.cols-1 && j==nozeros.rows-1){
                        if( round(nozeros(j,i-1)) > 0 ){ averange += nozeros(j,i-1); npoints++; }
                        if( round(nozeros(j,i-2)) > 0 ){ averange += nozeros(j,i-2); npoints++; }
                        if( round(nozeros(j-1,i)) > 0 ){ averange += nozeros(j-1,i); npoints++; }
                        if( round(nozeros(j-1,i-1)) > 0 ){ averange += nozeros(j-1,i-1); npoints++; }
                        if( round(nozeros(j-1,i-2)) > 0 ){ averange += nozeros(j-1,i-2); npoints++; }
                        if( round(nozeros(j-2,i)) > 0 ){ averange += nozeros(j-2,i); npoints++; }
                        if( round(nozeros(j-2,i-1)) > 0 ){ averange += nozeros(j-2,i-1); npoints++; }
                        if( round(nozeros(j-2,i-2)) > 0 ){ averange += nozeros(j-2,i-2); npoints++; }
                    }
                    // Left border
                    else if(i==0 && (j>0 && j<nozeros.rows-2)){
                        if( round(nozeros(j-1,i)) > 0 ){ averange += nozeros(j-1,i); npoints++; }
                        if( round(nozeros(j-1,i+1)) > 0 ){ averange += nozeros(j-1,i+1); npoints++; }
                        if( round(nozeros(j-1,i+2)) > 0 ){ averange += nozeros(j-1,i+2); npoints++; }
                        if( round(nozeros(j,i+1)) > 0 ){ averange += nozeros(j,i+1); npoints++; }
                        if( round(nozeros(j,i+2)) > 0 ){ averange += nozeros(j,i+2); npoints++; }
                        if( round(nozeros(j+1,i)) > 0 ){ averange += nozeros(j+1,i); npoints++; }
                        if( round(nozeros(j+1,i+1)) > 0 ){ averange += nozeros(j+1,i+1); npoints++; }
                        if( round(nozeros(j+1,i+2)) > 0 ){ averange += nozeros(j+1,i+2); npoints++; }
                    }
                    // Right border
                    else if(i==nozeros.cols-1 && (j>0 && j<nozeros.rows-2)){
                        if( round(nozeros(j-1,i)) > 0 ){ averange += nozeros(j-1,i); npoints++; }
                        if( round(nozeros(j-1,i-1)) > 0 ){ averange += nozeros(j-1,i-1); npoints++; }
                        if( round(nozeros(j-1,i-2)) > 0 ){ averange += nozeros(j-1,i-2); npoints++; }
                        if( round(nozeros(j,i-1)) > 0 ){ averange += nozeros(j,i-1); npoints++; }
                        if( round(nozeros(j,i-2)) > 0 ){ averange += nozeros(j,i-2); npoints++; }
                        if( round(nozeros(j+1,i)) > 0 ){ averange += nozeros(j+1,i); npoints++; }
                        if( round(nozeros(j+1,i-1)) > 0 ){ averange += nozeros(j+1,i-1); npoints++; }
                        if( round(nozeros(j+1,i-2)) > 0 ){ averange += nozeros(j+1,i-2); npoints++; }
                    }
                    // Up Border
                    else if((i>0 && i<nozeros.cols-2 ) && j==0){
                        if( round(nozeros(j,i-1)) > 0 ){ averange += nozeros(j,i-1); npoints++; }
                        if( round(nozeros(j,i+1)) > 0 ){ averange += nozeros(j,i+1); npoints++; }
                        if( round(nozeros(j+1,i-1)) > 0 ){ averange += nozeros(j+1,i-1); npoints++; }
                        if( round(nozeros(j+1,i)) > 0 ){ averange += nozeros(j+1,i); npoints++; }
                        if( round(nozeros(j+1,i+1)) > 0 ){ averange += nozeros(j+1,i+1); npoints++; }
                        if( round(nozeros(j+2,i-1)) > 0 ){ averange += nozeros(j+2,i-1); npoints++; }
                        if( round(nozeros(j+2,i)) > 0 ){ averange += nozeros(j+2,i); npoints++; }
                        if( round(nozeros(j+2,i+1)) > 0 ){ averange += nozeros(j+2,i+1); npoints++; }
                    }
                    // Down Border
                    else if((i>0 && i<nozeros.cols-2 ) && j==nozeros.rows-1){
                        if( round(nozeros(j,i-1)) > 0 ){ averange += nozeros(j,i-1); npoints++; }
                        if( round(nozeros(j,i+1)) > 0 ){ averange += nozeros(j,i+1); npoints++; }
                        if( round(nozeros(j-1,i-1)) > 0 ){ averange += nozeros(j-1,i-1); npoints++; }
                        if( round(nozeros(j-1,i)) > 0 ){ averange += nozeros(j-1,i); npoints++; }
                        if( round(nozeros(j-1,i+1)) > 0 ){ averange += nozeros(j-1,i+1); npoints++; }
                        if( round(nozeros(j-2,i-1)) > 0 ){ averange += nozeros(j-2,i-1); npoints++; }
                        if( round(nozeros(j-2,i)) > 0 ){ averange += nozeros(j-2,i); npoints++; }
                        if( round(nozeros(j-2,i+1)) > 0 ){ averange += nozeros(j-2,i+1); npoints++; }
                    }
                    // Everything else
                    else{
                        if( round(nozeros(j-1,i-1)) > 0 ){ averange += nozeros(j-1,i-1); npoints++; }
                        if( round(nozeros(j-1,i)) > 0 ){ averange += nozeros(j-1,i); npoints++; }
                        if( round(nozeros(j-1,i+1)) > 0 ){ averange += nozeros(j-1,i+1); npoints++; }
                        if( round(nozeros(j,i-1)) > 0 ){ averange += nozeros(j,i-1); npoints++; }
                        if( round(nozeros(j,i+1)) > 0 ){ averange += nozeros(j,i+1); npoints++; }
                        if( round(nozeros(j+1,i-1)) > 0 ){ averange += nozeros(j+1,i-1); npoints++; }
                        if( round(nozeros(j+1,i)) > 0 ){ averange += nozeros(j+1,i); npoints++; }
                        if( round(nozeros(j+1,i+1)) > 0 ){ averange += nozeros(j+1,i+1); npoints++; }
                    }

                    if(npoints != 0)
                        nozeros(j,i) = averange / (float) (npoints);
                }
        nozeros.copyTo(image);
    }

public:

    /** Constructor **/
    Heights (std::string filename):
        in_meters(true),
        filename(filename),
        cloud_ (new Cloud)
    {
        loadFiles();
        getCloudDimensions();
        getPointCloudCorrespondences();
        generateCloudImage();
        filterRGBMap();
        filterDepthMap();
        // drawKinectPoints(kinect_points, image);
    }

    /** Setter Measurement unit: Centimeters **/
    void setMeasureUnitCentimeters(){
        in_meters = false;
    }

    /** Setter Measurement unit: Meters **/
    void setMeasureUnitMeters(){
        in_meters = true;
    }

    /** Run function **/
    void run(){
        int keypressed;
        cv::Point point_;

        cv::namedWindow("RGB Map");
        cv::setMouseCallback("RGB Map", Heights::mouseEvent, &point_);
        // cv::imshow("Depth Map", getUcharImage(depth_map));
        // cv::imshow("Depth Map (Filtered)", getUcharImage(depth_map_filtered));
        image.copyTo(image_copy);

        do{
            addLabel(point_);
            cv::imshow("RGB Map", image_copy);
            keypressed = cv::waitKey(100);
        }while( keypressed != 113 && keypressed != 27);

        cv::destroyAllWindows();
    }

    /** Show input image **/

    /** Get uchar image to float image **/
    cv::Mat getUcharImage(const cv::Mat &img){
        float min, max;
        cv::Mat tmp(img.rows, img.cols, CV_8U, cv::Scalar(0));

        min = max = 0;

        for(int j=0; j<img.rows; j++)
            for(int i=0; i<img.cols; i++){
                if( img.at<float>(j,i) < min )
                    min = img.at<float>(j,i);
                if( img.at<float>(j,i) > max )
                    max = img.at<float>(j,i);
            }

        for(int j=0; j<img.rows; j++)
            for(int i=0; i<img.cols; i++)
                tmp.at<uchar>(j,i) = mapping(0, 255, img.at<float>(j,i), min, max);

        return tmp;
    }
};

#endif // HEIGHTS_H
