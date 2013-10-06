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
    /** Flags **/
    bool files_exists;

    /** Parameters **/
    bool meters;
    bool absolute;
    bool mp_mode;
    bool not_filter;

    /** Strings to handle filenames **/
    std::string filename;
    std::string filenameJPG;
    std::string filenamePCD;

    /** Handling Cloud **/
    CloudPtr cloud_;
    std::vector<cv::Point3f> kinect_points;
    float x_min, x_max;
    float y_min, y_max;
    float z_reference;

    /** Handling Image **/
    cv::Mat image;
    cv::Mat binary;     // Quitar de aqui

    /** Depth Maps **/
    cv::Mat depth_map;
    cv::Mat depth_map_filtered;

    /** Contours and content detection **/
    cv::Mat image_colored;
    std::vector<cv::Scalar> colors;
    std::vector<cv::Vec4i> hierarchy;
    std::vector<std::vector<cv::Point> > contours; 

    /** Vector of points **/
    std::vector<cv::Point> points;

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
        std::cout << " - Loaded: " << filenameJPG << std::endl;

        // Load Point Cloud from pcd file
        pcl::PCDReader reader;
        if( !reader.read<pcl::PointXYZ> (filenamePCD, *cloud_) ){
            files_exists = false;
        }
        std::cout << " - Loaded: " << filenamePCD << std::endl;

        image_colored = cv::Mat::zeros(image.rows, image.cols, CV_8UC3);
        depth_map = cv::Mat(image.rows, image.cols, CV_32F, cv::Scalar(0));
        depth_map_filtered = cv::Mat(image.rows, image.cols, CV_32F, cv::Scalar(0));
    }

    /** Get the Point Cloud Dimensions **/
    void getCloudDimensions(){
        x_min = x_max = cloud_->points[0].x;
        y_min = y_max = cloud_->points[0].y;
        z_reference = cloud_->points[0].z;

        for (size_t i = 1; i < cloud_->points.size (); ++i){
          if( cloud_->points[i].x < x_min )
              x_min = cloud_->points[i].x;

          if( cloud_->points[i].x > x_max )
              x_max = cloud_->points[i].x;

          if( cloud_->points[i].y < y_min )
              y_min = cloud_->points[i].y;

          if( cloud_->points[i].y > y_max )
              y_max = cloud_->points[i].y;

          if( cloud_->points[i].z > z_reference && cloud_->points[i].z > 0 )
              z_reference = cloud_->points[i].z;
        }

        std::cout << " ==================================================== " << std::endl;
        std::cout << " Point Cloud size: " << cloud_->size() << std::endl;
        std::cout << " Size in x (width): " << cloud_->width << std::endl;
        std::cout << " Size in y (height): " << cloud_->height << std::endl;
        std::cout << " x: " << x_min << " - " << x_max << std::endl;
        std::cout << " y: " << y_min << " - " << y_max << std::endl;
        std::cout << " Referencia (z): "<< z_reference << std::endl;
        std::cout << " ==================================================== " << std::endl;
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

    /** Mapping coordinates (x,y) in the image to the coordinates of the cloud
        @param x (int)
        @param y (int)
        @retun x, y, z values (cv::Point3f)
    **/
/*    cv::Point3f getCloudCordinates(int x, int y){
        cv::Point3f mapping_(0, 0, 0);
        // Mapping x value
        mapping_.x = mapping(x_min, x_max, x, 0, image.cols-1);
        // Mapping y value
        mapping_.y = mapping(y_min, y_max, y, 0, image.rows-1);
        return mapping_;
    }
*/

    /** Mapping function image to cloud
        @param yo (float)
        @param yf (float)
        @param x (int)
        @param xo (int)
        @param xf (int)
        @return (float) Mapped value
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
        @return (int) Mapped value
    **/
    int mapping(int yo, int yf, float x, float xo, float xf){
        float m, y;
        m =  ( (yf-yo) / (float)(xf-xo) );
        y = m * (x-xo) + yo;
        return (int)(y);
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
        // Filter image (Eliminate 0's)
        for(int j=0; j<depth_map.rows; j++)
            for(int i=0; i<depth_map.cols; i++)
                depth_map.at<float>(j,i) = z_reference;

        for (size_t i = 0; i < kinect_points.size(); ++i){
            depth_map.at<float>(kinect_points[i].y,kinect_points[i].x) = kinect_points[i].z;
        }
    }

    /** Mouse move event callback
        @param event (int) Event type
        @param x (int) x position
        @param y (int) y position
        @param flags (int)
        @param param (void*)
    **/
    static void mouseMoveEvent(int event, int x, int y, int flags, void* param){
        cv::Point* pnt = (cv::Point*) param;

        if( event == cv::EVENT_MOUSEMOVE ){
            pnt->x = x;
            pnt->y = y;
        }
        return;
    }

    /** Mouse left clic event callback
        @param event (int) Event type
        @param x (int) x position
        @param y (int) y position
        @param flags (int)
        @param param (void*)
    **/
    static void mouseLButtonEvent(int event, int x, int y, int flags, void* param){
        std::vector<cv::Point>* pnt = (std::vector<cv::Point>*) param;
        cv::Point point;

        if( event == cv::EVENT_LBUTTONDOWN ){
            pnt->push_back(cv::Point(x, y));
        }
        return;
    }

    /** Add data of coordinates to image
      @param img (cv::Mat&) labeled image
      @param pnt (cv::Point) is the coordinates to label
     **/
    void addLabel(cv::Mat &img, cv::Point pnt){        
        image.copyTo(img);
        cv::Point ptmp(pnt);
        std::stringstream tmp;

        tmp << std::fixed << std::setprecision(2) << getHeight(pnt);

        if(meters)
             tmp << "m";
        else
            tmp << "cm";

        textPositionAdjust(ptmp.x, ptmp.y, 1);
        cv::circle(img, pnt, 1, cv::Scalar(0, 255, 0), 2);
        cv::putText(img, tmp.str(), ptmp, cv::FONT_HERSHEY_COMPLEX, 0.8, cv::Scalar(0,0,255));
    }

    /** Add point
      @param img (cv::Mat&) image
      @param pnt (std::vector<cv::Point>) vector of coordinates
     **/
    void drawPoints(cv::Mat &img, std::vector<cv::Point> &pnt){
        for(int i=0; i<pnt.size(); i++){
            std::stringstream tmp;
            cv::Point ptmp(pnt[i]);
            tmp << i+1;
            textPositionAdjust(ptmp.x, ptmp.y, 2);
            cv::circle(img, pnt[i], 1, cv::Scalar(0, 255, 0), 2);
            cv::putText(img, tmp.str(), ptmp, cv::FONT_HERSHEY_COMPLEX, 0.8, cv::Scalar(0,0,255));
        }
    }

    /** Adjust the text position in up and left borders
        @param x (int &) x Position
        @parma y (int &) y position
    **/
    void textPositionAdjust(int &x, int &y, int type){
        if(type == 1){
            if(image.cols - x < 99)
                x = image.cols-99;
            if(y < 20)
                y = 20;
        }

        if(type == 2){
            if(image.cols - x < 20)
                x = image.cols-20;
            if(y < 20)
                y = 20;
        }
    }

    /** Print the point values **/
    void getPointsValues(){
        std::cout << "\n -> Points ("<< points.size() <<"):" << std::endl;

        std::string unit;
        if(meters)
            unit = "m";
        else
            unit = "cm";

        for(int i=0; i<points.size(); i++){
            std::stringstream tmp;
            tmp << std::fixed << std::setprecision(2)
                << getHeight(points[i]) << " " << unit;

            std::cout << " " << i+1 << ". (" << points[i].x << ", "
                      << points[i].y << "):  \t" << tmp.str() << std::endl;
        }
    }

    /** Reset point values **/
    void resetPointsValues(){
        points.clear();
        std::cout << "\n -> Reset point list..." << std::endl;
    }

    /** Get Height of a point
        @param pnt (cv::Point)
        @retun height (float)
    **/
    float getHeight(cv::Point pnt){
        float height = 0.0;

        if(not_filter)
            height = depth_map.at<float>(pnt.y, pnt.x);
        else
            height = depth_map_filtered.at<float>(pnt.y, pnt.x);

        if(!absolute)
            height = z_reference - height;

        if(!meters)
            height = height * 100.0f;

        return height;
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
            for(int i=0; i<nozeros.cols; i++){
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
                else
                    nozeros(j,i) = nozeros(j,i)/1;
            }
        nozeros.copyTo(image);
    }

    /** Get uchar image to float image
        @param (const cv::Mat&)
        @return (cv::Mat)
    **/
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

public:

    /** Constructor **/
    Heights (std::string filename):
        meters(true),
        absolute(true),
        mp_mode(false),
        not_filter(false),
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

    /** Setter Measurement unit:
        false: Centimeters
        true: Meters
        @param dim (bool)
    **/
    void distanceInMeters(bool dim){
        meters = dim;
    }

    /** Setter Absolute Measurement
        false: Distance relative
        true: Distance absolute
        @param dim (bool)
    **/
    void distanceAbsolute(bool dabs){
        absolute = dabs;
    }

    /** Setter Demo mode
        false: Disabled
        true: Enabled
        @param sdm (bool)
    **/
    void setMPMode(bool sdm){
        mp_mode = sdm;
    }

    /** Setter Not filter
        false: Filter data
        true: Use data from pcd file
        @param nf (bool)
    **/
    void setNoFilter(bool nf){
        not_filter = nf;
    }

    /** Run function **/
    void run(){
        int keypressed;
        cv::Point point_;
        cv::Mat tmp;
        image.copyTo(tmp);

        cv::namedWindow("RGB Map");
        // cv::imshow("Depth Map", getUcharImage(depth_map));
        // cv::imshow("Depth Map (Filtered)", getUcharImage(depth_map_filtered));

        std::cout << " - Measurement unit: ";
        if( meters )
            std::cout << "Meters" << std::endl;
        else
            std::cout << "Centimeters" << std::endl;

        std::cout << " - Measurement Absolute: ";
        if( absolute )
            std::cout << "on" << std::endl;
        else
            std::cout << "off" << std::endl;

        std::cout << " - Filter data: ";
        if( not_filter )
            std::cout << "no" << std::endl;
        else
            std::cout << "yes" << std::endl;

        if(!mp_mode){
            cv::setMouseCallback("RGB Map", Heights::mouseMoveEvent, &point_);
            std::cout << "\n Controls:"
                      << "\n    Move mouse to get the height or distance of a point"
                      << "\n    ESC or Q    - Finish and close program"
                      << std::endl;
            do{
                addLabel(tmp, point_);
                cv::imshow("RGB Map", tmp);
                keypressed = cv::waitKey(100);
            }while( keypressed != 113 && keypressed != 27);
        }
        else{ // Multiple points mode
            cv::setMouseCallback("RGB Map", Heights::mouseLButtonEvent, &points);
            std::cout << "\n Multiple points controls:"
                      << "\n    r           - Reset captured points"
                      << "\n    ENTER       - Get the heights or distances of the captured points"
                      << "\n    ESC or Q    - Finish and close program"
                      << std::endl;

            do{
                drawPoints(tmp, points);
                cv::imshow("RGB Map", tmp);
                keypressed = cv::waitKey(100);

                // Press R key
                if(keypressed == 114){
                    resetPointsValues();
                    image.copyTo(tmp);
                }

                // Press ENTER Key
                if(keypressed == 13)
                    getPointsValues();                

            }while( keypressed != 113 && keypressed != 27);
        }

        resetPointsValues();
        cv::destroyAllWindows();
    }

};

#endif // HEIGHTS_H
