#ifndef TEST_H
#define TEST_H

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/io/openni_camera/openni_image.h>
#include <pcl/visualization/boost.h>
#include <pcl/visualization/image_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "watershed_segmenter.h"

/*
#include <pcl/common/time.h> //fps calculations
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
*/

template <typename PointType>

class OpenNIViewer{

public:

    typedef pcl::PointCloud<PointType> Cloud;
    typedef typename Cloud::ConstPtr CloudConstPtr;

    boost::shared_ptr<pcl::visualization::ImageViewer> image_viewer_;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> cloud_viewer_;
    // boost::shared_ptr<pcl::visualization::PCLVisualizer> filtered_cloud_viewer_;

    pcl::Grabber& grabber_;
    boost::mutex cloud_mutex_;
    boost::mutex image_mutex_;

    cv::Mat img;
    CloudConstPtr cloud_;
    CloudConstPtr filtered_cloud_;
    boost::shared_ptr<openni_wrapper::Image> image_;

    WatershedSegmenter segmenter;

    unsigned char* rgb_data_;
    unsigned rgb_data_size_;

    // Constructor - Capture mode
    OpenNIViewer (pcl::Grabber& grabber):
        image_viewer_ (new pcl::visualization::ImageViewer ("RGB image")),
        cloud_viewer_ (new pcl::visualization::PCLVisualizer ("Point cloud")),
        // filtered_cloud_viewer_ (new pcl::visualization::PCLVisualizer ("Filtered Point cloud")),
        grabber_ (grabber),
        rgb_data_ (0),
        rgb_data_size_ (0)
    {}

    // Constructor - Measurement Mode
    OpenNIViewer ():
        image_viewer_ (new pcl::visualization::ImageViewer ("RGB image")),
        cloud_viewer_ (new pcl::visualization::PCLVisualizer ("Point cloud")),
        // filtered_cloud_viewer_ (new pcl::visualization::PCLVisualizer ("Filtered Point cloud")),
        rgb_data_ (0),
        rgb_data_size_ (0)
    {}

    // Callback for cloud
    void cloud_callback (const CloudConstPtr& cloud){
        // To control access to the device
        boost::mutex::scoped_lock lock(cloud_mutex_);
        cloud_ = cloud;
        filterCloud();
    }

    // Callback for RGB image
    void image_callback (const boost::shared_ptr<openni_wrapper::Image>& image){
        // To control access to the device
        boost::mutex::scoped_lock lock(image_mutex_);
        image_ = image;

        if (image->getEncoding () != openni_wrapper::Image::RGB){
            if (rgb_data_size_ < image->getWidth () * image->getHeight ()){
                if (rgb_data_)
                    delete [] rgb_data_;
                rgb_data_size_ = image->getWidth () * image->getHeight ();
                rgb_data_ = new unsigned char [rgb_data_size_ * 3];
            }

            image_->fillRGB (image_->getWidth (), image_->getHeight (), rgb_data_);

            // Convert image to cv::Mat
            cv::Mat tmp = cv::Mat( cv::Size(image_->getWidth(), image_->getHeight()), CV_8UC3, rgb_data_);
            cvtColor(tmp, img, CV_RGB2BGR);
        }
    }

    // Keyboard callback
    void keyboard_callback (const pcl::visualization::KeyboardEvent& event, void*){
        if ( event.keyDown() ){
            if(event.getKeySym() == "Return"){                
                // Save RGB image
                saveRGBImage();
                // Save cloud
                saveCloud();
                std::cout << "Saved RGB Data and Point Cloud." << std::endl;
            }
            if(event.getKeySym() == "Escape"){
                std::cout << event.getKeySym() << " key was pressed, to stop, press Q key" << std::endl;
            }
        }
    }

    // Mouse Callback
    void mouse_callback (const pcl::visualization::MouseEvent& mouse_event, void*){
        // If want show information in the viewer uncomment the commented lines and change the parameters of function
        // boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *> (viewer_void);

        if (mouse_event.getType() == pcl::visualization::MouseEvent::MouseButtonPress && mouse_event.getButton() == pcl::visualization::MouseEvent::LeftButton){
            cout << " - " << mouse_event.getX() << ", " << mouse_event.getY();
            double zvalue = getValueForZ( mouse_event.getX(), mouse_event.getY() );
            // char str[512];
            // sprintf (str, "text");
            // viewer->removeShape(str);
            // viewer->addText("Clicked here.. ", event.getX(), event.getY(), 200.0, 150.0, 254.0, str );
        }
    }

    double getValueForZ(int x, int y){
        double zvalue = 0.0;
        // const pcl::PointXYZRGBA& pnt = cloud->at(x,y);
        const pcl::PointXYZ &pnt = cloud_->at(x,y);
        zvalue = pnt.z;
        cout << ", " << zvalue << endl;
        return zvalue;
    }

    void saveRGBImage(){
        cv::imwrite("RGB_Data.jpg", img);
    }

    void saveCloud(){
        std::string filename = "Point_Data";
        pcl::io::savePCDFileASCII(filename, *cloud_);
        std::cerr << "Saved: " << filename << std::endl;
    }

    void openCloud(const char* filename){
        std::cout << filename << std::endl;
    }

    void openRGBImage(const char* filename){
        std::cout << filename << std::endl;
    }

    void printSomething(){
        std::cout << "Hello world!!" << std::endl;
    }

//    void openCloud(const char* filename){
//         pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

//        // Reading the cloud
//        if (pcl::io::loadPCDFile<pcl::PointXYZ> (filename, *cloud) == -1){
//            PCL_ERROR ("Couldn't read file %s \n", filename );
//            return;
//        }
//    }

//    void openRGBImage(const char* filename){
//        cv::Mat img = cv::imread(filename);
//        // Convert cv::Mat to unsigned char*
//        cv::Mat tmp;
//        cv::cvtColor(img, tmp, CV_BGR2RGB);
//        rgb_data_ = tmp.data;
//    }


    void filterCloud(){
        filtered_cloud_ = cloud_;
        
        
//        for(int i=0; i<cloud_->height; i++){ // iterate rows
//            for(int j=0; j<cloud_->width; j++){ // iterate columns
//                const pcl::PointXYZ &pnt = cloud_->at(j,i);
//                const pcl::PointXYZ &f_pnt = filtered_cloud_->at(j,i);

//                cloud_->at(j,i) = filtered_cloud_->at(j,i);;
//            }
//        }
        
        
//        // Empaquetado del color RGB
//        uint8_t r, g, b;
//        uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
    
//        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>(rows,cols));
//        for(int i=0; i<cols; i++)
//            for(int j=0; j<rows; j++){
//                pcl::PointXYZRGB& point = cloud->at(i,j);
//                point.x = i/50.0;
//                point.y = j/50.0;
//                point.z = a*((i-cols/2)*(i-cols/2) + (j-rows/2)*(j-rows/2))/50.0;
//                point.rgb = rgb;
//            }
}

    // Main Loop
    void run (){
        // cloud_viewer_->registerMouseCallback (&OpenNIViewer::mouse_callback, *this);
        // cloud_viewer_->registerKeyboardCallback(&OpenNIViewer::keyboard_callback, *this);
        boost::function<void (const CloudConstPtr&) > cloud_cb = boost::bind (&OpenNIViewer::cloud_callback, this, _1);
        boost::signals2::connection cloud_connection = grabber_.registerCallback (cloud_cb);

        boost::signals2::connection image_connection;
        if (grabber_.providesCallback<void (const boost::shared_ptr<openni_wrapper::Image>&)>()){
            image_viewer_.reset (new pcl::visualization::ImageViewer ("RGB image"));
            image_viewer_->registerMouseCallback (&OpenNIViewer::mouse_callback, *this);
            image_viewer_->registerKeyboardCallback(&OpenNIViewer::keyboard_callback, *this);
            boost::function<void (const boost::shared_ptr<openni_wrapper::Image>&) > image_cb = boost::bind (&OpenNIViewer::image_callback, this, _1);
            image_connection = grabber_.registerCallback (image_cb);
        }

        bool image_init = false;
        bool cloud_init = false;

        grabber_.start ();

        while (!cloud_viewer_->wasStopped() && (image_viewer_ && !image_viewer_->wasStopped ())){

            // Local RGB_Image and cloud
            boost::shared_ptr<openni_wrapper::Image> image;
            CloudConstPtr cloud;

            cloud_viewer_->spinOnce ();

            // See if we can get a cloud
            if (cloud_mutex_.try_lock ()){
                cloud_.swap (cloud);
                cloud_mutex_.unlock ();
            }

            if (cloud){
                if (!cloud_init){
                    // Point Cloud
                    cloud_viewer_->setPosition (0, 0);
                    cloud_viewer_->setSize (cloud->width, cloud->height);                    
                    // Filtered Point Cloud
                    // filtered_cloud_viewer_->setPosition(0, cloud->height);
                    // filtered_cloud_viewer_->setSize(cloud->width, cloud->height);

                    cloud_init = !cloud_init;
                }

                if (!cloud_viewer_->updatePointCloud(cloud, "Point cloud")){
                    // Point Cloud
                    // cloud_viewer_->addPointCloud(cloud, "Point cloud");
                    cloud_viewer_->addPointCloud(filtered_cloud_, "Point cloud");
                    cloud_viewer_->resetCameraViewpoint ("Point cloud");
                    // Filtered Point Cloud
                    // filtered_cloud_viewer_->addPointCloud(cloud, "Filtered Point cloud");
                    // filtered_cloud_viewer_->resetCameraViewpoint ("Filtered Point cloud");
                }
            }

            // See if we can get an image
            if (image_mutex_.try_lock ()){
                image_.swap (image);
                image_mutex_.unlock();
            }

            if (image){
                if (!image_init && cloud && cloud->width != 0){
                    image_viewer_->setPosition (cloud->width, 0);
                    image_viewer_->setSize (cloud->width, cloud->height);
                    image_init = !image_init;
            }

            cv::Mat tmp;
            // Sement cv::Mat image
            segmenter.process(img);
            // Conver image to char
            cvtColor(segmenter.getSegmentedImage(), tmp, CV_BGR2RGB);
            //image->fillRGB(image_->getWidth(), image_->getHeight(), tmp.data);

            if (image->getEncoding() == openni_wrapper::Image::RGB)
                // image_viewer_->addRGBImage(image->getMetaData().Data(), image->getWidth(), image->getHeight());
                image_viewer_->addRGBImage(tmp.data, image->getWidth(), image->getHeight());
            else
                // image_viewer_->addRGBImage (rgb_data_, image->getWidth (), image->getHeight ());
                image_viewer_->addRGBImage (tmp.data, image->getWidth(), image->getHeight());
                image_viewer_->spinOnce();
            }

        }

        grabber_.stop ();

        cloud_connection.disconnect ();
        image_connection.disconnect ();

        if (rgb_data_)
            delete[] rgb_data_;
    }

    void measurementMode(){
        
    }
    
};


#endif // TEST_H
