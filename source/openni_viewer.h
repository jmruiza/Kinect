#ifndef OPENNIVIEWER_H
#define OPENNIVIEWER_H

#include <iostream>
#include <time.h>

#include <pcl/io/pcd_io.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/boost.h>
#include <pcl/visualization/image_viewer.h>
#include <pcl/io/openni_camera/openni_image.h>
#include <pcl/visualization/image_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

class OpenNiViewer{
public:
    // Define new types for existent types in order to make the code more readable
    typedef pcl::PointCloud<pcl::PointXYZ> Cloud;
    typedef Cloud::ConstPtr CloudConstPtr;

    /** Viewers: ImageViewer for RGB images. PCLVisualizer for Point Clouds. **/
    boost::shared_ptr<pcl::visualization::ImageViewer> image_viewer_;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> cloud_viewer_;

    /** Graber and mutex for capture device **/
    pcl::Grabber& grabber_;
    boost::mutex cloud_mutex_;
    boost::mutex image_mutex_;

    /** To get RGB image **/
    cv::Mat img;
    unsigned char* rgb_data_;
    unsigned rgb_data_size_;
    boost::shared_ptr<openni_wrapper::Image> image_;

    /** To get Point Cloud **/
    Cloud c_cloud_;
    CloudConstPtr cloud_;

    /** Visualization Parameters **/
    pcl::visualization::Camera cam;

    /** Constructor
        @param grabber (pcl::Grabber&)
    **/
    OpenNiViewer (pcl::Grabber& grabber):
        image_viewer_ (new pcl::visualization::ImageViewer ("RGB image")),
        cloud_viewer_ (new pcl::visualization::PCLVisualizer ("Point cloud")),
        grabber_ (grabber),
        rgb_data_ (0),
        rgb_data_size_ (0)
    {
        initCamParams();
    }

    /** Initialization of the parameters of visualization **/
    void initCamParams(){
        cam.clip[0] = 1.72229;
        cam.clip[1] = 2.97948;

        cam.focal[0] = 0;
        cam.focal[1] = 0;
        cam.focal[2] = 0;

        cam.pos[0] = 0.0346705;
        cam.pos[1] = -0.0264621;
        cam.pos[2] = -0.908044;

        cam.view[0] = -0.00166531;
        cam.view[1] = -0.999576;
        cam.view[2] = 0.0290659;

        cam.fovy = 0.525500;

        cam.window_size[0] = 640;
        cam.window_size[1] = 480;

        cam.window_pos[0] = 8;
        cam.window_pos[1] = 305;
    }

    /** Callback for cloud, control access to device
        @param cloud (pcl::PointCloud<pcl::PointXYZ>::ConstPtr)
    **/
    void cloud_callback (const CloudConstPtr& cloud){        
        boost::mutex::scoped_lock lock(cloud_mutex_);
        cloud_ = cloud;
    }

    /** Callback for RGB image, control access to device
        @param image
    **/
    void image_callback (const boost::shared_ptr<openni_wrapper::Image>& image){
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

    /** Keyboard callback
        @param event
    **/
    void keyboard_callback (const pcl::visualization::KeyboardEvent& event, void*){
        if ( event.keyDown() ){
            if(event.getKeySym() == "Return"){
                // Make a cloud copy
                // Cloud::Ptr cloud_out;
                // pcl::copyPointCloud<pcl::PointXYZ, pcl::PointXYZ> (*cloud_, *cloud_out);
                // // copyCloud(cloud_);
                // CloudConstPtr c_cloud = cloud_out;

                // Get date and time to generate file name
                std::string path = "../../../data/";
                std::string date = getDate();
                std::stringstream rgb, cloud;
                rgb << path << date << ".jpg";
                cloud << path << date << ".pcd";

                // Save RGB image and cloud
                std::cout << " Saved files: "<< std::endl;
                saveRGBImage(img, rgb.str());
                saveCloud(cloud_, cloud.str());
            }

            if(event.getKeySym() == "Escape"){
                cloud_viewer_->close();
                image_viewer_->close();
            }
        }
    }

    /** Copy a cloud, element by element
        @param in (pcl::PointCloud<pcl::PointXYZ>::ConstPtr)
    **/
    void copyCloud( const CloudConstPtr& in){
        if(c_cloud_.empty())
            for(size_t i=0; i<in->points.size (); ++i)
                c_cloud_.push_back(in->points[i]);
        else
            for(size_t i=0; i<in->points.size(); ++i){
                c_cloud_.points[i].x = in->points[i].x;
                c_cloud_.points[i].y = in->points[i].y;
                c_cloud_.points[i].z = in->points[i].z;
            }
        std::cout << "Cloud copied.." << std::endl;
    }

    /** Get Date and time
        @return string data time
    **/
    std::string getDate(){
        std::string date;
        char buffer [80];
        time_t rawtime;

        struct tm * timeinfo;
        time (&rawtime);
        timeinfo = localtime (&rawtime);

        strftime (buffer,80,"%F_%H%M%S",timeinfo);

        date = buffer;
        return date;
    }

    /** Save RGB image in predeterminated file name
        @param image (cv::Mat)
        @param filename (std::String)
    **/
    void saveRGBImage(cv::Mat image, std::string file_name){
        cv::imwrite(file_name, image);
        std::cout << " -> Saved RGB image: " << file_name << std::endl;
    }

    /** Save Point Cloud file (PCD file) in predeterminated file name
        @param in (pcl::PointCloud<pcl::PointXYZ>::ConstPtr)
        @param filename (std::string)
    **/
    void saveCloud(CloudConstPtr &in, std::string file_name){
        boost::mutex::scoped_lock lock(cloud_mutex_);
        pcl::io::savePCDFile(file_name, *in);
        //pcl::io::savePCDFileASCII(filename, *cloud_);
        std::cout << " -> Saved Point Cloud: " << file_name << std::endl;
    }

    /** Main Loop **/
    void run (){
        // Assigning Callbacks (To PointC)
        boost::function<void (const CloudConstPtr&) > cloud_cb = boost::bind (&OpenNiViewer::cloud_callback, this, _1);
        boost::signals2::connection cloud_connection = grabber_.registerCallback (cloud_cb);

        boost::signals2::connection image_connection;
        if (grabber_.providesCallback<void (const boost::shared_ptr<openni_wrapper::Image>&)>()){
            image_viewer_.reset (new pcl::visualization::ImageViewer ("RGB image"));
            image_viewer_->registerKeyboardCallback(&OpenNiViewer::keyboard_callback, *this);
            boost::function<void (const boost::shared_ptr<openni_wrapper::Image>&) > image_cb = boost::bind (&OpenNiViewer::image_callback, this, _1);
            image_connection = grabber_.registerCallback (image_cb);
        }

        bool image_init = false;
        bool cloud_init = false;

        grabber_.start ();

        while (!cloud_viewer_->wasStopped() && (image_viewer_ && !image_viewer_->wasStopped())){
            // Local RGB_Image and cloud
            boost::shared_ptr<openni_wrapper::Image> image;
            CloudConstPtr cloud;

            cloud_viewer_->spinOnce();

            // See if we can get a cloud
            if (cloud_mutex_.try_lock ()){
                cloud_.swap (cloud);
                cloud_mutex_.unlock ();
            }

            if (cloud){
                if (!cloud_init){
                    // Point Cloud
                    cloud_viewer_->setCameraParameters(cam);
                    cloud_viewer_->setPosition (0, 0);
                    cloud_viewer_->setSize(cloud->width, cloud->height);
                    cloud_init = !cloud_init;
                }

                if (!cloud_viewer_->updatePointCloud(cloud, "Point cloud")){
                    cloud_viewer_->addPointCloud(cloud, "Point cloud");
                    // cloud_viewer_->resetCameraViewpoint("Point cloud");
                }
            }

            // See if we can get an image
            if (image_mutex_.try_lock ()){
                image_.swap (image);
                image_mutex_.unlock();
            }

            if(image){
                if (!image_init && cloud && cloud->width != 0){
                    image_viewer_->setPosition (cloud->width, 0);
                    image_viewer_->setSize (cloud->width, cloud->height);
                    image_init = !image_init;
                }

                if (image->getEncoding() == openni_wrapper::Image::RGB)
                    image_viewer_->addRGBImage(image->getMetaData().Data(), image->getWidth(), image->getHeight());
                else
                    image_viewer_->addRGBImage (rgb_data_, image->getWidth (), image->getHeight ());

                image_viewer_->spinOnce();
            }
        }

        grabber_.stop();
        cloud_connection.disconnect();
        image_connection.disconnect();

        if (rgb_data_)
            delete[] rgb_data_;
    }
};
#endif // OPENNIVIEWER_H

