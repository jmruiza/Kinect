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

    // Viewers: ImageViewer for RGB images. PCLVisualizer for Point Clouds.
    boost::shared_ptr<pcl::visualization::ImageViewer> image_viewer_;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> cloud_viewer_;

    // Graber and mutex for capture device
    pcl::Grabber& grabber_;
    boost::mutex cloud_mutex_;
    boost::mutex image_mutex_;

    // To get RGB image
    cv::Mat img;
    unsigned char* rgb_data_;
    unsigned rgb_data_size_;
    boost::shared_ptr<openni_wrapper::Image> image_;

    // To get Point Cloud
    CloudConstPtr cloud_;

    // Constructor
    OpenNiViewer (pcl::Grabber& grabber):
        image_viewer_ (new pcl::visualization::ImageViewer ("RGB image")),
        cloud_viewer_ (new pcl::visualization::PCLVisualizer ("Point cloud")),
        grabber_ (grabber),
        rgb_data_ (0),
        rgb_data_size_ (0)
    {}

    // Callback for cloud
    void cloud_callback (const CloudConstPtr& cloud){
        // To control access to the device
        boost::mutex::scoped_lock lock(cloud_mutex_);
        cloud_ = cloud;
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
                // Copy the PointCloud in new one
                // Cloud::Ptr test(cloud_);
                //Cloud::Ptr copy_ptr (new Cloud);
                CloudConstPtr copy_cptr = cloud_;

                // Get date and time to generate file name
                std::string path = "../../../data/";
                std::string date = getDate();
                std::stringstream rgb, cloud;
                rgb << path << date << ".jpg";
                cloud << path << date << ".pcd";

                // Save RGB image and cloud
                std::cout << " Saved files: "<< std::endl;
                saveRGBImage(img, rgb.str());
                //saveCloud(cloud_, cloud.str());
                saveCloud(copy_cptr, cloud.str());
                copy_cptr.reset();

            }
            if(event.getKeySym() == "Escape"){
                std::cout << event.getKeySym() << " key was pressed, to stop, press Q key" << std::endl;
            }
        }
    }

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

    void saveRGBImage(cv::Mat image, std::string file_name){
        cv::imwrite(file_name, image);
        std::cout << " -> Saved RGB image: " << file_name << std::endl;
    }

    void saveCloud(CloudConstPtr &in, std::string file_name){
        pcl::io::savePCDFile(file_name, *in);
        //pcl::io::savePCDFileASCII(filename, *cloud_);
        std::cout << " -> Saved Point Cloud: " << file_name << std::endl;
    }

    // Main Loop
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
                    cloud_viewer_->setPosition (0, 0);
                    cloud_viewer_->setSize(cloud->width, cloud->height);
                    cloud_init = !cloud_init;
                }

                if (!cloud_viewer_->updatePointCloud(cloud, "Point cloud")){
                    cloud_viewer_->addPointCloud(cloud, "Point cloud");
                    cloud_viewer_->resetCameraViewpoint("Point cloud");
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



//    void cloud_cb_ (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud){
//        if (!viewer.wasStopped())
//            viewer.showCloud (cloud);
//        else{
//            // Save the cloud
//            pcl::io::savePCDFileASCII("kinect.pcd", *cloud);
//            // pcl::io::savePCDFile("kinect.pcd", *cloud);
//            std::cerr << "Saved " << std::endl;
//            std::cout << "End viewer" << std::endl;
//        }
//    }

//    void run (){
//        // create a new grabber for OpenNI devices
//        pcl::Grabber* interface = new pcl::OpenNIGrabber();

//        // make callback function from member function
//        boost::function<void (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr&)> f =
//        boost::bind (&OpenNiViewer::cloud_cb_, this, _1);

//        // connect callback function for desired signal. In this case its a point cloud with color values
//        boost::signals2::connection c = interface->registerCallback (f);

//        // start receiving point clouds
//        interface->start();

//        // wait until user quits program with Ctrl-C, but no busy-waiting -> sleep (1);
//        while ( !viewer.wasStopped() )
//            boost::this_thread::sleep (boost::posix_time::seconds (1));

//        // stop the grabber
//        interface->stop ();
//    }
};
#endif // OPENNIVIEWER_H

