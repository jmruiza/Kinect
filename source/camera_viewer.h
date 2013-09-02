#ifndef CAMERAVIEWER_H
#define CAMERAVIEWER_H

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/cloud_viewer.h>

class CameraViewer{
public:
    pcl::visualization::ImageViewer viewer;
    CameraViewer():viewer("PCL Camera Image"){}

    void image_cb_ (const boost::shared_ptr<openni_wrapper::Image>& image){
        unsigned char *rgb_buffer;
        rgb_buffer = (unsigned char *) malloc(sizeof (char)*(640*480*4));
        // image->fillRGB (width, height, rgb_buffer, 0);
        image->fillRGB (640, 480, rgb_buffer, 0);
        
        if(!viewer.wasStopped()){
            viewer.showRGBImage (rgb_buffer, image->getWidth(), image->getHeight(), "rgb_image", 1.0);
            std::cout << viewer.wasStopped() << std::endl;
        }
        else{
            std::cout << "Saved Image" << std::endl;
            viewer.close();
        }
        free(rgb_buffer);
     }

    void run(){
        std::cout << " Camera viewer starting ... " <<std::endl;
        pcl::Grabber* interface = new pcl::OpenNIGrabber();  //define a new interface
        boost::function<void (const boost::shared_ptr<openni_wrapper::Image>&)> f = boost::bind (&CameraViewer::image_cb_, this, _1);
        boost::signals2::connection c =  interface->registerCallback (f);
        interface->start ();

        while (!viewer.wasStopped() ){
            // sleep (1);
            boost::this_thread::sleep(boost::posix_time::microseconds(10));
        }

        interface->stop ();
        // c.disconnect();
    }

 };

#endif // CAMERAVIEWER_H
