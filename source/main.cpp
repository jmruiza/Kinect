#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/console/parse.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "openni_viewer.h"

void printUsage (const char* progName){
    std::cout << "\nUsage: "<<progName<<" [options]\n"
              << "-------------------------------------------\n"
              << "-h \t Help mode: Show this help\n"
              << "-c \t Capture mode: Capture data from kinect device\n"
              << "-l \t Live mode: Get measurements of live data from Kinect\n"
              << "-f \t File mode: Get measurements of captured Point Cloud"
              << "\n";
    std::cout << std::endl;
}

//// Debbug
//int main(){
//    // Print mode information
//    std::cout << "\n - Capture Mode: \n"
//              << "-------------------------------------------\n"
//              << " Press Enter to save Point Cloud and RGB image"
//              << " Press Q or close the any Viewer to finish \"Capture Mode\"."
//              << std::endl;

//    std::string device_id("");
//    pcl::OpenNIGrabber::Mode depth_mode = pcl::OpenNIGrabber::OpenNI_Default_Mode;
//    pcl::OpenNIGrabber::Mode image_mode = pcl::OpenNIGrabber::OpenNI_Default_Mode;

//    // Create a new OpenNiGrabber
//    pcl::OpenNIGrabber grabber(device_id, depth_mode, image_mode);

//    //OpenNIViewer<pcl::PointXYZRGBA> openni_viewer(grabber);
//    OpenNIViewer<pcl::PointXYZ> openni_viewer(grabber);
//    openni_viewer.run();
//    return 0;
//}


int main (int argc, char** argv){

//    // Parse Command Line Arguments in order to determine mode.

//    // Help Mode (Print usage)
//    if (pcl::console::find_argument(argc, argv, "-h") >= 0){
//        printUsage (argv[0]);
//        return 0;
//    }

//    // Capture mode
//    if (pcl::console::find_argument(argc, argv, "-c") >= 0){
//        // Print mode information
//        std::cout << "\n - Capture Mode: \n"
//                  << "-------------------------------------------\n"
//                  << " Press Enter to save Point Cloud and RGB image"
//                  << " Press Q or close the any Viewer to finish \"Capture Mode\"."
//                  << std::endl;

//        std::string device_id("");
//        pcl::OpenNIGrabber::Mode depth_mode = pcl::OpenNIGrabber::OpenNI_Default_Mode;
//        pcl::OpenNIGrabber::Mode image_mode = pcl::OpenNIGrabber::OpenNI_Default_Mode;

//        // Create a new OpenNiGrabber
//        pcl::OpenNIGrabber grabber(device_id, depth_mode, image_mode);

//        //OpenNIViewer<pcl::PointXYZRGBA> openni_viewer(grabber);
//        OpenNIViewer<pcl::PointXYZ> openni_viewer(grabber);
//        openni_viewer.run();
//        return 0;
//    }

//    // Live mode
//    if (pcl::console::find_argument(argc, argv, "-l") >= 0){
//        // Print mode information
//        std::cout << "\n - Live Mode: \n"
//                  << "-------------------------------------------\n"
//                  << " Press Enter to save Point Cloud and RGB image"
//                  << " Press Q or close the any Viewer to finish \"Live Mode\"."
//                  << std::endl;

//        std::string device_id("");
//        pcl::OpenNIGrabber::Mode depth_mode = pcl::OpenNIGrabber::OpenNI_Default_Mode;
//        pcl::OpenNIGrabber::Mode image_mode = pcl::OpenNIGrabber::OpenNI_Default_Mode;

//        // Create a new OpenNiGrabber
//        pcl::OpenNIGrabber grabber(device_id, depth_mode, image_mode);

//        //OpenNIViewer<pcl::PointXYZRGBA> openni_viewer(grabber);
//        OpenNIViewer<pcl::PointXYZ> openni_viewer(grabber, 1);
//        openni_viewer.run();
//        return 0;
//    }

//    // File mode
//    if (pcl::console::find_argument(argc, argv, "-m") >= 0){
//        // Print mode information
//        std::cout << "\n - File mode: \n"
//                  << "-------------------------------------------\n"
//                  << " Press Q or close the any Viewer to finish \"Capture Mode\"."
//                  << std::endl;
//        return 0;
//    }

    // If there wasn't any argument on the command line
    printUsage (argv[0]);
    return 0;
}


