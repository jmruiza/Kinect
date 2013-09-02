#ifndef KINECTSCANNER_H
#define KINECTSCANNER_H

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/io/openni_camera/openni_image.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/image_viewer.h>

class KinectScanner{

public:
    // Constructor
    KinectScanner();

    // Constructor
    KinectScanner(const std::string &fname);

    pcl::visualization::CloudViewer viewer;
    std::string filename;

    // Routine for all the program
    void run();

    // callback function
    void cloud_cb_ (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud);

    // Routine to read a PCD file
    void readPCDFile();

    // Routine to write a PCD file
    void writePCDFile(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud);
};
#endif // KINECTSCANNER_H
