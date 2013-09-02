#include "kinect_scanner.h"

KinectScanner::KinectScanner():viewer("Test"){}

KinectScanner::KinectScanner(const std::string &fname): viewer("Test"), filename(fname){}

void KinectScanner::run(){
    // create a new grabber for OpenNI devices
    pcl::Grabber* interface = new pcl::OpenNIGrabber();

    // make callback function from member function
    boost::function<void (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr&)> f = boost::bind (&KinectScanner::cloud_cb_, this, _1);

    // connect callback function for desired signal. In this case its a point cloud with color values
    boost::signals2::connection c = interface->registerCallback (f);

    // start receiving point clouds
    interface->start();

    // wait until user quits program with Ctrl-C, but no busy-waiting -> sleep (1);
    while ( !viewer.wasStopped() )
        boost::this_thread::sleep (boost::posix_time::seconds(1));

    // stop the grabber
            interface->stop ();
}

void KinectScanner::cloud_cb_(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud){
    if (!viewer.wasStopped())
        viewer.showCloud (cloud);
    else{
        std::cout << "Live viewer" << std::endl;
        writePCDFile(cloud);
    }
}

void KinectScanner::readPCDFile(){
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>); // Cloud Pointer

    // Reading the cloud
    if (pcl::io::loadPCDFile<pcl::PointXYZ> (filename, *cloud) == -1){
        PCL_ERROR ("Couldn't read file %s \n", filename.c_str() );
        return;
    }

    if( !viewer.wasStopped() ){
        std::cout << "if" << std::endl;

        // Show the cloud in a cloud viewer
        viewer.showCloud(cloud);

        // Stop condition
        while(!viewer.wasStopped()){
            boost::this_thread::sleep(boost::posix_time::microseconds(10));
        }
    }
    else{
        pcl::visualization::CloudViewer viewer2("Cloud");

        // Show the cloud in a cloud viewer
        viewer2.showCloud(cloud);

        // Stop condition
        while(!viewer2.wasStopped()){
            boost::this_thread::sleep(boost::posix_time::microseconds(10));
        }
    }
}

void KinectScanner::writePCDFile(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud){
    // Save the cloud
    pcl::io::savePCDFileASCII(filename, *cloud);
    std::cerr << "Saved: " << filename << std::endl;
}
