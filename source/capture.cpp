#include <iostream>
#include "openni_viewer.h"

using namespace std;

int main (int argc, char** argv){

    cout << "\n Usage: "<< argv[0] <<"\n"
         << "---------------------------------------------------------\n"
         << " Press:\n"
         << "  - ENTER    - to save a Point Cloud and RGB image\n"
         << "  - Q        - to close the viewer and exit the program\n"
         << "---------------------------------------------------------\n"
         << endl;

    string device_id("");
    pcl::OpenNIGrabber::Mode depth_mode = pcl::OpenNIGrabber::OpenNI_Default_Mode;
    pcl::OpenNIGrabber::Mode image_mode = pcl::OpenNIGrabber::OpenNI_Default_Mode;

    // Create a new OpenNiGrabber
    pcl::OpenNIGrabber grabber(device_id, depth_mode, image_mode);

    //OpenNIViewer<pcl::PointXYZRGBA> openni_viewer(grabber);
    OpenNiViewer openni_viewer(grabber);
    openni_viewer.run();
    return 0;
}

