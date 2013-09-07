#include <iostream>
#include "filters_viewer.h"
//#include <string>
//#include <pcl/io/pcd_io.h>
//#include <pcl/point_types.h>
//#include <pcl/visualization/pcl_visualizer.h>
//#include <pcl/filters/statistical_outlier_removal.h>

//boost::shared_ptr<pcl::visualization::PCLVisualizer>
//viewportsVis ( pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in,
//               pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out){

//    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
//    viewer->initCameraParameters ();

//    int v1(0);
//    viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
//    viewer->setBackgroundColor (0, 0, 0, v1);
//    viewer->addText("Before..", 10, 10, "v1 text", v1);
//    viewer->addPointCloud<pcl::PointXYZ> (cloud_in, "sample cloud1", v1);

//    int v2(0);
//    viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
//    viewer->setBackgroundColor (0.3, 0.3, 0.3, v2);
//    viewer->addText("After..", 10, 10, "v2 text", v2);
//    viewer->addPointCloud<pcl::PointXYZ> (cloud_out, "sample cloud2", v2);

//    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud1");
//    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud2");
//    viewer->addCoordinateSystem (1.0);

//    return (viewer);
//}

void printUsage(char* pname){
    std::cout << "\n Usage: "<< pname << "  point_cloud_file.pcd" <<"\n"
              << "---------------------------------------------------------\n"
              << " Press:\n"
              << "  - ENTER    - to save the filtered Point Cloud\n"
              << "  - Q        - to close the viewer and exit the program\n"
              << "  - 1 - 9    - to apply a filter  \n"
              << "---------------------------------------------------------\n"
              << std::endl;
}

int main (int argc, char** argv){
    // Check number of parameters
    if(argc < 2 || argc > 2){
        std::cout << " Error: This program needs a parameter to work " << std::endl;
        printUsage(argv[0]);
        return 1;
    }

    // Check the filename:
    std::string parameter = argv[1];
    size_t position_ext = parameter.find(".pcd");
    std::string extension;
    std::string filename;

    // Check that filename has a point
    if( position_ext == parameter.npos ){
        std::cout << " Error: The given parameter \"" << parameter << "\" must have a valid file name!" << std::endl;
        printUsage(argv[0]);
        return 1;
    }

    extension = parameter.substr(position_ext+1, parameter.length());
    filename = parameter.substr(0, position_ext);

    // Check that file name has a "*.pcd" extension
    if( extension.compare("pcd") ){
        std::cout << " Error: The given parameter \"" << parameter << "\" isn't a \"*.pcd\" file!" << std::endl;
        printUsage(argv[0]);
        return 1;
    }

//    std::cout << "Input Parameter: " << parameter << std::endl;
//    std::cout << "Filename: " << filename << std::endl;
//    std::cout << "Extension: " << extension << std::endl;

    FiltersViewer fv;
    fv.set_FileNames(filename);
    fv.run();
/*


    // Functions for filters
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

    // Read the input cloud data
    pcl::PCDReader reader;
    reader.read<pcl::PointXYZ> (parameter, *cloud_in);

    // Here include all the availabel filters - Check documentation


    // Create the filtering object
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud (cloud);
    sor.setMeanK (50);
    sor.setStddevMulThresh (1.0);
    sor.filter (*cloud_filtered);

    pcl::PCDWriter writer;
    writer.write<pcl::PointXYZ> ("table_scene_lms400_inliers.pcd", *cloud_filtered, false);

    sor.setNegative (true);
    sor.filter (*cloud_filtered);
    writer.write<pcl::PointXYZ> ("table_scene_lms400_outliers.pcd", *cloud_filtered, false);


    // Added by me
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    viewer = viewportsVis(cloud, cloud_filtered);

    while (!viewer->wasStopped ()){
        viewer->spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }
*/
    return (0);
}
