#ifndef FILTERS_VIEWER_H
#define FILTERS_VIEWER_H

#include <iostream>
#include <string>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/statistical_outlier_removal.h>

class FiltersViewer{

public:
    // Define new types for existent types in order to make the code more readable
    typedef pcl::PointCloud<pcl::PointXYZ> Cloud;
    typedef typename Cloud::Ptr CloudPtr;

    // Filenames
    std::string st_cloud_in;
    std::string st_cloud_out;

    // Pointers for IO point clouds
    CloudPtr cloud_in_;
    CloudPtr cloud_out_;

    // Viewer for visualization
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_;

    FiltersViewer ():
        viewer_ (new pcl::visualization::PCLVisualizer ("Point cloud")),
        cloud_in_ (new Cloud),
        cloud_out_ (new Cloud)
    {}

    // Set filenames
    void set_FileNames(std::string filename){
        std::stringstream tmp1, tmp2;
        tmp1 << filename << ".pcd";
        st_cloud_in = tmp1.str();
        tmp2 << filename << "_out.pcd";
        st_cloud_out = tmp2.str();
        /*// Print filenames (optional)
        std::cout << "Input: " << st_cloud_in << std::endl;
        std::cout << "Ouput: " << st_cloud_out << std::endl; //*/
    }

    // Init visualizer
    void init_Visualizer(){
        // Reading the cloud
        pcl::PCDReader reader;
        reader.read<pcl::PointXYZ> (st_cloud_in, *cloud_in_);
        cloud_out_ = cloud_in_;
        /*// Print data about cloud (Optional)
        std::cerr << "Cloud before filtering: " << std::endl;
        std::cerr << *cloud_in_ << std::endl; //*/
        // Load viewer
        viewer_ = viewportsVis(cloud_in_, cloud_out_);
    }


    // Keyboard callback
    void keyboard_callback (const pcl::visualization::KeyboardEvent& event, void*){
        if ( event.keyDown() ){
            // Press ENTER key: Saves the filtered cloud
            if(event.getKeySym() == "Return"){
                pcl::PCDWriter writer;
                writer.write<pcl::PointXYZ> (st_cloud_out, *cloud_out_);
            }
            // Press ESCAPE key: Do nothing
            if(event.getKeySym() == "Escape"){
                std::cout << event.getKeySym() << " key was pressed, to stop, press Q key" << std::endl;
            }
        }
    }

    // Visualization Loop
    void run(){
        init_Visualizer();
        while (!viewer_->wasStopped ()){
            viewer_->spinOnce (100);
            boost::this_thread::sleep (boost::posix_time::microseconds (100000));
        }
    }

//    // Function for Statistical Outlier Removal filter:
//    void fil_StatisticalOutlierRemoval(bool negative=false){
//        // Create filtering object
//        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
//        // Parameters
//        sor.setInputCloud (cloud_in_);
//        sor.setMeanK (50);
//        sor.setStddevMulThresh (1.0);
//        sor.setNegative(negative);
//        // Output
//        sor.filter (*cloud_out_);

//        // Update visualizer
//        // viewer = viewportsVis(cloud_in_, cloud_out_);
//    }

private:    

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewportsVis
    ( pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out){

        boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
        viewer->initCameraParameters ();

        int v1(0);
        viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
        // viewer->setBackgroundColor (0, 0, 0, v1);
        viewer->addText("Without Filter...", 10, 10, "v1 text", v1);
        viewer->addPointCloud<pcl::PointXYZ> (cloud_in, "sample cloud1", v1);

        int v2(0);
        viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
        // viewer->setBackgroundColor (0.3, 0.3, 0.3, v2);
        viewer->addText("With Filter..", 10, 10, "v2 text", v2);
        viewer->addPointCloud<pcl::PointXYZ> (cloud_out, "sample cloud2", v2);

        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud1");
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud2");
        // viewer->addCoordinateSystem (1.0);

        return (viewer);
    }

};

#endif // FILTERS_VIEWER_H
