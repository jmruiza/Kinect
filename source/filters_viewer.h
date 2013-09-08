#ifndef FILTERS_VIEWER_H
#define FILTERS_VIEWER_H

#include <iostream>
#include <string>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

class FiltersViewer{

public:
    // Define new types for existent types: Code easier
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

    // Constructor
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
        tmp2 << filename << "_filtered.pcd";
        st_cloud_out = tmp2.str();
        /*// Print filenames (optional)
        std::cout << "Input: " << st_cloud_in << std::endl;
        std::cout << "Ouput: " << st_cloud_out << std::endl; //*/
    }

    // Init visualizer
    void load_Cloud(){
        // Reading the cloud
        pcl::PCDReader reader;
        reader.read<pcl::PointXYZ> (st_cloud_in, *cloud_in_);
        /*// Print data about cloud (Optional)
        std::cerr << "Cloud before filtering: " << std::endl;
        std::cerr << *cloud_in_ << std::endl; //*/
    }


    // Keyboard callback
    void keyboard_callback (const pcl::visualization::KeyboardEvent& event, void*){
        if ( event.keyDown() ){

            std::cout << event.getKeySym() << std::endl;

            // Press ENTER key: Saves the filtered cloud
            if(event.getKeySym() == "Return"){
                pcl::PCDWriter writer;
                writer.write<pcl::PointXYZ> (st_cloud_out, *cloud_out_);
                std::cout << "Saved: " << st_cloud_out << std::endl;
            }

            // Press ESCAPE key: Close visualizer (End program)
            if(event.getKeySym() == "Escape"){
                viewer_->close();
                // std::cout << event.getKeySym() << " key was pressed, to stop, press Q key" << std::endl;
            }

            /*// Pending: I would like the filter change dynamics has been of a "live"
            // Press S: Statistical Outlier Removal filter
            if(event.getKeySym() == "S" || event.getKeySym() == "s"){
                std::cout << " -> Statistical Outlier Removal filter" << std::endl;
                fil_StatisticalOutlierRemoval();
            }*/
        }
    }

    // Visualization Loop
    void run(){
        viewer_ = viewportsVis(cloud_in_, cloud_out_);
        while (!viewer_->wasStopped ()){
            viewer_->spinOnce ();
            boost::this_thread::sleep (boost::posix_time::microseconds (100));
        }
    }

    // Function for Statistical Outlier Removal filter:
    void filter_StatisticalOutlierRemoval(bool negative=true){
        // The number of neighbors to analyze for each point is set to 50,
        // and the standard deviation multiplier to 1.
        // What this means is that all points who have a distance larger
        // than 1 standard deviation of the mean distance to the query
        // point will be marked as outliers and removed.

        // Create filtering object
        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;

        // Parameters
        sor.setInputCloud (cloud_in_);
        sor.setMeanK (50);
        sor.setStddevMulThresh (1.0);
        sor.setNegative(negative);
        // Output
        sor.filter (*cloud_out_);
    }

    // Function for Radius Outlier Removal filter
    void filter_RadiusOutlierRemoval(){
        // The user specifies a number of neighbors which every indice must
        // have within a specified radius to remain in the PointCloud.

        // Build the Filter
        pcl::RadiusOutlierRemoval<pcl::PointXYZ> ror;
        ror.setInputCloud(cloud_in_);
        ror.setRadiusSearch(0.8);
        ror.setMinNeighborsInRadius (9);

        // Apply filter
        ror.filter (*cloud_out_);
    }

    // Function for Conditional Removal filter
    void filter_ConditionalRemoval(){
        // This filter object removes all points from the PointCloud that do
        // not satisfy one or more conditions.

        // build the condition
        pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_cond (new
        pcl::ConditionAnd<pcl::PointXYZ> ());

        range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new
        pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::GT, 0.0)));

        range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new
        pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::LT, 1.5)));

        // build the filter
        pcl::ConditionalRemoval<pcl::PointXYZ> condrem (range_cond);
        condrem.setInputCloud (cloud_in_);
        condrem.setKeepOrganized(true);

        // Apply filter
        condrem.filter (*cloud_out_);
    }

    // Function for Voxel Grid filter
    void filter_VoxelGrid(){
        // The VoxelGrid class creates a 3D voxel grid (think about a voxel
        // grid as a set of tiny 3D boxes in space) over the input point
        // cloud data. Then, in each voxel (i.e., 3D box), all the points
        // present will be approximated (i.e., downsampled) with their centroid.
        // This approach is a bit slower than approximating them with the center
        // of the voxel, but it represents the underlying surface more accurately.

        // Create the filtering object
        pcl::VoxelGrid<pcl::PointXYZ> vg;
        vg.setInputCloud (cloud_in_);
        vg.setLeafSize (0.01f, 0.01f, 0.01f);
        vg.filter (*cloud_out_);
    }

     // Function for Extract Indices filter
    void filter_ExtractIndices(){
        // ExtractIndices filter extracts a subset of points from a point cloud
        // based on the indices output by a segmentation algorithm.

        // Create a VoxelGrid filter, to downsample the data, in order to speed
        // things up (less points means less time needed to spend within the
        // segmentation loop).
        filter_VoxelGrid();

        Cloud::Ptr cloud_filtered (new Cloud);
        Cloud::Ptr cloud_p (new Cloud);

        cloud_filtered = cloud_out_;

        // Parametric segmentation
        pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
        pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
        // Create the segmentation object
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        // Optional
        seg.setOptimizeCoefficients (true);
        // Mandatory
        seg.setModelType (pcl::SACMODEL_PLANE);
        seg.setMethodType (pcl::SAC_RANSAC);
        seg.setMaxIterations (1000);
        seg.setDistanceThreshold (0.01);

        // Create the filtering object
        pcl::ExtractIndices<pcl::PointXYZ> extract;

        int i = 0, nr_points = (int) cloud_filtered->points.size ();
        // While 30% of the original cloud is still there
        while (cloud_filtered->points.size () > 0.3 * nr_points){
            // Segment the largest planar component from the remaining cloud
            seg.setInputCloud (cloud_filtered);
            seg.segment (*inliers, *coefficients);
            if (inliers->indices.size () == 0){
                std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
                break;
            }

            // Extract the inliers
            extract.setInputCloud (cloud_filtered);
            extract.setIndices (inliers);
            extract.setNegative (false);
            extract.filter (*cloud_p);

            // Create the filtering object
            extract.setNegative (true);
            extract.filter (*cloud_out_);
            cloud_filtered.swap (cloud_out_);
            i++;
        }
    }


private:    

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewportsVis
    ( pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out){

        boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("Point cloud"));
        viewer->initCameraParameters ();
        viewer->registerKeyboardCallback(&FiltersViewer::keyboard_callback, *this);

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
        viewer->resetCameraViewpoint("Point cloud");
        // viewer->addCoordinateSystem (1.0);

        return (viewer);
    }

};

#endif // FILTERS_VIEWER_H
