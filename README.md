# SFND - Lidar Detection Project

## Build Instruction

Please refer to [Install.md](./Install.md)

## Codes Explaination
The main cpp file to run this project is `environment.cpp`.

In this project, I implemented Plane Segmentation using RANSAC and Euclidean Clustering using PCL library and from scratch (learning material credits go to Udacity Sensor Fusion Nanodegree program).

After the project is built, by default, the `build/environment` file will process the streaming data from `sensors/data/pcd/data_1` with `cityBlockProject()` function. Inside `cityBlockProject()` function, the Plane Segmentation(RANSAC3D) and Euclidean Clustering methods are programmed without using PCL library, and this is the requirement for the project. The self programmed `RANSAC3D` and `euclideanCluster` are in `ProcessPointClouds` class in `processPointClouds.h`. 

#### Explain some main functions in `environment.cpp`:

- `simpleHighway()` implements PCL library for Segmentation and Euclidean Clustering to point cloud in simulation for learning purposes.

- `cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer)` implements PCL library for Segmentation and Euclidean Clustering to actual point clouds for one sample data in `sensors/data/pcd/data_1`.

- `cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer)` implements PCL library for Segmentation and Euclidean Clustering to actual point clouds for one sample data in `sensors/data/pcd/data_1`.

- `cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>* pointProcessorI, const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud)` implements PCL library for Segmentation and Euclidean Clustering to actual point clouds to stream the entire sample data in `sensors/data/pcd/data_1`.

- `cityBlockProject(pcl::visualization::PCLVisualizer::Ptr& viewer)` implements from scratch Segmentation and Euclidean Clustering to actual point clouds for one sample data in `sensors/data/pcd/data_1`.

- `cityBlockProject(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>* pointProcessorI, const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud)` implements from scratch Segmentation and Euclidean Clustering to actual point clouds to stream the entire sample data in `sensors/data/pcd/data_1`.

#### Some other way to run all function above
- If youw ant to run any static data without stream, set `streaming` to `false` on line 270 and follow terminal instruction when launch `environment` for your choice of available options.

- If you want to run the streaming data with PCL library using `cityBlock()`, set `streaming` to `true` on line 270 and set `using_pcl` to `true` on line 271 and build.

- If you want to run the streaming data using `cityBlockProject()`, set `streaming` to `true` on line 270 and set `using_pcl` to `false` on line 271 and build.


