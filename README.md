<h2> 
<a href="https://github.com/smile99L/plane-analysis"> Crack Detection and Analysis Method for Timber Structures of Ancient Buildings Based on Laser Point Cloud Data</a>
</h2>
 
> **FreeReg:  Crack Detection and Analysis Method for Timber Structures of Ancient Buildings Based on Laser Point Cloud Data**<br/>
> [Jian Ma], [Dechao Liu], [Weidong Yan], [Guoqi Liu], [Xueyan Guo]<br/>

## 🎶 Introduction
<p align="justify">
<strong>Abstract:</strong> Cracks in ancient timber structures often exhibit complex morphologies and uneven surfaces, posing significant challenges for traditional crack detection methods that require non-destructive and high-precision capabilities. This study proposes a non-contact crack detection and analysis method based on terrestrial laser scanning (TLS) point cloud data, integrating Region Growing segmentation, Random Sample Consensus (RANSAC) fitting, and Alpha-shapes contour extraction algorithms. The method overcomes the limitations of previous approaches that focus solely on two-dimensional crack features (length and width) by introducing crack depth computation, enabling precise three-dimensional characterization of crack morphology. Experimental results demonstrate that the proposed method effectively addresses the challenges posed by the irregular surfaces and complex crack forms of ancient timber structures, accurately extracting geometric parameters such as crack length, width, and depth, and exhibiting robust performance and adaptability. This research offers a scalable technical approach for the non-destructive health monitoring of ancient timber structures, providing a reliable foundation for conservation measures and safety assessments.
</p>
## 💻 Requirements
The code has been tested on:  

- visual studio 2019/2022
- pcl 1.12.0

## 🚩 Installation
first,

[Download the PCL address](https://github.com/PointCloudLibrary/pcl/releases):

PCL-1.12.0-AlllnOne-msvc2019-win64.exe

pcl-1.12.0-rc1-pdb-msvc2019-win64.zip

second:

[Environment configuration tutorial](https://blog.csdn.net/qq_36686437/article/details/1190442990)

## 🚩 test

Here's the test code :

#include <iostream>
#include <vector>
#include <ctime>
#include <pcl/point_cloud.h>
#include <pcl/octree/octree.h>
#include <boost/thread/thread.hpp>
#include <pcl/visualization/pcl_visualizer.h>
using namespace std;
int
main(int argc, char** argv)
{
	srand((unsigned int)time(NULL));
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	// Create point cloud data
	cloud->width = 1000;
	cloud->height = 1;
	cloud->points.resize(cloud->width * cloud->height);
	for (size_t i = 0; i < cloud->points.size(); ++i)
	{
		cloud->points[i].x = 1024.0f * rand() / (RAND_MAX + 1.0f);
		cloud->points[i].y = 1024.0f * rand() / (RAND_MAX + 1.0f);
		cloud->points[i].z = 1024.0f * rand() / (RAND_MAX + 1.0f);
	}

	pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree(0.1);
	octree.setInputCloud(cloud);
	octree.addPointsFromInputCloud();
	pcl::PointXYZ searchPoint;
	searchPoint.x = 1024.0f * rand() / (RAND_MAX + 1.0f);
	searchPoint.y = 1024.0f * rand() / (RAND_MAX + 1.0f);
	searchPoint.z = 1024.0f * rand() / (RAND_MAX + 1.0f);

	//Neighborhood search within a radius
	vector<int>pointIdxRadiusSearch;
	vector<float>pointRadiusSquaredDistance;
	float radius = 256.0f * rand() / (RAND_MAX + 1.0f);
	cout << "Neighbors within radius search at (" << searchPoint.x
		<< " " << searchPoint.y
		<< " " << searchPoint.z
		<< ") with radius=" << radius << endl;
	if (octree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
	{
		for (size_t i = 0; i < pointIdxRadiusSearch.size(); ++i)
			cout << "    " << cloud->points[pointIdxRadiusSearch[i]].x
			<< " " << cloud->points[pointIdxRadiusSearch[i]].y
			<< " " << cloud->points[pointIdxRadiusSearch[i]].z
			<< " (squared distance: " << pointRadiusSquaredDistance[i] << ")" << endl;
	}
	// Initialize the point cloud visualization
	boost::shared_ptr<pcl::visualization::PCLVisualizer>viewer(new pcl::visualization::PCLVisualizer("Displays the point cloud"));
	viewer->setBackgroundColor(0, 0, 0);  //Set the background color to black
	// Visualization of point cloud shading (red).
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>target_color(cloud, 255, 0, 0);
	viewer->addPointCloud<pcl::PointXYZ>(cloud, target_color, "target cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "target cloud");

	// Wait until the visualization window closes
	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(1000));
	}

	return (0);
}

## 🔦 Demo

The first step is to run a file called "plane segment" to generate planes and cracks.



The second step is to run the "plane analysis" file on the premise that the file name corresponds.

<img src="utils/media/teaser.png" alt="Network" style="zoom:50%;">


