#include <vector>
#include <thread>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
//#include <pcl/visualization/cloud_viewer.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
//#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/time.h>
#include <iostream>
#include <sstream>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>

using namespace std::chrono_literals;

int main(int argc, char** argv)
{
	
	// Read in the cloud data
	pcl::PCDReader reader;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>), cloud_f(new pcl::PointCloud<pcl::PointXYZ>);
	reader.read("D:/Hp/datas/newsahne_pcd.pcd", *cloud);
	std::cout << "PointCloud before filtering has: " << cloud->points.size() << " data points." << std::endl; //*

																											  // Create the filtering object: downsample the dataset using a leaf size of 50m
	pcl::VoxelGrid<pcl::PointXYZ> vg;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
	vg.setInputCloud(cloud);
	vg.setLeafSize(10.0f, 10.0f, 10.0f);
	vg.filter(*cloud_filtered);
	std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size() << " data points." << std::endl; //*

																													  // Create the segmentation object for the planar model and set all the parameters
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PCDWriter writer;
	seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setMaxIterations(1000);
	seg.setDistanceThreshold(50.0);
	int max_point = 0;
	int i = 0, nr_points = (int)cloud_filtered->points.size();

	while (cloud_filtered->points.size() > 0.3 * nr_points)
	{
		// Segment the largest planar component from the remaining cloud
		seg.setInputCloud(cloud_filtered);
		seg.segment(*inliers, *coefficients);
		if (inliers->indices.size() == 0)
		{
			std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
			break;
		}

		// Extract the planar inliers from the input cloud
		pcl::ExtractIndices<pcl::PointXYZ> extract;
		extract.setInputCloud(cloud_filtered);
		extract.setIndices(inliers);
		extract.setNegative(false);

		// Get the points associated with the planar surface
		extract.filter(*cloud_plane);
		std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size() << " data points." << std::endl;

		// Remove the planar inliers, extract the rest
		extract.setNegative(true);
		extract.filter(*cloud_f);
		*cloud_filtered = *cloud_f;
	}
	
	//writer.write("C:/Users/Hp/Desktop/groundplane50.pcd", *cloud_plane, false);

	//					 C L U S T E R I N G

	// Creating the KdTree object for the search method of the extraction
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(cloud_filtered);

	std::vector<pcl::PointIndices> cluster_indices;


	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
	ec.setClusterTolerance(25.0f); // 2cm
	ec.setMinClusterSize(100);
	ec.setMaxClusterSize(25000);
	ec.setSearchMethod(tree);
	ec.setInputCloud(cloud_filtered);
	ec.extract(cluster_indices);

	//std::cout << "1st Cluster: " << cluster_indices[0] << std::endl;


	int j = 0;
	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
		for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
			cloud_cluster->points.push_back(cloud_filtered->points[*pit]); //*
		cloud_cluster->width = cloud_cluster->points.size();
		cloud_cluster->height = 1;
		cloud_cluster->is_dense = true;

		std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size() << " data points." << std::endl;
		std::stringstream ss;
		ss << "D:/Hp/objects/deneme/cloud_cluster_" << j << ".pcd";
		writer.write<pcl::PointXYZ>(ss.str(), *cloud_cluster, false); //*
		j++;
	}

	//                   B O U N D I N G         B O X
	pcl::PointCloud<pcl::PointXYZ>::Ptr bb_cloud(new pcl::PointCloud<pcl::PointXYZ>());
	//pcl::PCDReader reader;
	std::ofstream myfile;
	myfile.open("D:/Hp/myfile2.txt");

	for (int k = 0; k < 5; k++) {
		std::stringstream clusterFilePath;

		clusterFilePath << "D:/Hp/objects/deneme/cloud_cluster_" << k << ".pcd";
		
		reader.read(clusterFilePath.str(), *bb_cloud);

		pcl::MomentOfInertiaEstimation <pcl::PointXYZ> feature_extractor;
		feature_extractor.setInputCloud(bb_cloud);
		feature_extractor.compute();

		std::vector <float> moment_of_inertia;
		std::vector <float> eccentricity;
		pcl::PointXYZ min_point_AABB;
		pcl::PointXYZ max_point_AABB;
		//pcl::PointXYZ min_point_OBB;
		//pcl::PointXYZ max_point_OBB;
		//pcl::PointXYZ position_OBB;
		//Eigen::Matrix3f rotational_matrix_OBB;
		//float major_value, middle_value, minor_value;
		//Eigen::Vector3f major_vector, middle_vector, minor_vector;
		//Eigen::Vector3f mass_center;

		feature_extractor.getMomentOfInertia(moment_of_inertia);
		feature_extractor.getEccentricity(eccentricity);
		feature_extractor.getAABB(min_point_AABB, max_point_AABB);
		std::cout << "min points" << min_point_AABB.x << std::endl;
		std::cout << "max points" << max_point_AABB.x << std::endl;

		if (myfile.is_open()) {
			myfile << min_point_AABB.x << " " << min_point_AABB.y << " " << min_point_AABB.z << "\n" << max_point_AABB.x << " " << max_point_AABB.y << " " << max_point_AABB.z << "\n";
		}
		
	}
	myfile.close();
	
	

	//feature_extractor.getOBB(min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);

	//feature_extractor.getEigenValues(major_value, middle_value, minor_value);
	//feature_extractor.getEigenVectors(major_vector, middle_vector, minor_vector);
	//feature_extractor.getMassCenter(mass_center);

	//pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	//viewer->setBackgroundColor(0, 0, 0);
	//viewer->addCoordinateSystem(1.0);
	//viewer->initCameraParameters();
	//viewer->addPointCloud<pcl::PointXYZ>(cloud, "sample cloud");
	//viewer->addCube(min_point_AABB.x, max_point_AABB.x, min_point_AABB.y, max_point_AABB.y, min_point_AABB.z, max_point_AABB.z, 1.0, 1.0, 0.0, "AABB");
	//viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "AABB");

	//Eigen::Vector3f position(position_OBB.x, position_OBB.y, position_OBB.z);
	//Eigen::Quaternionf quat(rotational_matrix_OBB);
	//viewer->addCube(position, quat, max_point_OBB.x - min_point_OBB.x, max_point_OBB.y - min_point_OBB.y, max_point_OBB.z - min_point_OBB.z, "OBB");
	//viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "OBB");

	//pcl::PointXYZ center(mass_center(0), mass_center(1), mass_center(2));
	//pcl::PointXYZ x_axis(major_vector(0) + mass_center(0), major_vector(1) + mass_center(1), major_vector(2) + mass_center(2));
	//pcl::PointXYZ y_axis(middle_vector(0) + mass_center(0), middle_vector(1) + mass_center(1), middle_vector(2) + mass_center(2));
	//pcl::PointXYZ z_axis(minor_vector(0) + mass_center(0), minor_vector(1) + mass_center(1), minor_vector(2) + mass_center(2));
	//viewer->addLine(center, x_axis, 1.0f, 0.0f, 0.0f, "major eigen vector");
	//viewer->addLine(center, y_axis, 0.0f, 1.0f, 0.0f, "middle eigen vector");
	//viewer->addLine(center, z_axis, 0.0f, 0.0f, 1.0f, "minor eigen vector");
	/*
	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		std::this_thread::sleep_for(100ms);
	}*/


	return (0);
}
	