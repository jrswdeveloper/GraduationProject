#include <vector>
#include <thread>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/time.h>
#include <iostream>
#include <sstream>
#include <pcl/filters/project_inliers.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/visualization/cloud_viewer.h>



using namespace std::chrono_literals;

int main(int argc, char** argv)
{
/*
	// Read in the cloud data
	pcl::PCDReader reader;
	pcl::PCDWriter writer;
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
	
	//writer.write("D:/Hp/filteredsahne50.pcd", *cloud_filtered, false);

	// Create the segmentation object for the planar model and set all the parameters

	pcl::SACSegmentation<pcl::PointXYZ> seg;
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>());
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

	//writer.write("D:/Hp/groundplane50.pcd", *cloud_plane, false);

	//					 C L U S T E R I N G

	// Creating the KdTree object for the search method of the extraction
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(cloud_filtered);

	std::vector<pcl::PointIndices> cluster_indices;


	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
	ec.setClusterTolerance(25.0f); // 2500cm
	ec.setMinClusterSize(100);
	ec.setMaxClusterSize(25000);
	ec.setSearchMethod(tree);
	ec.setInputCloud(cloud_filtered);
	ec.extract(cluster_indices);


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
		ss << "D:/Hp/objects/cloud_cluster_" << j << ".pcd";
		writer.write<pcl::PointXYZ>(ss.str(), *cloud_cluster, false); //*
		j++;
	}

	//                   B O U N D I N G         B O X
	pcl::PointCloud<pcl::PointXYZ>::Ptr bb_cloud(new pcl::PointCloud<pcl::PointXYZ>());
	std::ofstream myfile;
	myfile.open("D:/Hp/3dboundingbox_minmaxpoints.txt");

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
*/

	// Cluster projection islemi
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PCDReader reader;
	pcl::PCDWriter writer;

	for (int a = 0; a < 5; a++) {
		std::stringstream clusterpath;
		clusterpath << "D:/Hp/objects/cloud_cluster_" << a << ".pcd";
		reader.read(clusterpath.str(), *cloud);
		// Create a set of planar coefficients with X=Y=0,Z=1
		pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
		coefficients->values.resize(4);
		coefficients->values[0] = coefficients->values[1] = 0;
		coefficients->values[2] = 1.0;
		coefficients->values[3] = 0;

		// Create the filtering object
		pcl::ProjectInliers<pcl::PointXYZ> proj;
		proj.setModelType(pcl::SACMODEL_PLANE);
		proj.setInputCloud(cloud);
		proj.setModelCoefficients(coefficients);
		proj.filter(*cloud_projected);

		// Create a Convex Hull representation of the projected inliers
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::ConvexHull<pcl::PointXYZ> chull;
		chull.setInputCloud(cloud_projected);
		chull.reconstruct(*cloud_hull);

		std::stringstream ss;
		ss << "D:/Hp/projection/boundingbox" << a << ".pcd";
		writer.write<pcl::PointXYZ>(ss.str(), *cloud_hull, false);

	}

	// Cluster'larin projection'larini bir arada görmek icin
	// tum cluster'larin projection'larini tek bir file'a yazdirma islemi

	std::ofstream myfile;
	myfile.open("D:/Hp/projection/myboundingboxfile.txt");

	for (int j = 0; j < 5; j++) {
		std::stringstream path;
		path << "D:/Hp/projection/boundingbox" << j << ".pcd";
		reader.read(path.str(), *cloud_hull);

		for (std::size_t i = 0; i < cloud_hull->points.size(); ++i) {

			if (myfile.is_open()) {
				myfile << cloud_hull->points[i].x << " " << cloud_hull->points[i].y << " " << cloud_hull->points[i].z << "\n";
			}
		}

	}
	myfile.close();
	


	return (0);
}

	