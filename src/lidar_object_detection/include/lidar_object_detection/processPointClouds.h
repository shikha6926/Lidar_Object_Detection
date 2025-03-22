// PCL lib Functions for processing point clouds 

#ifndef PROCESSPOINTCLOUDS_H_
#define PROCESSPOINTCLOUDS_H_

#include <cstddef>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/common.h>
#include <pcl/segmentation/extract_clusters.h>  // Use EuclideanClusterExtraction here
#include <pcl/filters/voxel_grid.h>  // For VoxelGrid filter
#include <pcl/filters/crop_box.h>    // For CropBox filter
#include <pcl/filters/extract_indices.h> // For ExtractIndices

#include "box.h"
#include "kdtree.h"

template<typename PointT>
class ProcessPointClouds {
public:

    //constructor
    ProcessPointClouds();
    //deconstructor
    ~ProcessPointClouds();

    // void numPoints(typename pcl::PointCloud<PointT>::Ptr cloud);

    typename pcl::PointCloud<PointT>::Ptr FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint);

    // std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> SeparateCloudsFromScratch(std::unordered_set<int> inliers, typename pcl::PointCloud<PointT>::Ptr cloud);

    // std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> Ransac3D(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol);

    // std::vector<typename pcl::PointCloud<PointT>::Ptr> Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize);
    
    void proximity(typename pcl::PointCloud<PointT>::Ptr cloud, int& idx, std::vector<bool>& processed, typename pcl::PointCloud<PointT>::Ptr& cluster, KdTree<PointT>* tree, float distanceTol);

    std::vector<typename pcl::PointCloud<PointT>::Ptr> euclideanCluster(typename pcl::PointCloud<PointT>::Ptr cloud, KdTree<PointT>* tree, float distanceTol);

    std::vector<typename pcl::PointCloud<PointT>::Ptr> ClusteringFromScratch(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, std::size_t minSize, std::size_t maxSize);

    Box BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster);
    
    BoxQ ComputeOBB(typename pcl::PointCloud<PointT>::Ptr cluster);

    // void savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file);

    typename pcl::PointCloud<PointT>::Ptr loadPcd(std::string file);

    // std::vector<boost::filesystem::path> streamPcd(std::string dataPath);
  
};

//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
    typename pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);
    // Create the filtering object, for downsampling the pointCloud
    pcl::VoxelGrid<PointT> sor;
    sor.setInputCloud (cloud);
    sor.setLeafSize (filterRes, filterRes, filterRes);
    sor.filter (*cloud_filtered);

    typename pcl::PointCloud<PointT>::Ptr cloud_bounded (new pcl::PointCloud<PointT>);
    
    // Crop the point cloud for a certain region
    pcl::CropBox<PointT> region(true);
    region.setMin(minPoint);
    region.setMax(maxPoint);
    region.setInputCloud (cloud_filtered);
    region.filter(*cloud_bounded);

    // remove the roof points
    std::vector<int> indices;

    pcl::CropBox<PointT> roof(true);
    roof.setMin(Eigen::Vector4f(-1.5,-1.7,-1,1));
    roof.setMax(Eigen::Vector4f(2.6,1.7,-0.4,1));
    roof.setInputCloud (cloud_bounded);
    roof.filter(indices);

    pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
     for (auto point : indices){
        inliers->indices.push_back(point);
    }

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud (cloud_bounded);
    extract.setIndices (inliers);
    extract.setNegative (true);
    extract.filter (*cloud_bounded);


    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloud_bounded;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateCloudsFromScratch(std::unordered_set<int> inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
    typename pcl::PointCloud<PointT>::Ptr obstCloud (new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr planeCloud (new pcl::PointCloud<PointT>());

    for(std::size_t index = 0; index < cloud->points.size(); index++)
	{
		PointT point = cloud->points[index];
		if(inliers.count(index))
			planeCloud->points.push_back(point);
		else
			obstCloud->points.push_back(point);
	}

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstCloud, planeCloud);
    return segResult;
}

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Ransac3D(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	auto startTime = std::chrono::steady_clock::now();
	
	while(maxIterations !=0){
		maxIterations--;
    
		std::unordered_set<int> inliers;
		while(inliers.size() < 3){
			inliers.insert(rand() % (cloud->points.size()));
		}

		float x1, y1, z1, x2, y2, z2, x3, y3, z3;

		auto itr = inliers.begin();
		x1 = cloud->points[*itr].x;
		y1 = cloud->points[*itr].y;
		z1 = cloud->points[*itr].z;
		itr++;
		x2 = cloud->points[*itr].x;
		y2 = cloud->points[*itr].y;
		z2 = cloud->points[*itr].z;
		itr++;
		x3 = cloud->points[*itr].x;
		y3 = cloud->points[*itr].y;
		z3 = cloud->points[*itr].z;

        PointT v1{x2-x3, y2-y1, z2-z1};
		PointT v2{x3-x1, y3-y1, z3-z1};

		float A = v1.y * v2.z - v1.z * v2.y;
		float B = v1.z * v2.x - v1.x * v2.z;
		float C = v1.x * v2.y - v1.y * v2.x;
		float D = - (A*x1  +  B*y1 + C*z1);

		// Measure distance between every point and fitted line
		// If distance is smaller than threshold count it as inlier
		for(std::size_t idx = 0; idx < cloud->points.size(); idx++){
			
			if(inliers.count(idx) > 0)
				continue;

			PointT point = cloud->points[idx];
			
			float distance = fabs(A*point.x + B*point.y + C*point.z + D) / std::sqrt(A*A + B*B + C*C);
			
			if (distance <= distanceTol) {
                inliers.insert(idx);
            }
		}

		if(inliers.size() > inliersResult.size()){
			inliersResult = inliers;
		}
	}

	auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "RANSAC algo:  " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult;
    segResult = SeparateCloudsFromScratch(inliersResult,cloud);

	// Return indicies of inliers from fitted plane with most inliers
	return segResult;
	
}

template<typename PointT>
void ProcessPointClouds<PointT>::proximity(typename pcl::PointCloud<PointT>::Ptr cloud, int& idx, std::vector<bool>& processed, typename pcl::PointCloud<PointT>::Ptr& cluster, KdTree<PointT>* tree, float distanceTol)
{

	processed[idx] = true;
	// cluster->push_back(idx);
	cluster->push_back(cloud->points[idx]);
	std::vector<int> nearby = tree->search(cloud->points[idx], distanceTol);
	
	for(int index : nearby){
		if(!processed[index]){
			proximity(cloud, index, processed, cluster, tree, distanceTol);
		}
	}
	
}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::euclideanCluster(typename pcl::PointCloud<PointT>::Ptr cloud, KdTree<PointT>* tree, float distanceTol)
{

	std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

	std::vector<bool> processedPts(cloud->points.size(), false);

	int idx = 0;

	while((std::size_t)idx < cloud->points.size()){

		if(processedPts[idx] == true){
			idx++;
			continue;
		}

		typename pcl::PointCloud<PointT>::Ptr cluster (new pcl::PointCloud<PointT>);
		proximity(cloud, idx, processedPts, cluster, tree, distanceTol);
		clusters.push_back(cluster);
	}

 
	return clusters;

}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::ClusteringFromScratch(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, std::size_t minSize, std::size_t maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();
    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    KdTree<PointT>* tree = new KdTree<PointT>;
  
    for (std::size_t i=0; i < cloud->points.size(); i++){ 
    	tree->insert(cloud->points[i], i); 
  	}
    
    std::vector<typename pcl::PointCloud<PointT>::Ptr> allClustersObserved;

  	allClustersObserved = euclideanCluster(cloud, tree, clusterTolerance);

    for(auto& cloud_cluster : allClustersObserved){
        if((cloud_cluster->size() > minSize) && (cloud_cluster->size() < maxSize)){
            clusters.push_back(cloud_cluster);
        }
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}

template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}

template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{
    typename pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
    pcl::io::loadPCDFile(file, *cloud);
    return cloud;
}


#endif /* PROCESSPOINTCLOUDS_H_ */