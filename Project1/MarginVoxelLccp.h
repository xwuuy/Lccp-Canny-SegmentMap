#pragma once
#ifndef MarginVoxelLccp_hpp
#define MarginVoxelLccp_hpp

#include "Thirdpartylib\lccp_segmentation.h"
#include "Thirdpartylib\supervoxel_clustering.h"

using PointT = pcl::PointXYZRGBA;  // The point type used for input
typedef pcl::PointCloud<PointT> PointCloudT;
namespace ORBSLAM2 {
class MarginVoxelLccp {
	public:
		MarginVoxelLccp(float voxel_resolution, float seed_resolution, PointCloudT::Ptr edgeCloud, cv::Mat mask, cv::Rect ROI, float maskProportion = 0.75, float fx = 517.306408, float fy = 516.469215, float cx = 318.643040, float cy = 255.313989);
		void setSuperVoxel(float color_importance, float spatial_importance, float normal_importance);
		void createLccp(float concavity_tolerance_threshold = 8, float smoothness_threshold = 0.01, std::uint32_t min_segment_size = 0, bool use_extended_convexity = false, bool use_sanity_criterion = false, unsigned int k_factor = 0);
		template<class Point>
		bool isInMask(Point p) {
			int x =round( (p.x*fx / p.z + cx - ROI.x));
			int y =round( (p.y*fy / p.z + cy - ROI.y));
			return mask.ptr<uchar>(y)[x] > 0;
		}
		pcl::PointCloud<pcl::PointXYZL>::Ptr labelpoint() {
			lccp.segment();
			pcl::PointCloud<pcl::PointXYZL>::Ptr sv_labeled_cloud = super.getLabeledCloud();
			pcl::PointCloud<pcl::PointXYZL>::Ptr lccp_labeled_cloud = sv_labeled_cloud->makeShared();
			lccp.relabelCloud(*lccp_labeled_cloud);
			return lccp_labeled_cloud;
		}
		uint BFS(const uint32_t &seed, const boost::unordered_map<uint32_t, std::pair<int, PointCloudT::Ptr>>& matchPointCloud, std::unordered_set<uint32_t> &connectedvertexes, const std::map<uint32_t, std::set<uint32_t>> &seg_label_to_neighbor_set_map_, const std::map<uint32_t, std::set<uint32_t> > &seg_label_to_sv_list_map_,   std::unordered_set<uint32_t> &childgraph);
		std::unordered_set<uint32_t> getMaxConnectedgraph
		(const boost::unordered_map<uint32_t, std::pair<int, PointCloudT::Ptr>> &matchPointCloud, const std::map<uint32_t, std::set<uint32_t>> &seg_label_to_neighbor_set_map_, const std::map<uint32_t, std::set<uint32_t> > &seg_label_to_sv_list_map_, const uint32_t &seed, float matchedvoxelnum);
		PointCloudT::Ptr segmentObjPoinCloud();
		PointCloudT::Ptr edgeCloud;
		cv::Mat mask;
		cv::Rect ROI;
		float fx;
		float fy;
		float cx;
		float cy;
	private:
		float maskProportion;
		std::map<std::uint32_t, pcl::Supervoxel<PointT>::Ptr> supervoxel_clusters;
		std::multimap<std::uint32_t, std::uint32_t> supervoxel_adjacency;
		boost::unordered_set<uint32_t>  marginvoxel_set_;
		ORBSLAM2::SupervoxelClustering<PointT> super;
		pcl::LCCPSegmentation<PointT> lccp;
	};
}
#endif