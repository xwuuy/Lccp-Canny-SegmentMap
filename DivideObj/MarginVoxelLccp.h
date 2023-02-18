#pragma once
#ifndef MarginVoxelLccp_hpp
#define MarginVoxelLccp_hpp

#include "lccp_segmentation.h"
#include "supervoxel_clustering.h"

using PointT = pcl::PointXYZRGBA;  // The point type used for input
typedef pcl::PointCloud<PointT> PointCloudT;
namespace ORBSLAM2 {
class MarginVoxelLccp {
    class matchDetail{
    public:
        matchDetail(size_t matchPoint_Num, size_t matchVoxel_Num,PointCloudT::Ptr total_Point):
            matchPointNum(matchPoint_Num),matchVoxelNum(matchVoxel_Num),totalPoint(total_Point)
        {

        }
        size_t matchPointNum;//num of Points in Mask
        size_t matchVoxelNum;//num of supervoxel in Mask
        PointCloudT::Ptr totalPoint;// lccp segment Pointcloud
        typedef boost::shared_ptr<matchDetail> Ptr;
    };
	public:
        MarginVoxelLccp(float voxel_resolution, float seed_resolution, PointCloudT::Ptr edgeCloud, cv::Rect pointRect,
                        cv::Mat mask, cv::Rect ROI,
                        float fx = 517.306408, float fy = 516.469215, float cx = 318.643040, float cy = 255.313989 ,float maskProportion = 0.7);
        MarginVoxelLccp(PointCloudT::Ptr edgeCloud, cv::Rect pointRect,
                        cv::Mat mask, cv::Rect ROI,
                        float fx = 517.306408, float fy = 516.469215, float cx = 318.643040, float cy = 255.313989 ,float maskProportion = 0.7);
        void setSuperVoxel(float color_importance, float spatial_importance, float normal_importance);
		void setSuperVoxelWithoutMargin(float color_importance, float spatial_importance, float normal_importance);
		void setSuperVoxelAdaptive(float color_importance, float spatial_importance, float normal_importance);
        void createLccp(float concavity_tolerance_threshold = 30, float smoothness_threshold = 0.01, std::uint32_t min_segment_size = 0, bool connect_MarginVoxel_arg=false, bool use_sanity_criterion = true, unsigned int k_factor = 0);
		template<class Point>
		bool isInMask(Point p) {
            int x =std::round( (p.x*fx / p.z + cx ));
            int y =std::round( (p.y*fy / p.z + cy ));
            if(ROI.contains(cv::Point(x,y))){
                x -= ROI.x;
                y -= ROI.y;
                return mask.ptr<uchar>(y)[x] > 0;
            }else{
                return false;
            }
		}
        auto getSVAdjacencyList() {
            pcl::LCCPSegmentation<PointT>::SupervoxelAdjacencyList sv_adjacency_list;
            lccp.getSVAdjacencyList(sv_adjacency_list);  // Needed for visualization
            return sv_adjacency_list;
		}

        uint BFS(uint32_t seed,  boost::unordered_map<uint32_t, std::pair<size_t, PointCloudT::Ptr>>& matchPointCloud, /*std::unordered_set<uint32_t> &connectedvertexes, */const std::map<uint32_t, std::set<uint32_t>> &seg_label_to_neighbor_set_map_, const std::map<uint32_t, std::set<uint32_t> > &seg_label_to_sv_list_map_,
                 std::unordered_set<uint32_t> &childgraph);
        uint BFS(uint32_t seed,  boost::unordered_map<uint32_t, matchDetail::Ptr>& matchPointCloud, /*std::unordered_set<uint32_t> &connectedvertexes, */const std::map<uint32_t, std::set<uint32_t>> &seg_label_to_neighbor_set_map_,
                 std::unordered_set<uint32_t> &childgraph);
		std::unordered_set<uint32_t> getMaxConnectedgraph
        ( boost::unordered_map<uint32_t, std::pair<size_t, PointCloudT::Ptr>> matchPointCloud, const std::map<uint32_t, std::set<uint32_t>> &seg_label_to_neighbor_set_map_, const std::map<uint32_t, std::set<uint32_t> > &seg_label_to_sv_list_map_, const uint32_t &seed,  uint matchedvoxelnum);
        std::unordered_set<uint32_t> getMaxConnectedgraph
        ( boost::unordered_map<uint32_t, matchDetail::Ptr> matchPointCloud, const std::map<uint32_t, std::set<uint32_t>> &seg_label_to_neighbor_set_map_, const std::map<uint32_t, std::set<uint32_t> > &seg_label_to_sv_list_map_, const uint32_t &seed,  uint matchedvoxelnum);

        PointCloudT::Ptr segmentObjPoinCloud(int seedSflag=0);
        PointCloudT::Ptr segmentObjVoxelCloud(int seedSflag=0);
		PointCloudT::Ptr edgeCloud;
        pcl::PointCloud<pcl::PointXYZL>::Ptr spuervoxellabelCloud(){
            return super.getFullLabeledWithMarginCloud(marginvoxel_set_);
        }

        pcl::PointCloud<pcl::PointXYZL>::Ptr lccplabelCloud() {
			pcl::PointCloud<pcl::PointXYZL>::Ptr sv_labeled_cloud = super.getLabeledCloud();
			pcl::PointCloud<pcl::PointXYZL>::Ptr lccp_labeled_cloud = sv_labeled_cloud->makeShared();
			lccp.relabelCloud(*lccp_labeled_cloud);
			return lccp_labeled_cloud; 
		}
        std::map<std::uint32_t, pcl::Supervoxel<PointT>::Ptr> getsupervoxel_clusters(){
            return supervoxel_clusters;
        }
        cv::Rect pointRect;
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
