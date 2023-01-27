#include "MarginVoxelLccp.h"
#include<stack>
#include<map>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
ORBSLAM2::MarginVoxelLccp::MarginVoxelLccp
(float voxel_resolution, float seed_resolution, PointCloudT::Ptr edgeCloud, cv::Rect pointRect, cv::Mat mask,
 cv::Rect ROI, float fx, float fy, float cx, float cy,float maskProportion)
	:super(voxel_resolution, seed_resolution),
	edgeCloud(edgeCloud),
    pointRect(pointRect),
    mask(mask),
	ROI(ROI),
	fx(fx),
	fy(fy),
	cx(cx),
    cy(cy),
    maskProportion(maskProportion)
{

}

ORBSLAM2::MarginVoxelLccp::MarginVoxelLccp(PointCloudT::Ptr edgeCloud, cv::Rect pointRect, cv::Mat mask,
    cv::Rect ROI,float fx, float fy, float cx, float cy, float maskProportion)
    :edgeCloud(edgeCloud),
    pointRect(pointRect),
    mask(mask),
    ROI(ROI),
    fx(fx),
    fy(fy),
    cx(cx),
    cy(cy),
    maskProportion(maskProportion)
{
    int shorter=std::min(ROI.width,ROI.height);
    if(shorter>290){
        super=ORBSLAM2::SupervoxelClustering<PointT>( 0.02,0.08);
    }else{
        if(shorter>100){
            super=ORBSLAM2::SupervoxelClustering<PointT>(0.014,0.06 );
        }else{
            super=ORBSLAM2::SupervoxelClustering<PointT>(0.008, 0.04);
        }
    }
}


void ORBSLAM2::MarginVoxelLccp::setSuperVoxel(float color_importance, float spatial_importance, float normal_importance) {
	super.setInputCloud(edgeCloud);
	super.setColorImportance(color_importance);
	super.setSpatialImportance(spatial_importance);
	super.setNormalImportance(normal_importance);
	super.extract(supervoxel_clusters);
	super.refineSupervoxels(1, supervoxel_clusters);
    marginvoxel_set_ = super.setMarginVoxel(pointRect, 5, fx, fy, cx, cy);//边缘体素提取
	super.getSupervoxelAdjacency(supervoxel_adjacency);
}

void ORBSLAM2::MarginVoxelLccp::setSuperVoxelWithoutMargin(float color_importance, float spatial_importance, float normal_importance) {
	super.setInputCloud(edgeCloud);
	super.setColorImportance(color_importance);
	super.setSpatialImportance(spatial_importance);
	super.setNormalImportance(normal_importance);
	super.extract(supervoxel_clusters);
	super.refineSupervoxels(1, supervoxel_clusters);
	super.getSupervoxelAdjacency(supervoxel_adjacency);
}
void ORBSLAM2::MarginVoxelLccp::setSuperVoxelAdaptive(float color_importance, float spatial_importance, float normal_importance) {
	super.setInputCloud(edgeCloud);
	super.setColorImportance(color_importance);
	super.setSpatialImportance(spatial_importance);
	super.setNormalImportance(normal_importance);
	super.extract(supervoxel_clusters);
    super.refineSupervoxels(1, supervoxel_clusters);
    marginvoxel_set_ = super.setMarginVoxelAdaptive(ROI, fx, fy, cx, cy);//边缘体素提取
	super.getSupervoxelAdjacency(supervoxel_adjacency);
}

void ORBSLAM2::MarginVoxelLccp::createLccp(float concavity_tolerance_threshold, float smoothness_threshold, std::uint32_t min_segment_size, bool use_sanity_criterion, unsigned int k_factor) {
	lccp = pcl::LCCPSegmentation<PointT>();
	lccp.setConcavityToleranceThreshold(concavity_tolerance_threshold);
	lccp.setSanityCheck(use_sanity_criterion);
	lccp.setSmoothnessCheck(true, super.getVoxelResolution(), super.getSeedResolution(), smoothness_threshold);
	lccp.setKFactor(k_factor);
	lccp.setInputSupervoxels(supervoxel_clusters, supervoxel_adjacency, marginvoxel_set_);
    lccp.setMinSegmentSize(min_segment_size);
}



uint ORBSLAM2::MarginVoxelLccp::BFS(const uint32_t &seed, boost::unordered_map<uint32_t, std::pair<int, PointCloudT::Ptr>>& matchPointCloud, const std::map<uint32_t, std::set<uint32_t>> &seg_label_to_neighbor_set_map_, const std::map<uint32_t, std::set<uint32_t> > &seg_label_to_sv_list_map_, std::unordered_set<uint32_t> &childgraph, uint &matchedvoxelnum)
{
	childgraph.clear();
    std::stack<uint32_t> stackvoxel;
    uint voxelnum =seg_label_to_sv_list_map_.find(seed)->second.size();//获得当前分割区域包含的超体素
	/*if (voxelnum == 0) {
		return
	}*/
	stackvoxel.push(seed);
    matchPointCloud.erase(seed);
	childgraph.insert(seed);
	while (!stackvoxel.empty()) {
		uint32_t index = stackvoxel.top();
		stackvoxel.pop();
		auto &voxelset = seg_label_to_neighbor_set_map_.find(index)->second;
		if (voxelset.empty()) {
			continue;
		}
        for (auto i = voxelset.begin(); i != voxelset.end(); ++i) {//遍历当前顶点的邻接顶点
			auto mpcit = matchPointCloud.find(*i);
            if (mpcit != matchPointCloud.end()) {
                int tempvnum=seg_label_to_sv_list_map_.find(*i)->second.size();
                if((float)mpcit->second.first / (float)mpcit->second.second->size()>maskProportion){
                    childgraph.insert(*i);//加入子图
                    stackvoxel.push(*i);//加入栈
                    voxelnum += tempvnum;//统计当前子图的超体素数量
                }
                matchPointCloud.erase(*i);//delete label
                matchedvoxelnum-=tempvnum;
			}
		}
	}
	return voxelnum;
}

std::unordered_set<uint32_t> ORBSLAM2::MarginVoxelLccp::getMaxConnectedgraph
(boost::unordered_map<uint32_t, std::pair<int, PointCloudT::Ptr>> matchPointCloud, const std::map<uint32_t,
 std::set<uint32_t>> &seg_label_to_neighbor_set_map_, const std::map<uint32_t, std::set<uint32_t>>& seg_label_to_sv_list_map_,
 const uint32_t & seed, uint matchedvoxelnum)
{
	
    auto it = matchPointCloud.find(seed);
	std::unordered_set<uint32_t> Biggestchildgraph;
	uint maxvoxelnum = 0;
	std::unordered_set<uint32_t> childgraph;
	/*首先对包含超体素最多的节点进行广度优先搜索*/
    if ((float)it->second.first/(float) it->second.second->size()>maskProportion) {
        uint voxelnum = BFS(it->first, matchPointCloud,  seg_label_to_neighbor_set_map_, seg_label_to_sv_list_map_, childgraph,matchedvoxelnum);
		if (voxelnum >(matchedvoxelnum / 2)) {
			return childgraph;
		}
        matchedvoxelnum-=voxelnum;
        maxvoxelnum=voxelnum;
        Biggestchildgraph=childgraph;
	}
	it = matchPointCloud.begin();
	while (it != matchPointCloud.end())
	{
        auto d=it->second.second;
		if ((float)it->second.first / (float)it->second.second->size() < maskProportion) {
            matchedvoxelnum-=seg_label_to_sv_list_map_.find(it->first)->second.size();//获得当前分割区域包含的超体素;
            it=matchPointCloud.erase(it);
			continue;
		}
		//bfs
		std::unordered_set<uint32_t> childgraph;
        uint voxelnum = BFS(it->first, matchPointCloud, seg_label_to_neighbor_set_map_, seg_label_to_sv_list_map_, childgraph,matchedvoxelnum);
        if( maxvoxelnum < voxelnum){
            Biggestchildgraph = childgraph;
            maxvoxelnum=voxelnum;
        }
        if (maxvoxelnum > (matchedvoxelnum / 2)) {
			break;
        }
        matchedvoxelnum-=voxelnum;
        it=matchPointCloud.begin();
	}
	return Biggestchildgraph;
}


PointCloudT::Ptr ORBSLAM2::MarginVoxelLccp::segmentObjPoinCloud() {
	lccp.segment();
	boost::unordered_map<uint32_t, std::pair<int, PointCloudT::Ptr>> matchPointCloud;
	std::pair<uint32_t, size_t> seed(0,0);//选择包含超体素最多的做为种子
	uint matchedvoxelnum = 0;
	for (auto i_labeled = edgeCloud->begin(); i_labeled != edgeCloud->end(); ++i_labeled)
	{
		if (pcl::isFinite<PointT>(*i_labeled))
		{
			ORBSLAM2::SupervoxelClustering<PointT>::LeafContainerT *leaf = super.adjacency_octree_->getLeafContainerAtPoint(*i_labeled);
			ORBSLAM2::SupervoxelClustering<PointT>::VoxelData& voxel_data = leaf->getData();
			if (voxel_data.owner_)
			{
				uint32_t label = lccp.sv_label_to_seg_label_map_[voxel_data.owner_->getLabel()];
				//i_labeled->r = 255;
				//i_labeled->g = 255;
				//i_labeled->b = 255;
				bool inMask = isInMask(*i_labeled);
				auto res=matchPointCloud.find(label);
                if (res != matchPointCloud.end()) {//justic is or not in matchPointCloud
					if (inMask) {
                        size_t size = lccp.seg_label_to_sv_list_map_[label].size();
                        seed = seed.second < size ?  std::make_pair(label,size) :seed ;
                        if(res->second.first==0){
                            matchedvoxelnum+= size;
                        }
                        ++res->second.first;//add voxel in mask num
					}
					res->second.second->push_back(*i_labeled);
				}
				else {
					PointCloudT::Ptr voxelPoint(new PointCloudT());
					std::pair<int, PointCloudT::Ptr > pnum;
					if (inMask) {
						matchedvoxelnum+= lccp.seg_label_to_sv_list_map_[label].size();;
						pnum.first = 1;
					}
					else {
						pnum.first = 0;
					}
					voxelPoint->push_back(*i_labeled);
					pnum.second = voxelPoint;
					matchPointCloud.insert({ label ,pnum });
				}
			}
		}
	}
	PointCloudT::Ptr objcloud(new PointCloudT);
    if(matchPointCloud.size()==0||matchedvoxelnum==0||seed.first==0){
        return objcloud;
    }
    std::unordered_set<uint32_t> connectedGrap = getMaxConnectedgraph(
                matchPointCloud, lccp.seg_label_to_neighbor_set_map_, lccp.seg_label_to_sv_list_map_,seed.first, matchedvoxelnum);

	
	for (auto i = connectedGrap.begin(); i != connectedGrap.end(); ++i)
	{
		auto pointcloud = matchPointCloud[*i];
		*objcloud += *pointcloud.second;//点云相加
	}
	//for (auto i = matchPointCloud.begin(); i != matchPointCloud.end(); ++i)
	//{
	//	auto pointcloud = i->second;
	//	float n = pointcloud.first;
	//	float m = pointcloud.second->size();
	//	float rate = n / m;
	//	if (rate > maskProportion)
	//	{
	//		*objcloud += *pointcloud.second;//点云相加
	//	}
	//}
	return objcloud;
}

PointCloudT::Ptr ORBSLAM2::MarginVoxelLccp::segmentObjVoxelCloud()
{
	lccp.segment();
	boost::unordered_map<uint32_t, std::pair<int, PointCloudT::Ptr>> matchPointCloud;
	std::pair<uint32_t, size_t> seed(0, 0);//选择包含超体素最多的做为种子
	uint matchedvoxelnum = 0;
	for (typename SupervoxelClustering<PointT>::HelperListT::const_iterator sv_itr = super.supervoxel_helpers_.cbegin(); sv_itr != super.supervoxel_helpers_.cend(); ++sv_itr)
	{
		
		uint32_t label = lccp.sv_label_to_seg_label_map_[sv_itr->getLabel()];
		typename PointCloudT::Ptr voxels(new PointCloudT);
		voxels->resize(sv_itr->leaves_.size());
		typename pcl::PointCloud<PointT>::iterator voxel_itr = voxels->begin();
		uint inMaskNum = 0;
		for (auto leaf_itr = sv_itr->leaves_.begin(); leaf_itr != sv_itr->leaves_.end(); ++leaf_itr, ++voxel_itr)
		{
			const auto& leaf_data = (*leaf_itr)->getData();
			leaf_data.getPoint(*voxel_itr);
			if (isInMask(*voxel_itr))
			{
				inMaskNum++;
			}
		}
		auto res = matchPointCloud.insert({ label ,{ inMaskNum ,voxels} });
		if (inMaskNum > 0) {
			matchedvoxelnum++;
		}
		if (res.second)
		{
			seed = seed.second < voxels->size() ?  std::make_pair( label, voxels->size()): seed ;
		}
		else {
			std::pair<int, PointCloudT::Ptr> &temp = res.first->second;
			temp.first += inMaskNum;
			*(temp.second) += *voxels;
			seed = seed.second < temp.second->size() ? std::make_pair(label, temp.second->size()) : seed;
		}
	}


	PointCloudT::Ptr objcloud(new PointCloudT);



	std::unordered_set<uint32_t> connectedGrap = getMaxConnectedgraph(matchPointCloud, lccp.seg_label_to_neighbor_set_map_, lccp.seg_label_to_sv_list_map_, seed.first, matchedvoxelnum);


	for (auto i = connectedGrap.begin(); i != connectedGrap.end(); ++i)
	{
		auto pointcloud = matchPointCloud[*i];
		*objcloud += *pointcloud.second;//点云相加
	}

	//for (auto i = connectedGrap.begin(); i != connectedGrap.end(); ++i)
	//{
	//	auto pointcloud = matchPointCloud[*i];
	//	float n = pointcloud.first;
	//	float m = pointcloud.second->size();
	//	float rate = n / m;
	//	if (rate > maskProportion)
	//	{
	//		*objcloud += *pointcloud.second;//点云相加
	//	}
	//}

	return objcloud;
}

