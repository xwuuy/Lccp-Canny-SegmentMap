#include "MarginVoxelLccp.h"
#include<stack>
#include<map>
ORBSLAM2::MarginVoxelLccp::MarginVoxelLccp(float voxel_resolution, float seed_resolution, PointCloudT::Ptr edgeCloud, cv::Mat mask, cv::Rect ROI, float maskProportion, float fx, float fy, float cx, float cy)
	:super(voxel_resolution, seed_resolution),
	edgeCloud(edgeCloud),
	mask(mask),
	maskProportion(maskProportion),
	ROI(ROI),
	fx(fx),
	fy(fy),
	cx(cx),
	cy(cy)
{


}

void ORBSLAM2::MarginVoxelLccp::setSuperVoxel(float color_importance, float spatial_importance, float normal_importance) {
	super.setInputCloud(edgeCloud);
	super.setColorImportance(color_importance);
	super.setSpatialImportance(spatial_importance);
	super.setNormalImportance(normal_importance);
	super.extract(supervoxel_clusters);
	super.refineSupervoxels(1, supervoxel_clusters);
	marginvoxel_set_ = super.setMarginVoxel(ROI, 5, fx, fy, cx, cy);//边缘体素提取
	super.getSupervoxelAdjacency(supervoxel_adjacency);
}
void ORBSLAM2::MarginVoxelLccp::createLccp(float concavity_tolerance_threshold, float smoothness_threshold, std::uint32_t min_segment_size, bool use_extended_convexity, bool use_sanity_criterion, unsigned int k_factor) {
	lccp = pcl::LCCPSegmentation<PointT>();
	lccp.setConcavityToleranceThreshold(concavity_tolerance_threshold);
	lccp.setSanityCheck(use_sanity_criterion);
	lccp.setSmoothnessCheck(true, super.getVoxelResolution(), super.getSeedResolution(), smoothness_threshold);
	lccp.setKFactor(k_factor);
	lccp.setInputSupervoxels(supervoxel_clusters, supervoxel_adjacency, marginvoxel_set_);
	lccp.setMinSegmentSize(min_segment_size);
}



uint ORBSLAM2::MarginVoxelLccp::BFS(const uint32_t &seed, const boost::unordered_map<uint32_t, std::pair<int, PointCloudT::Ptr>>& matchPointCloud,  std::unordered_set<uint32_t> &connectedvertexes, const std::map<uint32_t, std::set<uint32_t>> &seg_label_to_neighbor_set_map_, const std::map<uint32_t, std::set<uint32_t> > &seg_label_to_sv_list_map_, std::unordered_set<uint32_t> &childgraph)
{
	std::stack<uint32_t> stackvoxel;
	auto itneigbor = seg_label_to_neighbor_set_map_.find(seed);
	uint voxelnum = itneigbor->second.size();//获得当前分割区域包含的超体素
	/*if (voxelnum == 0) {
		return
	}*/
	stackvoxel.push(seed);
	connectedvertexes.insert(seed);
	childgraph.insert(seed);
	while (!stackvoxel.empty()) {
		uint32_t index = stackvoxel.top();
		stackvoxel.pop();
		auto &voxelset = seg_label_to_neighbor_set_map_.find(index)->second;
		if (voxelset.empty()) {
			continue;
		}
		for (auto i = voxelset.begin(); i != voxelset.end(); ++i) {//遍历当前顶点的邻接顶点
			if (matchPointCloud.find(*i) != matchPointCloud.end() && connectedvertexes.find(*i) == connectedvertexes.end()) {
				connectedvertexes.insert(*i);//加入已遍历的节点
				childgraph.insert(*i);//加入子图
				stackvoxel.push(*i);//加入栈
				voxelnum += seg_label_to_sv_list_map_.find(*i)->second.size();//统计当前子图的超体素数量
			}
		}
	}
	return voxelnum;
}

std::unordered_set<uint32_t> ORBSLAM2::MarginVoxelLccp::getMaxConnectedgraph(const boost::unordered_map<uint32_t, std::pair<int, PointCloudT::Ptr>>& matchPointCloud, const std::map<uint32_t, std::set<uint32_t>> &seg_label_to_neighbor_set_map_, const std::map<uint32_t, std::set<uint32_t>>& seg_label_to_sv_list_map_, const uint32_t & seed, float matchedvoxelnum)
{
	
	auto it = matchPointCloud.find(seed);
	std::unordered_set<uint32_t> connectedvertexes;
	std::unordered_set<uint32_t> Biggestchildgraph;
	uint maxvoxelnum = 0;
	std::unordered_set<uint32_t> childgraph;
	/*首先对包含超体素最多的节点进行广度优先搜索*/
	uint voxelnum = BFS(it->first, matchPointCloud, connectedvertexes, seg_label_to_neighbor_set_map_, seg_label_to_sv_list_map_, childgraph);
	if (voxelnum >(matchedvoxelnum / 2)) {
		return childgraph;
	}
	it = matchPointCloud.begin();
	while (it != matchPointCloud.end())
	{
		if (connectedvertexes.find(it->first) != connectedvertexes.end()) {
			++it;
			continue;
		}
		//bfs
		std::unordered_set<uint32_t> childgraph;
		uint voxelnum = BFS(it->first, matchPointCloud, connectedvertexes, seg_label_to_neighbor_set_map_, seg_label_to_sv_list_map_, childgraph);
		Biggestchildgraph = maxvoxelnum < voxelnum ? childgraph : Biggestchildgraph;
		if (voxelnum > (matchedvoxelnum / 2)) {
			break;
		}
		++it;
	}
	return Biggestchildgraph;
}


PointCloudT::Ptr ORBSLAM2::MarginVoxelLccp::segmentObjPoinCloud() {
	lccp.segment();
	boost::unordered_map<uint32_t, std::pair<int, PointCloudT::Ptr>> matchPointCloud;
	std::pair<uint32_t, size_t> seed(0,0);//选择包含超体素最多的做为种子
	int matchedvoxelnum = 0;
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
				if (res != matchPointCloud.end()) {
					if (inMask) {
						++res->second.first;
						size_t size = lccp.seg_label_to_sv_list_map_[label].size();
						seed = seed.second < size ?  std::make_pair(label,size) :seed ;
					}
					res->second.second->push_back(*i_labeled);
				}
				else {
					PointCloudT::Ptr voxelPoint(new PointCloudT());
					std::pair<int, PointCloudT::Ptr > pnum;
					if (inMask) {
						++matchedvoxelnum;
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
	std::unordered_set<uint32_t> connectedGrap = getMaxConnectedgraph(matchPointCloud, lccp.seg_label_to_neighbor_set_map_, lccp.seg_label_to_sv_list_map_,seed.first, matchedvoxelnum);
	PointCloudT::Ptr objcloud(new PointCloudT);
	
	for (auto i = connectedGrap.begin(); i != connectedGrap.end();++ i)
	{
		auto pointcloud = matchPointCloud[*i];
		float n = pointcloud.first;
		float m = pointcloud.second->size();
		float rate = n / m;
		if (rate > maskProportion)
		{
			*objcloud += *pointcloud.second;//点云相加
		}
	}
	return objcloud;
}

