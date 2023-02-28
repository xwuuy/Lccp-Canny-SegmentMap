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
        super=ORBSLAM2::SupervoxelClustering<PointT>( 0.02,0.12);
    }else{
        if(shorter>100){
            super=ORBSLAM2::SupervoxelClustering<PointT>(0.014,0.09 );
        }else{
            super=ORBSLAM2::SupervoxelClustering<PointT>(0.008, 0.06);
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
    super.setMarginVoxel(pointRect, 5, marginvoxel_set_, fx, fy, cx, cy);//边缘体素提取
	super.getSupervoxelAdjacency(supervoxel_adjacency);
}

void ORBSLAM2::MarginVoxelLccp::setSuperVoxelWithoutMargin(float color_importance, float spatial_importance, float normal_importance) {
    super.setUseSingleCameraTransform(true);
	super.setInputCloud(edgeCloud);
	super.setColorImportance(color_importance);
	super.setSpatialImportance(spatial_importance);
	super.setNormalImportance(normal_importance);
	super.extract(supervoxel_clusters);
	super.refineSupervoxels(1, supervoxel_clusters);
	super.getSupervoxelAdjacency(supervoxel_adjacency);
}
void ORBSLAM2::MarginVoxelLccp::setSuperVoxelAdaptive(float color_importance, float spatial_importance, float normal_importance) {
    super.setUseSingleCameraTransform(true);
	super.setInputCloud(edgeCloud);
	super.setColorImportance(color_importance);
	super.setSpatialImportance(spatial_importance);
	super.setNormalImportance(normal_importance);
	super.extract(supervoxel_clusters);
    super.refineSupervoxels(1, supervoxel_clusters);
    super.setMarginVoxelAdaptive(ROI,marginvoxel_set_, fx, fy, cx, cy);//边缘体素提取
	super.getSupervoxelAdjacency(supervoxel_adjacency);
}

void ORBSLAM2::MarginVoxelLccp::createLccp(float concavity_tolerance_threshold, float smoothness_threshold,  bool connect_MarginVoxel_arg, unsigned int k_factor, bool use_sanity_criterion, std::uint32_t min_segment_size ) {
    lccp = pcl::LCCPSegmentation<PointT>();
	lccp.setConcavityToleranceThreshold(concavity_tolerance_threshold);
	lccp.setSanityCheck(use_sanity_criterion);
	lccp.setSmoothnessCheck(true, super.getVoxelResolution(), super.getSeedResolution(), smoothness_threshold);
	lccp.setKFactor(k_factor);
	lccp.setInputSupervoxels(supervoxel_clusters, supervoxel_adjacency, marginvoxel_set_);
    lccp.setMinSegmentSize(min_segment_size);
    lccp.setconnectMarginVoxel(connect_MarginVoxel_arg);
}



uint ORBSLAM2::MarginVoxelLccp::BFS( uint32_t seed, boost::unordered_map<uint32_t, std::pair<size_t, PointCloudT::Ptr> >& matchPointCloud, const std::map<uint32_t, std::set<uint32_t>> &seg_label_to_neighbor_set_map_, const std::map<uint32_t, std::set<uint32_t> > &seg_label_to_sv_list_map_, std::unordered_set<uint32_t> &childgraph)
{
	childgraph.clear();
    std::stack<uint32_t> stackvoxel;
    uint voxelnum =seg_label_to_sv_list_map_.find(seed)->second.size();//获得当前分割区域包含的超体素
	/*if (voxelnum == 0) {
		return
	}*/
	stackvoxel.push(seed);
    childgraph.insert(seed);
    matchPointCloud.erase(seed);
	while (!stackvoxel.empty()) {
		uint32_t index = stackvoxel.top();
		stackvoxel.pop();
        auto neigborit=seg_label_to_neighbor_set_map_.find(index);
        if(neigborit==seg_label_to_neighbor_set_map_.end()){
            continue;
        }
        auto &voxelset = neigborit->second;
        for (auto i = voxelset.begin(); i != voxelset.end(); ++i) {//遍历当前顶点的邻接顶点
            auto in=*i;
            auto mpcit = matchPointCloud.find(in);
            if (mpcit != matchPointCloud.end()) {
                int tempvnum=seg_label_to_sv_list_map_.find(*i)->second.size();
                if((float)mpcit->second.first / (float)mpcit->second.second->size()>maskProportion){
                    childgraph.insert(*i);//加入子图
                    stackvoxel.push(*i);//加入栈
                    voxelnum += tempvnum;//统计当前子图的超体素数量
                }
                matchPointCloud.erase(*i);//delete label
			}
		}
	}
	return voxelnum;
}

uint ORBSLAM2::MarginVoxelLccp::BFS( uint32_t seed, boost::unordered_map<uint32_t, matchDetail::Ptr>& matchPointCloud, const std::map<uint32_t, std::set<uint32_t>> &seg_label_to_neighbor_set_map_, std::unordered_set<uint32_t> &childgraph)
{
    childgraph.clear();
    std::stack<uint32_t> stackvoxel;
    uint voxelnum =matchPointCloud[seed]->matchVoxelNum;//获得当前分割区域包含的存在Mask内的超体素
    /*if (voxelnum == 0) {
        return
    }*/
    stackvoxel.push(seed);
    childgraph.insert(seed);
    matchPointCloud.erase(seed);
    while (!stackvoxel.empty()) {
        uint32_t index = stackvoxel.top();
        stackvoxel.pop();
        auto neigborit=seg_label_to_neighbor_set_map_.find(index);
        if(neigborit==seg_label_to_neighbor_set_map_.end()){
            continue;
        }
        auto &voxelset = neigborit->second;
        for (auto i = voxelset.begin(); i != voxelset.end(); ++i) {//遍历当前顶点的邻接顶点
            auto mpcit = matchPointCloud.find(*i);
            if (mpcit != matchPointCloud.end()) {
                int tempvnum=matchPointCloud[*i]->matchVoxelNum;
                if((float)mpcit->second->matchPointNum / (float)mpcit->second->totalPoint->size()>maskProportion){
                    childgraph.insert(*i);//加入子图
                    stackvoxel.push(*i);//加入栈
                    voxelnum += tempvnum;//统计当前子图的存在于Mask中的超体素数量
                }
                matchPointCloud.erase(*i);//delete label
            }
        }
    }
    return voxelnum;
}

std::unordered_set<uint32_t> ORBSLAM2::MarginVoxelLccp::getMaxConnectedgraph
(boost::unordered_map<uint32_t, std::pair<size_t, PointCloudT::Ptr>> matchPointCloud, const std::map<uint32_t,
 std::set<uint32_t>> &seg_label_to_neighbor_set_map_, const std::map<uint32_t, std::set<uint32_t>>& seg_label_to_sv_list_map_,
 const uint32_t & seed, uint matchedvoxelnum)
{
	
    auto it = matchPointCloud.find(seed);
	std::unordered_set<uint32_t> Biggestchildgraph;
	uint maxvoxelnum = 0;
	std::unordered_set<uint32_t> childgraph;
	/*首先对包含超体素最多的节点进行广度优先搜索*/
    if ((float)it->second.first/(float) it->second.second->size()>maskProportion) {
        uint voxelnum = BFS(it->first, matchPointCloud,  seg_label_to_neighbor_set_map_, seg_label_to_sv_list_map_, childgraph);
		if (voxelnum >(matchedvoxelnum / 2)) {
			return childgraph;
		}
        matchedvoxelnum-=voxelnum;
        maxvoxelnum=voxelnum;
        Biggestchildgraph=childgraph;
    }else{
        if(it->second.first>(matchedvoxelnum / 2)){
         return Biggestchildgraph;
        }
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
        uint voxelnum = BFS(it->first, matchPointCloud, seg_label_to_neighbor_set_map_, seg_label_to_sv_list_map_, childgraph);
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

std::unordered_set<uint32_t> ORBSLAM2::MarginVoxelLccp::getMaxConnectedgraph
(boost::unordered_map<uint32_t, matchDetail::Ptr> matchPointCloud, const std::map<uint32_t,
 std::set<uint32_t>> &seg_label_to_neighbor_set_map_, const std::map<uint32_t, std::set<uint32_t>>& seg_label_to_sv_list_map_,
 const uint32_t & seed, uint matchedvoxelnum)
{

    auto it = matchPointCloud.find(seed);
    std::unordered_set<uint32_t> Biggestchildgraph;
    uint maxvoxelnum = 0;
    std::unordered_set<uint32_t> childgraph;
    /*首先对包含超体素最多的节点进行广度优先搜索*/
    if ((float)it->second->matchPointNum/(float) it->second->totalPoint->size()>maskProportion) {
        uint voxelnum = BFS(it->first, matchPointCloud,  seg_label_to_neighbor_set_map_, childgraph);
        if (voxelnum >(matchedvoxelnum / 2)) {
            return childgraph;
        }
        matchedvoxelnum-=voxelnum;
        maxvoxelnum=voxelnum;
        Biggestchildgraph=childgraph;
    }else{
        if(it->second->matchVoxelNum>(matchedvoxelnum / 2)){
         return Biggestchildgraph;
        }
    }

    it = matchPointCloud.begin();
    while (it != matchPointCloud.end())
    {

        if ((float)it->second->matchPointNum/(float) it->second->totalPoint->size() < maskProportion) {
            matchedvoxelnum-=matchPointCloud[it->first]->matchVoxelNum;//获得当前分割区域包含的超体素;
            it=matchPointCloud.erase(it);
            continue;
        }
        //bfs
        std::unordered_set<uint32_t> childgraph;
        uint voxelnum = BFS(it->first, matchPointCloud, seg_label_to_neighbor_set_map_, childgraph);
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

PointCloudT::Ptr ORBSLAM2::MarginVoxelLccp::segmentObjPoinCloud(int seedSflag) {
	lccp.segment();
    boost::unordered_map<uint32_t, matchDetail::Ptr> matchPointCloud;
    std::pair<uint32_t, size_t> seed(0,0);//选择包含超体素最多的做为种子
    boost::unordered_set<uint32_t> inmaskVoxel;
	for (auto i_labeled = edgeCloud->begin(); i_labeled != edgeCloud->end(); ++i_labeled)
	{
		if (pcl::isFinite<PointT>(*i_labeled))
		{
			ORBSLAM2::SupervoxelClustering<PointT>::LeafContainerT *leaf = super.adjacency_octree_->getLeafContainerAtPoint(*i_labeled);
			ORBSLAM2::SupervoxelClustering<PointT>::VoxelData& voxel_data = leaf->getData();
			if (voxel_data.owner_)
			{
                auto voxelabel=voxel_data.owner_->getLabel();
                uint32_t label = lccp.sv_label_to_seg_label_map_[voxelabel];
				//i_labeled->r = 255;
				//i_labeled->g = 255;
				//i_labeled->b = 255;
				bool inMask = isInMask(*i_labeled);
				auto res=matchPointCloud.find(label);
                if (res != matchPointCloud.end()) {//justic is or not in matchPointCloud
					if (inMask) {
                        size_t size = lccp.seg_label_to_sv_list_map_[label].size();
                        seed = seed.second < size ?  std::make_pair(label,size) :seed ;
                        if(inmaskVoxel.find(voxelabel)==inmaskVoxel.end()){
                            inmaskVoxel.insert(voxelabel);
                            ++res->second->matchVoxelNum;
                        }
                        ++res->second->matchPointNum;//add voxel in mask num
					}
                    res->second->totalPoint->push_back(*i_labeled);
				}
                else {
                    matchDetail::Ptr mm(new matchDetail);
					if (inMask) {
                        if(inmaskVoxel.find(voxelabel)==inmaskVoxel.end()){
                            inmaskVoxel.insert(voxelabel);
                            ++mm->matchVoxelNum;
                        }
                        ++mm->matchPointNum;
                    }
                    mm->totalPoint->push_back(*i_labeled);
                    matchPointCloud.insert({ label ,mm });
				}
			}
		}
	}
	PointCloudT::Ptr objcloud(new PointCloudT);
    if(seedSflag==1){

    }
    if(matchPointCloud.size()==0||inmaskVoxel.size()==0||seed.second<=0){
        return objcloud;
    }
    std::unordered_set<uint32_t> connectedGrap = getMaxConnectedgraph(
                matchPointCloud, lccp.seg_label_to_neighbor_set_map_, lccp.seg_label_to_sv_list_map_,seed.first, inmaskVoxel.size());

	
    for (auto i = connectedGrap.begin(); i != connectedGrap.end(); ++i)
    {
        auto pointcloud = matchPointCloud[*i];
        if(pointcloud->totalPoint)
            *objcloud += *pointcloud->totalPoint;//点云相加
    }
	return objcloud;
}

PointCloudT::Ptr ORBSLAM2::MarginVoxelLccp::segmentObjVoxelCloud(int seedSflag)
{
	lccp.segment();
    boost::unordered_map<uint32_t, matchDetail::Ptr> matchPointCloud;
    std::pair<uint32_t, size_t> seed(0,0);//选择在mask内超体素最多的做为种子
	uint matchedvoxelnum = 0;
	for (typename SupervoxelClustering<PointT>::HelperListT::const_iterator sv_itr = super.supervoxel_helpers_.cbegin(); sv_itr != super.supervoxel_helpers_.cend(); ++sv_itr)
	{		
		uint32_t label = lccp.sv_label_to_seg_label_map_[sv_itr->getLabel()];
        PointCloudT::Ptr voxels(new PointCloudT);
		voxels->resize(sv_itr->leaves_.size());
        pcl::PointCloud<PointT>::iterator voxel_itr = voxels->begin();
        uint inMaskNum = 0;//属于mask的点云数
		for (auto leaf_itr = sv_itr->leaves_.begin(); leaf_itr != sv_itr->leaves_.end(); ++leaf_itr, ++voxel_itr)
		{
            auto& leaf_data = (*leaf_itr)->getData();
			leaf_data.getPoint(*voxel_itr);
			if (isInMask(*voxel_itr))
			{
				inMaskNum++;
			}
		}
        matchDetail::Ptr tempDetail(new matchDetail(0 , 0 ,voxels));
        std::pair<uint32_t, matchDetail::Ptr> tempmatch={label, tempDetail};
        auto res = matchPointCloud.insert(tempmatch);
        auto temp = res.first->second;
        if (inMaskNum > 0) {
            matchedvoxelnum++;
            temp->matchVoxelNum++;
            temp->matchPointNum += inMaskNum;
        }
        if (!res.second)
        {
            *(temp->totalPoint) += *voxels;
        }
        size_t matchVoxelNum=temp->matchVoxelNum;
        seed = seed.second < matchVoxelNum ?  std::make_pair(label,matchVoxelNum) :seed ;
	}
	PointCloudT::Ptr objcloud(new PointCloudT);
    if(seedSflag==1){

    }
    if(matchPointCloud.size()==0||matchedvoxelnum==0||seed.second<=0){
        return objcloud;
    }
    std::unordered_set<uint32_t> connectedGrap = getMaxConnectedgraph(
                matchPointCloud, lccp.seg_label_to_neighbor_set_map_, lccp.seg_label_to_sv_list_map_, seed.first, matchedvoxelnum);

//    for (auto i = matchPointCloud.begin(); i != matchPointCloud.end(); ++i)
//    {
//            *objcloud += *(i->second->totalPoint);//点云相加
//    }

    for (auto i = connectedGrap.begin(); i != connectedGrap.end(); ++i)
    {
        auto pointcloud = matchPointCloud[*i];
        if(pointcloud->totalPoint)
            *objcloud += *pointcloud->totalPoint;//点云相加
    }
	return objcloud;
}

