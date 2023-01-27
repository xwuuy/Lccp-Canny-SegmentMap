
#define PCL_NO_PRECOMPILE

#include <pcl/point_types.h>
#include <pcl/impl/instantiate.hpp>
#include "supervoxel_clustering.hpp"
#include <pcl/octree/impl/octree_pointcloud_adjacency.hpp>

namespace pcl
{ 
  namespace octree
  {
    //Explicit overloads for RGB types
    template<>
    void
    pcl::octree::OctreePointCloudAdjacencyContainer<pcl::PointXYZRGB,ORBSLAM2::SupervoxelClustering<pcl::PointXYZRGB>::VoxelData>::addPoint (const pcl::PointXYZRGB &new_point)
    {
      ++num_points_;
      //Same as before here
      data_.xyz_[0] += new_point.x;
      data_.xyz_[1] += new_point.y;
      data_.xyz_[2] += new_point.z;
      //Separate sums for r,g,b since we can't sum in uchars
      data_.rgb_[0] += static_cast<float> (new_point.r); 
      data_.rgb_[1] += static_cast<float> (new_point.g); 
      data_.rgb_[2] += static_cast<float> (new_point.b); 
    }
    
    template<>
    void
    pcl::octree::OctreePointCloudAdjacencyContainer<pcl::PointXYZRGBA,ORBSLAM2::SupervoxelClustering<pcl::PointXYZRGBA>::VoxelData>::addPoint (const pcl::PointXYZRGBA &new_point)
    {
      ++num_points_;
      //Same as before here
      data_.xyz_[0] += new_point.x;
      data_.xyz_[1] += new_point.y;
      data_.xyz_[2] += new_point.z;
	  data_.a |= new_point.a;
      //Separate sums for r,g,b since we can't sum in uchars
      data_.rgb_[0] += static_cast<float> (new_point.r); 
      data_.rgb_[1] += static_cast<float> (new_point.g); 
      data_.rgb_[2] += static_cast<float> (new_point.b); 
    }
    
    
    
    //Explicit overloads for RGB types
    template<> void
    pcl::octree::OctreePointCloudAdjacencyContainer<pcl::PointXYZRGB,ORBSLAM2::SupervoxelClustering<pcl::PointXYZRGB>::VoxelData>::computeData ()
    {
      data_.rgb_[0] /= (static_cast<float> (num_points_));
      data_.rgb_[1] /= (static_cast<float> (num_points_));
      data_.rgb_[2] /= (static_cast<float> (num_points_));
      data_.xyz_[0] /= (static_cast<float> (num_points_));
      data_.xyz_[1] /= (static_cast<float> (num_points_));
      data_.xyz_[2] /= (static_cast<float> (num_points_));    
    }
    
    template<> void
    pcl::octree::OctreePointCloudAdjacencyContainer<pcl::PointXYZRGBA,ORBSLAM2::SupervoxelClustering<pcl::PointXYZRGBA>::VoxelData>::computeData ()
    {
      data_.rgb_[0] /= (static_cast<float> (num_points_));
      data_.rgb_[1] /= (static_cast<float> (num_points_));
      data_.rgb_[2] /= (static_cast<float> (num_points_));
      data_.xyz_[0] /= (static_cast<float> (num_points_));
      data_.xyz_[1] /= (static_cast<float> (num_points_));
      data_.xyz_[2] /= (static_cast<float> (num_points_));
	  data_.pointnum = num_points_;
    }
    
    //Explicit overloads for XYZ types
    template<>
    void
    pcl::octree::OctreePointCloudAdjacencyContainer<pcl::PointXYZ,ORBSLAM2::SupervoxelClustering<pcl::PointXYZ>::VoxelData>::addPoint (const pcl::PointXYZ &new_point)
    {
      ++num_points_;
      //Same as before here
      data_.xyz_[0] += new_point.x;
      data_.xyz_[1] += new_point.y;
      data_.xyz_[2] += new_point.z;
    }
    
    template<> void
    pcl::octree::OctreePointCloudAdjacencyContainer<pcl::PointXYZ,ORBSLAM2::SupervoxelClustering<pcl::PointXYZ>::VoxelData>::computeData ()
    {
      data_.xyz_[0] /= (static_cast<float> (num_points_));
      data_.xyz_[1] /= (static_cast<float> (num_points_));
      data_.xyz_[2] /= (static_cast<float> (num_points_));
    }
    
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
namespace ORBSLAM2
{
  template<> void
  ORBSLAM2::SupervoxelClustering<pcl::PointXYZRGB>::VoxelData::getPoint (pcl::PointXYZRGB &point_arg) const
  {
    point_arg.rgba = static_cast<uint32_t>(rgb_[0]) << 16 |
    static_cast<uint32_t>(rgb_[1]) << 8 |
    static_cast<uint32_t>(rgb_[2]);
    point_arg.x = xyz_[0];
    point_arg.y = xyz_[1];
    point_arg.z = xyz_[2];
  }
  
    template<> void
    ORBSLAM2::SupervoxelClustering<pcl::PointXYZRGBA>::VoxelData::getPoint (pcl::PointXYZRGBA &point_arg ) const
    {
        point_arg.rgba = static_cast<uint32_t>(rgb_[0]) << 16 |
                         static_cast<uint32_t>(rgb_[1]) << 8 |
                         static_cast<uint32_t>(rgb_[2]);
        point_arg.a = a;
        point_arg.x = xyz_[0];
        point_arg.y = xyz_[1];
        point_arg.z = xyz_[2];
    }

template<>
boost::unordered_set<uint32_t> ORBSLAM2::SupervoxelClustering<pcl::PointXYZRGBA>::setMarginVoxelAdaptive(cv::Rect rect, float fx, float fy, float cx, float cy)
 {
     double gridresolution = std::round(fx*resolution_);//根据体素分辨率自动计算栅格分辨率
     boost::unordered_set<uint32_t>  marginvoxel_set_;
     boost::unordered_set<uint32_t>  edgevoxel_set_;
     int gridwidth = std::ceil(rect.width / gridresolution);
     int gridheight = std::ceil(rect.height / gridresolution);
     std::vector<std::vector<gridData>> grid(gridheight, std::vector<gridData>(gridwidth));
     for (typename HelperListT::const_iterator sv_itr = supervoxel_helpers_.cbegin(); sv_itr != supervoxel_helpers_.cend(); ++sv_itr)
     {
         typename SupervoxelHelper::const_iterator leaf_itr;
         for (leaf_itr = sv_itr->leaves_.begin(); leaf_itr != sv_itr->leaves_.end(); ++leaf_itr )
         {
             const VoxelData& leaf_data = (*leaf_itr)->getData();
             pcl::PointXYZRGBA p;
             leaf_data.getPoint(p);
             int gridx = std::round ((p.x*fx / p.z + cx - rect.x) / gridresolution) ;
             int gridy = std::round ((p.y*fy / p.z + cy - rect.y) / gridresolution);
             if(gridx<gridwidth&&gridy<gridheight)
                 if(gridx>=0&&gridy>=0){
                     grid[gridy][gridx].updateGrid(leaf_data.owner_);
                     if (p.a > 0)
                     {
                         edgevoxel_set_.insert(leaf_data.owner_->getLabel());
                     }
                 }
         }
     }
//     enum ori{
//         top,
//         bottom,
//         left,
//         right
//     };
//     std::vector<std::pair<size_t,ori> >  index;
     for (size_t y = 0; y < gridheight; y++)
     {
         bool left = true;
         bool right = true;
         for (size_t x = 0, bx = gridwidth - 1; x < gridwidth && bx >= 0 && (left || right); )
         {
             if (left) {//最左
                 SupervoxelHelper* temphelp = grid[y][x].getMaxEdgeSupervoxelHelper(edgevoxel_set_);
                 if (temphelp) {
                     marginvoxel_set_.insert(temphelp->getLabel());
                     left = false;
                 }
                 if (x > gridwidth / 2) {

                     left = false;
                 }
                 else {
                     x++;
                 }
             }
             if (right) {//最右
                 SupervoxelHelper* temphelp = grid[y][bx].getMaxEdgeSupervoxelHelper(edgevoxel_set_);
                 if (temphelp) {
                     marginvoxel_set_.insert(temphelp->getLabel());
                     right = false;
                 }
                 if (bx < gridwidth / 2) {
                     right = false;
                 }
                 else {
                     bx--;
                 }
             }
         }
     }

     for (size_t x = 0; x < gridwidth; x++)
     {
         bool top = true;
         bool bottom = true;
         for (size_t y = 0, by = gridheight - 1; y < gridheight && by >= 0 && (top || bottom);)
         {
             if (top) {//最上
                 SupervoxelHelper* temphelp = grid[y][x].getMaxEdgeSupervoxelHelper(edgevoxel_set_);
                 if (temphelp) {
                     marginvoxel_set_.insert(temphelp->getLabel());
                     top = false;
                 }
                 if (y > gridheight / 2) {
                     top = false;
                 }
                 else {
                     y++;
                 }
             }
             if (bottom) {//最下
                 SupervoxelHelper* temphelp = grid[by][x].getMaxEdgeSupervoxelHelper(edgevoxel_set_);
                 if (temphelp) {
                     marginvoxel_set_.insert(temphelp->getLabel());
                     bottom = false;
                 }
                 if (by < gridheight / 2) {
                     bottom = false;
                 }
                 else {
                     by--;
                 }
             }
         }
     }
     return marginvoxel_set_;
 }

}

typedef ORBSLAM2::SupervoxelClustering<pcl::PointXYZ>::VoxelData VoxelDataT;
typedef ORBSLAM2::SupervoxelClustering<pcl::PointXYZRGB>::VoxelData VoxelDataRGBT;
typedef ORBSLAM2::SupervoxelClustering<pcl::PointXYZRGBA>::VoxelData VoxelDataRGBAT;

typedef pcl::octree::OctreePointCloudAdjacencyContainer<pcl::PointXYZ, VoxelDataT> AdjacencyContainerT;
typedef pcl::octree::OctreePointCloudAdjacencyContainer<pcl::PointXYZRGB, VoxelDataRGBT> AdjacencyContainerRGBT;
typedef pcl::octree::OctreePointCloudAdjacencyContainer<pcl::PointXYZRGBA, VoxelDataRGBAT> AdjacencyContainerRGBAT;

template class ORBSLAM2::SupervoxelClustering<pcl::PointXYZ>;
template class ORBSLAM2::SupervoxelClustering<pcl::PointXYZRGB>;
template class ORBSLAM2::SupervoxelClustering<pcl::PointXYZRGBA>;

template class pcl::octree::OctreePointCloudAdjacencyContainer<pcl::PointXYZ, VoxelDataT>;
template class pcl::octree::OctreePointCloudAdjacencyContainer<pcl::PointXYZRGB, VoxelDataRGBT>;
template class pcl::octree::OctreePointCloudAdjacencyContainer<pcl::PointXYZRGBA, VoxelDataRGBAT>;

template class pcl::octree::OctreePointCloudAdjacency<pcl::PointXYZ, AdjacencyContainerT>;
template class pcl::octree::OctreePointCloudAdjacency<pcl::PointXYZRGB, AdjacencyContainerRGBT>;
template class pcl::octree::OctreePointCloudAdjacency<pcl::PointXYZRGBA, AdjacencyContainerRGBAT>;
