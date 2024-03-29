﻿#ifndef ORBSLAM2_SEGMENTATION_SUPERVOXEL_CLUSTERING_HPP_
#define ORBSLAM2_SEGMENTATION_SUPERVOXEL_CLUSTERING_HPP_
#include"supervoxel_clustering.h"

#include<opencv2/opencv.hpp>
#include<opencv2/imgproc/imgproc.hpp>
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
ORBSLAM2::SupervoxelClustering<PointT>::SupervoxelClustering (float voxel_resolution, float seed_resolution) :
  resolution_ (voxel_resolution),
  seed_resolution_ (seed_resolution),
  adjacency_octree_ (),
  voxel_centroid_cloud_ (),
  color_importance_ (0.1f),
  spatial_importance_ (0.4f),
  normal_importance_ (1.0f),
  use_default_transform_behaviour_ (true)
{
  adjacency_octree_.reset (new OctreeAdjacencyT (resolution_));
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
ORBSLAM2::SupervoxelClustering<PointT>::SupervoxelClustering (float voxel_resolution, float seed_resolution, bool) :
  resolution_ (voxel_resolution),
  seed_resolution_ (seed_resolution),
  adjacency_octree_ (),
  voxel_centroid_cloud_ (),
  color_importance_ (0.1f),
  spatial_importance_ (0.4f),
  normal_importance_ (1.0f),
  use_default_transform_behaviour_ (true)
{
  adjacency_octree_.reset (new OctreeAdjacencyT (resolution_));
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
ORBSLAM2::SupervoxelClustering<PointT>::~SupervoxelClustering ()
{

}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
ORBSLAM2::SupervoxelClustering<PointT>::setInputCloud (const typename pcl::PointCloud<PointT>::ConstPtr& cloud)
{
  if ( cloud->size () == 0 )
  {
    PCL_ERROR ("ORBSLAM2::SupervoxelClustering::setInputCloud Empty cloud set, doing nothing \n");
    return;
  }

  input_ = cloud;
  adjacency_octree_->setInputCloud (cloud);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
ORBSLAM2::SupervoxelClustering<PointT>::setNormalCloud (typename NormalCloudT::ConstPtr normal_cloud)
{
  if ( normal_cloud->size () == 0 )
  {
    PCL_ERROR ("ORBSLAM2::SupervoxelClustering::setNormalCloud] Empty cloud set, doing nothing \n");
    return;
  }

  input_normals_ = normal_cloud;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
ORBSLAM2::SupervoxelClustering<PointT>::extract (std::map<uint32_t,typename pcl::Supervoxel<PointT>::Ptr > &supervoxel_clusters)
{
  //timer_.reset ();
  //double t_start = timer_.getTime ();
  //std::cout << "Init compute  \n";
  bool segmentation_is_possible = initCompute ();
  if ( !segmentation_is_possible )
  {
    deinitCompute ();
    return;
  }

  //std::cout << "Preparing for segmentation \n";
  segmentation_is_possible = prepareForSegmentation ();
  if ( !segmentation_is_possible )
  {
    deinitCompute ();
    return;
  }

  //double t_prep = timer_.getTime ();
  //std::cout << "Placing Seeds" << std::endl;
  std::vector<int> seed_indices;
  selectInitialSupervoxelSeeds (seed_indices);
  //std::cout << "Creating helpers "<<std::endl;
  createSupervoxelHelpers (seed_indices);
  //double t_seeds = timer_.getTime ();


  //std::cout << "Expanding the supervoxels" << std::endl;
  int max_depth = static_cast<int> (1.8f*seed_resolution_/resolution_);
  expandSupervoxels (max_depth);
  //double t_iterate = timer_.getTime ();

  //std::cout << "Making pcl::Supervoxel structures" << std::endl;
  makeSupervoxels (supervoxel_clusters);
  //double t_supervoxels = timer_.getTime ();

 // std::cout << "--------------------------------- Timing Report --------------------------------- \n";
 // std::cout << "Time to prep (normals, neighbors, voxelization)="<<t_prep-t_start<<" ms\n";
 // std::cout << "Time to seed clusters                          ="<<t_seeds-t_prep<<" ms\n";
 // std::cout << "Time to expand clusters                        ="<<t_iterate-t_seeds<<" ms\n";
 // std::cout << "Time to create supervoxel structures           ="<<t_supervoxels-t_iterate<<" ms\n";
 // std::cout << "Total run time                                 ="<<t_supervoxels-t_start<<" ms\n";
 // std::cout << "--------------------------------------------------------------------------------- \n";

  deinitCompute ();
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
ORBSLAM2::SupervoxelClustering<PointT>::refineSupervoxels (int num_itr, std::map<uint32_t,typename pcl::Supervoxel<PointT>::Ptr > &supervoxel_clusters)
{
  if (supervoxel_helpers_.size () == 0)
  {
    PCL_ERROR ("ORBSLAM2::SupervoxelClustering::refineVoxelNormals Supervoxels not extracted, doing nothing - (Call extract first!) \n");
    return;
  }

  int max_depth = static_cast<int> (1.8f*seed_resolution_/resolution_);
  for (int i = 0; i < num_itr; ++i)
  {
    for (typename HelperListT::iterator sv_itr = supervoxel_helpers_.begin (); sv_itr != supervoxel_helpers_.end (); ++sv_itr)
    {
      sv_itr->refineNormals ();
    }

    reseedSupervoxels ();
    expandSupervoxels (max_depth);
  }


  makeSupervoxels (supervoxel_clusters);

}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


template <typename PointT> bool
ORBSLAM2::SupervoxelClustering<PointT>::prepareForSegmentation ()
{

  // if user forgot to pass point cloud or if it is empty
  if ( input_->points.size () == 0 )
    return (false);

  //Add the new cloud of data to the octree
  //std::cout << "Populating adjacency octree with new cloud \n";
  //double prep_start = timer_.getTime ();
  if ( (use_default_transform_behaviour_ && input_->isOrganized ())
       || (!use_default_transform_behaviour_ && use_single_camera_transform_))
      adjacency_octree_->setTransformFunction (boost::bind (&SupervoxelClustering::transformFunction, this, _1));

  adjacency_octree_->addPointsFromInputCloud ();
  //double prep_end = timer_.getTime ();
  //std::cout<<"Time elapsed populating octree with next frame ="<<prep_end-prep_start<<" ms\n";

  //Compute normals and insert data for centroids into data field of octree
  //double normals_start = timer_.getTime ();
  computeVoxelData ();
  //double normals_end = timer_.getTime ();
  //std::cout << "Time elapsed finding normals and pushing into octree ="<<normals_end-normals_start<<" ms\n";

  return true;
}

template <typename PointT> void
ORBSLAM2::SupervoxelClustering<PointT>::computeVoxelData ()
{
  voxel_centroid_cloud_.reset (new PointCloudT);
  voxel_centroid_cloud_->resize (adjacency_octree_->getLeafCount ());
  typename LeafVectorT::iterator leaf_itr = adjacency_octree_->begin ();
  typename PointCloudT::iterator cent_cloud_itr = voxel_centroid_cloud_->begin ();
  for (int idx = 0 ; leaf_itr != adjacency_octree_->end (); ++leaf_itr, ++cent_cloud_itr, ++idx)
  {
    VoxelData& new_voxel_data = (*leaf_itr)->getData ();
    //Add the point to the centroid cloud
    new_voxel_data.getPoint (*cent_cloud_itr);
    //voxel_centroid_cloud_->push_back(new_voxel_data.getPoint ());
    new_voxel_data.idx_ = idx;
  }

  //If normals were provided
  if (input_normals_)
  {
    //Verify that input normal cloud size is same as input cloud size
    assert (input_normals_->size () == input_->size ());
    //For every point in the input cloud, find its corresponding leaf
    typename NormalCloudT::const_iterator normal_itr = input_normals_->begin ();
    for (typename PointCloudT::const_iterator input_itr = input_->begin (); input_itr != input_->end (); ++input_itr, ++normal_itr)
    {
      //If the point is not finite we ignore it
      if ( !pcl::isFinite<PointT> (*input_itr))
        continue;
      //Otherwise look up its leaf container
      LeafContainerT* leaf = adjacency_octree_->getLeafContainerAtPoint (*input_itr);

      //Get the voxel data object
      VoxelData& voxel_data = leaf->getData ();
      //Add this normal in (we will normalize at the end)
      voxel_data.normal_ += normal_itr->getNormalVector4fMap ();
      voxel_data.curvature_ += normal_itr->curvature;
    }
    //Now iterate through the leaves and normalize
    for (leaf_itr = adjacency_octree_->begin (); leaf_itr != adjacency_octree_->end (); ++leaf_itr)
    {
      VoxelData& voxel_data = (*leaf_itr)->getData ();
      voxel_data.normal_.normalize ();
      voxel_data.owner_ = 0;
      voxel_data.distance_ = std::numeric_limits<float>::max ();
      //Get the number of points in this leaf
      int num_points = (*leaf_itr)->getPointCounter ();
      voxel_data.curvature_ /= num_points;
    }
  }
  else //Otherwise just compute the normals
  {
    for (leaf_itr = adjacency_octree_->begin (); leaf_itr != adjacency_octree_->end (); ++leaf_itr)
    {
      VoxelData& new_voxel_data = (*leaf_itr)->getData ();
      //For every point, get its neighbors, build an index vector, compute normal
      std::vector<int> indices;
      indices.reserve (81);
      //Push this point
      indices.push_back (new_voxel_data.idx_);
      for (typename LeafContainerT::const_iterator neighb_itr=(*leaf_itr)->cbegin (); neighb_itr!=(*leaf_itr)->cend (); ++neighb_itr)
      {
        VoxelData& neighb_voxel_data = (*neighb_itr)->getData ();
        //Push neighbor index
        indices.push_back (neighb_voxel_data.idx_);
        //Get neighbors neighbors, push onto cloud
        for (typename LeafContainerT::const_iterator neighb_neighb_itr=(*neighb_itr)->cbegin (); neighb_neighb_itr!=(*neighb_itr)->cend (); ++neighb_neighb_itr)
        {
          VoxelData& neighb2_voxel_data = (*neighb_neighb_itr)->getData ();
          indices.push_back (neighb2_voxel_data.idx_);
        }
      }
      //Compute normal
      pcl::computePointNormal (*voxel_centroid_cloud_, indices, new_voxel_data.normal_, new_voxel_data.curvature_);
      pcl::flipNormalTowardsViewpoint (voxel_centroid_cloud_->points[new_voxel_data.idx_], 0.0f,0.0f,0.0f, new_voxel_data.normal_);
      new_voxel_data.normal_[3] = 0.0f;
      new_voxel_data.normal_.normalize ();
      new_voxel_data.owner_ = 0;
      new_voxel_data.distance_ = std::numeric_limits<float>::max ();
    }
  }


}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
ORBSLAM2::SupervoxelClustering<PointT>::expandSupervoxels ( int depth )
{


  for (int i = 1; i < depth; ++i)
  {
      //Expand the the supervoxels by one iteration
      for (typename HelperListT::iterator sv_itr = supervoxel_helpers_.begin (); sv_itr != supervoxel_helpers_.end (); ++sv_itr)
      {
        sv_itr->expand ();
      }

      //Update the centers to reflect new centers
      for (typename HelperListT::iterator sv_itr = supervoxel_helpers_.begin (); sv_itr != supervoxel_helpers_.end (); )
      {
        if (sv_itr->size () == 0)
        {
          sv_itr = supervoxel_helpers_.erase (sv_itr);
        }
        else
        {
          sv_itr->updateCentroid ();
          ++sv_itr;
        }
      }

  }

}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
ORBSLAM2::SupervoxelClustering<PointT>::makeSupervoxels (std::map<uint32_t,typename pcl::Supervoxel<PointT>::Ptr > &supervoxel_clusters)
{
  supervoxel_clusters.clear ();
  for (typename HelperListT::iterator sv_itr = supervoxel_helpers_.begin (); sv_itr != supervoxel_helpers_.end (); ++sv_itr)
  {
    uint32_t label = sv_itr->getLabel ();
    supervoxel_clusters[label].reset (new pcl::Supervoxel<PointT>);
    sv_itr->getXYZ (supervoxel_clusters[label]->centroid_.x,supervoxel_clusters[label]->centroid_.y,supervoxel_clusters[label]->centroid_.z);
    sv_itr->getRGB (supervoxel_clusters[label]->centroid_.rgba);
    sv_itr->getNormal (supervoxel_clusters[label]->normal_);
    sv_itr->getVoxels (supervoxel_clusters[label]->voxels_);
    sv_itr->getNormals (supervoxel_clusters[label]->normals_);
  }
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
ORBSLAM2::SupervoxelClustering<PointT>::createSupervoxelHelpers (std::vector<int> &seed_indices)
{

  supervoxel_helpers_.clear ();
  for (size_t i = 0; i < seed_indices.size (); ++i)
  {
    supervoxel_helpers_.push_back (new SupervoxelHelper(i+1,this));
    //Find which leaf corresponds to this seed index
    LeafContainerT* seed_leaf = adjacency_octree_->at(seed_indices[i]);//adjacency_octree_->getLeafContainerAtPoint (seed_points[i]);
    if (seed_leaf)
    {
      supervoxel_helpers_.back ().addLeaf (seed_leaf);
    }
    else
    {
      PCL_WARN ("Could not find leaf in ORBSLAM2::SupervoxelClustering<PointT>::createSupervoxelHelpers - supervoxel will be deleted \n");
    }
  }

}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
ORBSLAM2::SupervoxelClustering<PointT>::selectInitialSupervoxelSeeds (std::vector<int> &seed_indices)
{
  //TODO THIS IS BAD - SEEDING SHOULD BE BETTER
  //TODO Switch to assigning leaves! Don't use Octree!

 // std::cout << "Size of centroid cloud="<<voxel_centroid_cloud_->size ()<<", seeding resolution="<<seed_resolution_<<"\n";
  //Initialize octree with voxel centroids
  pcl::octree::OctreePointCloudSearch <PointT> seed_octree (seed_resolution_);
  seed_octree.setInputCloud (voxel_centroid_cloud_);
  seed_octree.addPointsFromInputCloud ();
 // std::cout << "Size of octree ="<<seed_octree.getLeafCount ()<<"\n";
  std::vector<PointT, Eigen::aligned_allocator<PointT> > voxel_centers;
  int num_seeds = seed_octree.getOccupiedVoxelCenters(voxel_centers);
  //std::cout << "Number of seed points before filtering="<<voxel_centers.size ()<<std::endl;

  std::vector<int> seed_indices_orig;
  seed_indices_orig.resize (num_seeds, 0);
  seed_indices.clear ();
  std::vector<int> closest_index;
  std::vector<float> distance;
  closest_index.resize(1,0);
  distance.resize(1,0);
  if (voxel_kdtree_ == 0)
  {
    voxel_kdtree_.reset (new pcl::search::KdTree<PointT>);
    voxel_kdtree_ ->setInputCloud (voxel_centroid_cloud_);
  }

  for (int i = 0; i < num_seeds; ++i)
  {
    voxel_kdtree_->nearestKSearch (voxel_centers[i], 1, closest_index, distance);
    seed_indices_orig[i] = closest_index[0];
  }

  std::vector<int> neighbors;
  std::vector<float> sqr_distances;
  seed_indices.reserve (seed_indices_orig.size ());
  float search_radius = 0.5f*seed_resolution_;
  // This is 1/20th of the number of voxels which fit in a planar slice through search volume
  // Area of planar slice / area of voxel side. (Note: This is smaller than the value mentioned in the original paper)
  float min_points = 0.05f * (search_radius)*(search_radius) * 3.1415926536f  / (resolution_*resolution_);
  for (size_t i = 0; i < seed_indices_orig.size (); ++i)
  {
    int num = voxel_kdtree_->radiusSearch (seed_indices_orig[i], search_radius , neighbors, sqr_distances);
    int min_index = seed_indices_orig[i];
    if ( num > min_points)
    {
      seed_indices.push_back (min_index);
    }

  }
 // std::cout << "Number of seed points after filtering="<<seed_points.size ()<<std::endl;

}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
ORBSLAM2::SupervoxelClustering<PointT>::reseedSupervoxels ()
{
  //Go through each supervoxel and remove all it's leaves
  for (typename HelperListT::iterator sv_itr = supervoxel_helpers_.begin (); sv_itr != supervoxel_helpers_.end (); ++sv_itr)
  {
    sv_itr->removeAllLeaves ();
  }

  std::vector<int> closest_index;
  std::vector<float> distance;
  //Now go through each supervoxel, find voxel closest to its center, add it in
  for (typename HelperListT::iterator sv_itr = supervoxel_helpers_.begin (); sv_itr != supervoxel_helpers_.end (); ++sv_itr)
  {
    PointT point;
    sv_itr->getXYZ (point.x, point.y, point.z);
    voxel_kdtree_->nearestKSearch (point, 1, closest_index, distance);

    LeafContainerT* seed_leaf = adjacency_octree_->at (closest_index[0]);
    if (seed_leaf)
    {
      sv_itr->addLeaf (seed_leaf);
    }
    else
    {
      PCL_WARN ("Could not find leaf in ORBSLAM2::SupervoxelClustering<PointT>::reseedSupervoxels - supervoxel will be deleted \n");
    }
  }

}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
ORBSLAM2::SupervoxelClustering<PointT>::transformFunction (PointT &p)
{
  p.x /= p.z;
  p.y /= p.z;
  p.z = std::log (p.z);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> float
ORBSLAM2::SupervoxelClustering<PointT>::voxelDataDistance (const VoxelData &v1, const VoxelData &v2) const
{

  float spatial_dist = (v1.xyz_ - v2.xyz_).norm () / seed_resolution_;
  float color_dist =  (v1.rgb_ - v2.rgb_).norm () / 255.0f;
  float cos_angle_normal = 1.0f - std::abs (v1.normal_.dot (v2.normal_));
 // std::cout << "s="<<spatial_dist<<"  c="<<color_dist<<"   an="<<cos_angle_normal<<"\n";
  return  cos_angle_normal * normal_importance_ + color_dist * color_importance_+ spatial_dist * spatial_importance_;

}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////// GETTER FUNCTIONS
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
ORBSLAM2::SupervoxelClustering<PointT>::getSupervoxelAdjacencyList (VoxelAdjacencyList &adjacency_list_arg) const
{
  adjacency_list_arg.clear ();
    //Add a vertex for each label, store ids in map
  std::map <uint32_t, VoxelID> label_ID_map;
  for (typename HelperListT::const_iterator sv_itr = supervoxel_helpers_.cbegin (); sv_itr != supervoxel_helpers_.cend (); ++sv_itr)
  {
    VoxelID node_id = add_vertex (adjacency_list_arg);
    adjacency_list_arg[node_id] = (sv_itr->getLabel ());
    label_ID_map.insert (std::make_pair (sv_itr->getLabel (), node_id));
  }

  for (typename HelperListT::const_iterator sv_itr = supervoxel_helpers_.cbegin (); sv_itr != supervoxel_helpers_.cend (); ++sv_itr)
  {
    uint32_t label = sv_itr->getLabel ();
    std::set<uint32_t> neighbor_labels;
    sv_itr->getNeighborLabels (neighbor_labels);
    for (std::set<uint32_t>::iterator label_itr = neighbor_labels.begin (); label_itr != neighbor_labels.end (); ++label_itr)
    {
      bool edge_added;
      EdgeID edge;
      VoxelID u = (label_ID_map.find (label))->second;
      VoxelID v = (label_ID_map.find (*label_itr))->second;
      boost::tie (edge, edge_added) = add_edge (u,v,adjacency_list_arg);
      //Calc distance between centers, set as edge weight
      if (edge_added)
      {
        VoxelData centroid_data = (sv_itr)->getCentroid ();
        //Find the neighbhor with this label
        VoxelData neighb_centroid_data;

        for (typename HelperListT::const_iterator neighb_itr = supervoxel_helpers_.cbegin (); neighb_itr != supervoxel_helpers_.cend (); ++neighb_itr)
        {
          if (neighb_itr->getLabel () == (*label_itr))
          {
            neighb_centroid_data = neighb_itr->getCentroid ();
            break;
          }
        }

        float length = voxelDataDistance (centroid_data, neighb_centroid_data);
        adjacency_list_arg[edge] = length;
      }
    }

  }

}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
ORBSLAM2::SupervoxelClustering<PointT>::getSupervoxelAdjacency (std::multimap<uint32_t, uint32_t> &label_adjacency) const
{
  label_adjacency.clear ();
  for (typename HelperListT::const_iterator sv_itr = supervoxel_helpers_.cbegin (); sv_itr != supervoxel_helpers_.cend (); ++sv_itr)
  {
    uint32_t label = sv_itr->getLabel ();
    std::set<uint32_t> neighbor_labels;
    sv_itr->getNeighborLabels (neighbor_labels);
    for (std::set<uint32_t>::iterator label_itr = neighbor_labels.begin (); label_itr != neighbor_labels.end (); ++label_itr)
      label_adjacency.insert (std::pair<uint32_t,uint32_t> (label, *label_itr) );
    //if (neighbor_labels.size () == 0)
    //  std::cout << label<<"(size="<<sv_itr->size () << ") has "<<neighbor_labels.size () << "\n";
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> typename pcl::PointCloud<PointT>::Ptr
ORBSLAM2::SupervoxelClustering<PointT>::getVoxelCentroidCloud () const
{
  typename PointCloudT::Ptr centroid_copy (new PointCloudT);
  copyPointCloud (*voxel_centroid_cloud_, *centroid_copy);
  return centroid_copy;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> pcl::PointCloud<pcl::PointXYZL>::Ptr
ORBSLAM2::SupervoxelClustering<PointT>::getLabeledVoxelCloud () const
{
  pcl::PointCloud<pcl::PointXYZL>::Ptr labeled_voxel_cloud (new pcl::PointCloud<pcl::PointXYZL>);
  for (typename HelperListT::const_iterator sv_itr = supervoxel_helpers_.cbegin (); sv_itr != supervoxel_helpers_.cend (); ++sv_itr)
  {
    typename PointCloudT::Ptr voxels;
    sv_itr->getVoxels (voxels);
    pcl::PointCloud<pcl::PointXYZL> xyzl_copy;
    copyPointCloud (*voxels, xyzl_copy);

    pcl::PointCloud<pcl::PointXYZL>::iterator xyzl_copy_itr = xyzl_copy.begin ();
    for ( ; xyzl_copy_itr != xyzl_copy.end (); ++xyzl_copy_itr)
      xyzl_copy_itr->label = sv_itr->getLabel ();

    *labeled_voxel_cloud += xyzl_copy;
  }

  return labeled_voxel_cloud;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> pcl::PointCloud<pcl::PointXYZL>::Ptr
ORBSLAM2::SupervoxelClustering<PointT>::getLabeledCloud () const
{
  pcl::PointCloud<pcl::PointXYZL>::Ptr labeled_cloud (new pcl::PointCloud<pcl::PointXYZL>);
  pcl::copyPointCloud (*input_,*labeled_cloud);

  pcl::PointCloud <pcl::PointXYZL>::iterator i_labeled;
  typename pcl::PointCloud <PointT>::const_iterator i_input = input_->begin ();
  std::vector <int> indices;
  std::vector <float> sqr_distances;
  for (i_labeled = labeled_cloud->begin (); i_labeled != labeled_cloud->end (); ++i_labeled,++i_input)
  {
    if ( !pcl::isFinite<PointT> (*i_input))
      i_labeled->label = 0;
    else
    {
      i_labeled->label = 0;
      LeafContainerT *leaf = adjacency_octree_->getLeafContainerAtPoint (*i_input);
      VoxelData& voxel_data = leaf->getData ();
      if (voxel_data.owner_)
        i_labeled->label = voxel_data.owner_->getLabel ();

    }

  }

  return (labeled_cloud);
}

template<typename PointT>
 pcl::PointCloud<pcl::PointXYZL>::Ptr ORBSLAM2::SupervoxelClustering<PointT>::getFullLabeledWithMarginCloud(const boost::unordered_set<uint32_t> &MarginVoxellabel)
{
    pcl::PointCloud<pcl::PointXYZL>::Ptr labeled_cloud(new pcl::PointCloud<pcl::PointXYZL>);
    pcl::copyPointCloud(*input_, *labeled_cloud);

    pcl::PointCloud <pcl::PointXYZL>::iterator i_labeled;
    typename pcl::PointCloud <PointT>::const_iterator i_input = input_->begin();
    std::vector <int> indices;
    std::vector <float> sqr_distances;
    for (i_labeled = labeled_cloud->begin(); i_labeled != labeled_cloud->end(); ++i_labeled, ++i_input)
    {
        if (!pcl::isFinite<PointT>(*i_input))
            i_labeled->label = 0;
        else
        {
            i_labeled->label = 0;
            LeafContainerT *leaf = adjacency_octree_->getLeafContainerAtPoint(*i_input);
            VoxelData& voxel_data = leaf->getData();
            if (voxel_data.owner_)
            {
                if (MarginVoxellabel.find(voxel_data.owner_->getLabel()) != MarginVoxellabel.end())
                    i_labeled->label = 100;
                else
                    i_labeled->label = voxel_data.owner_->getLabel();
            }

        }

    }

    return (labeled_cloud);
}





template<>
void ORBSLAM2::SupervoxelClustering<pcl::PointXYZRGBA>::setMarginVoxel(cv::Rect rect,double gridresolution, boost::unordered_set<uint32_t> &marginvoxel_set_,float fx, float fy , float cx , float cy )
{
//    boost::unordered_set<uint32_t>  marginvoxel_set_;
    boost::unordered_set<uint32_t>  edgevoxel_set_;
    int gridwidth = std::ceil(rect.width / gridresolution);
    int gridheight = std::ceil(rect.height / gridresolution);
    std::vector<std::vector<gridData>> grid(gridheight, std::vector<gridData>(gridwidth));
    for (auto inputBegin = input_->begin(); inputBegin != input_->end(); ++inputBegin)
    {
        if (pcl::isFinite<pcl::PointXYZRGBA>(*inputBegin) )
        {
            LeafContainerT *leaf = adjacency_octree_->getLeafContainerAtPoint(*inputBegin);
            VoxelData& voxel_data = leaf->getData();
            if (voxel_data.owner_)
            {
                float gridx = (inputBegin->x*fx / inputBegin->z + cx - rect.x) / gridresolution;
                float gridy = (inputBegin->y*fy / inputBegin->z + cy - rect.y) / gridresolution;
                //if (gridx < 0 || gridy < 0)
                //	continue;

                grid[gridy][gridx].updateGrid(voxel_data.owner_);
                if (inputBegin->a > 0)
                {
                    edgevoxel_set_.insert(voxel_data.owner_->getLabel());
                }
            }
        }
    }
    //return edgevoxel_set_;
    /*cv::Mat gridimg(gridheight, gridwidth, CV_8UC1, cv::Scalar(0, 0, 0));
    for (size_t i = 0; i < grid.size(); i++)
    {
        for (size_t ii = 0; ii < grid[i].size(); ii++)
        {
            auto hh = grid[i][ii].getMaxSupervoxelHelper();
            if (hh&&edgevoxel_set_.find(hh->getLabel()) != edgevoxel_set_.end())
                gridimg.ptr<uchar>(i)[ii] = 255;
        }
    }
    cv::resize(gridimg, gridimg, rect.size(), 0, 0, cv::INTER_NEAREST);
    cv::imshow("gridimg1", gridimg);*/
    for (size_t y = 0; y < gridheight; y++)
    {
        bool one = true;
        bool two = true;
        for (size_t x = 0,bx= gridwidth-1; x < gridwidth && bx>=0 && (one || two); )
        {
            if (one) {
                SupervoxelHelper* temphelp = grid[y][x].getMaxSupervoxelHelper();
                if (temphelp && (edgevoxel_set_.find(temphelp->getLabel()) != edgevoxel_set_.end()) ) {
                    marginvoxel_set_.insert(temphelp->getLabel());
                    one = false;
                }
                if (x > gridwidth / 2) {
                    one = false;
                }
                else {
                    x++;
                }
            }
            if (two) {
                SupervoxelHelper* temphelp = grid[y][bx].getMaxSupervoxelHelper();
                if (temphelp&& (edgevoxel_set_.find(temphelp->getLabel()) != edgevoxel_set_.end())) {
                    marginvoxel_set_.insert(temphelp->getLabel());
                    two = false;
                }
                if (bx < gridwidth / 2) {
                    two = false;
                }
                else {
                    bx--;
                }
            }
        }
    }

    for (size_t x = 0; x < gridwidth; x++)
    {
        bool one = true;
        bool two = true;
        for (size_t y = 0, by = gridheight - 1; y < gridheight && by>=0 &&  (one || two) ;)
        {
            if (one){
                SupervoxelHelper* temphelp = grid[y][x].getMaxSupervoxelHelper();
                if (temphelp&& (edgevoxel_set_.find(temphelp->getLabel()) != edgevoxel_set_.end()) ) {
                    marginvoxel_set_.insert(temphelp->getLabel());
                    one = false;
                }
                if (y > gridheight / 2) {
                    one = false;
                }
                else {
                    y++;
                }
            }
            if (two) {
                SupervoxelHelper* temphelp = grid[by][x].getMaxSupervoxelHelper();
                if (temphelp&& (edgevoxel_set_.find(temphelp->getLabel()) != edgevoxel_set_.end()) ) {
                    marginvoxel_set_.insert(temphelp->getLabel());
                    two = false;
                }
                if (by < gridheight / 2) {
                    two = false;
                }
                else {
                    by--;
                }
            }
        }
    }

}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> pcl::PointCloud<pcl::PointNormal>::Ptr
ORBSLAM2::SupervoxelClustering<PointT>::makeSupervoxelNormalCloud (std::map<uint32_t,typename pcl::Supervoxel<PointT>::Ptr > &supervoxel_clusters)
{
  pcl::PointCloud<pcl::PointNormal>::Ptr normal_cloud (new pcl::PointCloud<pcl::PointNormal>);
  normal_cloud->resize (supervoxel_clusters.size ());
  typename std::map <uint32_t, typename pcl::Supervoxel<PointT>::Ptr>::iterator sv_itr,sv_itr_end;
  sv_itr = supervoxel_clusters.begin ();
  sv_itr_end = supervoxel_clusters.end ();
  pcl::PointCloud<pcl::PointNormal>::iterator normal_cloud_itr = normal_cloud->begin ();
  for ( ; sv_itr != sv_itr_end; ++sv_itr, ++normal_cloud_itr)
  {
    (sv_itr->second)->getCentroidPointNormal (*normal_cloud_itr);
  }
  return normal_cloud;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> float
ORBSLAM2::SupervoxelClustering<PointT>::getVoxelResolution () const
{
  return (resolution_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
ORBSLAM2::SupervoxelClustering<PointT>::setVoxelResolution (float resolution)
{
  resolution_ = resolution;

}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> float
ORBSLAM2::SupervoxelClustering<PointT>::getSeedResolution () const
{
  return (seed_resolution_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
ORBSLAM2::SupervoxelClustering<PointT>::setSeedResolution (float seed_resolution)
{
  seed_resolution_ = seed_resolution;
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
ORBSLAM2::SupervoxelClustering<PointT>::setColorImportance (float val)
{
  color_importance_ = val;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
ORBSLAM2::SupervoxelClustering<PointT>::setSpatialImportance (float val)
{
  spatial_importance_ = val;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
ORBSLAM2::SupervoxelClustering<PointT>::setNormalImportance (float val)
{
  normal_importance_ = val;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
ORBSLAM2::SupervoxelClustering<PointT>::setUseSingleCameraTransform (bool val)
{
  use_default_transform_behaviour_ = false;
  use_single_camera_transform_ = val;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> int
ORBSLAM2::SupervoxelClustering<PointT>::getMaxLabel () const
{
  int max_label = 0;
  for (typename HelperListT::const_iterator sv_itr = supervoxel_helpers_.cbegin (); sv_itr != supervoxel_helpers_.cend (); ++sv_itr)
  {
    int temp = sv_itr->getLabel ();
    if (temp > max_label)
      max_label = temp;
  }
  return max_label;
}

namespace pcl
{
  namespace octree
  {
    //Explicit overloads for RGB types
    template<>
    void
    pcl::octree::OctreePointCloudAdjacencyContainer<pcl::PointXYZRGB,ORBSLAM2::SupervoxelClustering<pcl::PointXYZRGB>::VoxelData>::addPoint (const pcl::PointXYZRGB &new_point);

    template<>
    void
    pcl::octree::OctreePointCloudAdjacencyContainer<pcl::PointXYZRGBA,ORBSLAM2::SupervoxelClustering<pcl::PointXYZRGBA>::VoxelData>::addPoint (const pcl::PointXYZRGBA &new_point);

    //Explicit overloads for RGB types
    template<> void
    pcl::octree::OctreePointCloudAdjacencyContainer<pcl::PointXYZRGB,ORBSLAM2::SupervoxelClustering<pcl::PointXYZRGB>::VoxelData>::computeData ();

    template<> void
    pcl::octree::OctreePointCloudAdjacencyContainer<pcl::PointXYZRGBA,ORBSLAM2::SupervoxelClustering<pcl::PointXYZRGBA>::VoxelData>::computeData ();

    //Explicit overloads for XYZ types
    template<>
    void
    pcl::octree::OctreePointCloudAdjacencyContainer<pcl::PointXYZ,ORBSLAM2::SupervoxelClustering<pcl::PointXYZ>::VoxelData>::addPoint (const pcl::PointXYZ &new_point);

    template<> void
    pcl::octree::OctreePointCloudAdjacencyContainer<pcl::PointXYZ,ORBSLAM2::SupervoxelClustering<pcl::PointXYZ>::VoxelData>::computeData ();
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
namespace ORBSLAM2
{

    template<> void
        ORBSLAM2::SupervoxelClustering<pcl::PointXYZRGB>::VoxelData::getPoint(pcl::PointXYZRGB &point_arg) const;

    template<> void
        ORBSLAM2::SupervoxelClustering<pcl::PointXYZRGBA>::VoxelData::getPoint(pcl::PointXYZRGBA &point_arg) const;

    template<>
        void ORBSLAM2::SupervoxelClustering<pcl::PointXYZRGBA>::setMarginVoxelAdaptive(cv::Rect rect,boost::unordered_set<uint32_t> &marginvoxel_set_, float fx, float fy, float cx, float cy);

    template<typename PointT> void
        ORBSLAM2::SupervoxelClustering<PointT>::VoxelData::getPoint(PointT &point_arg) const
    {
        //XYZ is required or this doesn't make much sense...
        point_arg.x = xyz_[0];
        point_arg.y = xyz_[1];
        point_arg.z = xyz_[2];

    }
    template < > void
    ORBSLAM2::SupervoxelClustering<pcl::PointXYZRGBA>::VoxelData::getNormal(pcl::Normal &normal_arg) const{
    }
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    template <typename PointT> void
        ORBSLAM2::SupervoxelClustering<PointT>::VoxelData::getNormal(pcl::Normal &normal_arg) const
    {
        normal_arg.normal_x = normal_[0];
        normal_arg.normal_y = normal_[1];
        normal_arg.normal_z = normal_[2];
        normal_arg.curvature = curvature_;
    }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
ORBSLAM2::SupervoxelClustering<PointT>::SupervoxelHelper::addLeaf (LeafContainerT* leaf_arg)
{
  leaves_.insert (leaf_arg);
  VoxelData& voxel_data = leaf_arg->getData ();
  voxel_data.owner_ = this;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
ORBSLAM2::SupervoxelClustering<PointT>::SupervoxelHelper::removeLeaf (LeafContainerT* leaf_arg)
{
  leaves_.erase (leaf_arg);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
ORBSLAM2::SupervoxelClustering<PointT>::SupervoxelHelper::removeAllLeaves ()
{
  typename SupervoxelHelper::iterator leaf_itr;
  for (leaf_itr = leaves_.begin (); leaf_itr != leaves_.end (); ++leaf_itr)
  {
    VoxelData& voxel = ((*leaf_itr)->getData ());
    voxel.owner_ = 0;
    voxel.distance_ = std::numeric_limits<float>::max ();
  }
  leaves_.clear ();
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
ORBSLAM2::SupervoxelClustering<PointT>::SupervoxelHelper::expand ()
{
  //std::cout << "Expanding sv "<<label_<<", owns "<<leaves_.size ()<<" voxels\n";
  //Buffer of new neighbors - initial size is just a guess of most possible
  std::vector<LeafContainerT*> new_owned;
  new_owned.reserve (leaves_.size () * 9);
  //For each leaf belonging to this supervoxel
  typename SupervoxelHelper::iterator leaf_itr;
  for (leaf_itr = leaves_.begin (); leaf_itr != leaves_.end (); ++leaf_itr)
  {
    //for each neighbor of the leaf
    for (typename LeafContainerT::const_iterator neighb_itr=(*leaf_itr)->cbegin (); neighb_itr!=(*leaf_itr)->cend (); ++neighb_itr)
    {
      //Get a reference to the data contained in the leaf
      VoxelData& neighbor_voxel = ((*neighb_itr)->getData ());
      //TODO this is a shortcut, really we should always recompute distance
      if(neighbor_voxel.owner_ == this)
        continue;
      //Compute distance to the neighbor
      float dist = parent_->voxelDataDistance (centroid_, neighbor_voxel);
      //If distance is less than previous, we remove it from its owner's list
      //and change the owner to this and distance (we *steal* it!)
      if (dist < neighbor_voxel.distance_)
      {
        neighbor_voxel.distance_ = dist;
        if (neighbor_voxel.owner_ != this)
        {
          if (neighbor_voxel.owner_)
            (neighbor_voxel.owner_)->removeLeaf(*neighb_itr);
          neighbor_voxel.owner_ = this;
          new_owned.push_back (*neighb_itr);
        }
      }
    }
  }
  //Push all new owned onto the owned leaf set
  typename std::vector<LeafContainerT*>::iterator new_owned_itr;
  for (new_owned_itr=new_owned.begin (); new_owned_itr!=new_owned.end (); ++new_owned_itr)
  {
    leaves_.insert (*new_owned_itr);
  }

}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
ORBSLAM2::SupervoxelClustering<PointT>::SupervoxelHelper::refineNormals ()
{
  typename SupervoxelHelper::iterator leaf_itr;
  //For each leaf belonging to this supervoxel, get its neighbors, build an index vector, compute normal
  for (leaf_itr = leaves_.begin (); leaf_itr != leaves_.end (); ++leaf_itr)
  {
    VoxelData& voxel_data = (*leaf_itr)->getData ();
    std::vector<int> indices;
    indices.reserve (81);
    //Push this point
    indices.push_back (voxel_data.idx_);
    for (typename LeafContainerT::const_iterator neighb_itr=(*leaf_itr)->cbegin (); neighb_itr!=(*leaf_itr)->cend (); ++neighb_itr)
    {
      //Get a reference to the data contained in the leaf
      VoxelData& neighbor_voxel_data = ((*neighb_itr)->getData ());
      //If the neighbor is in this supervoxel, use it
      if (neighbor_voxel_data.owner_ == this)
      {
        indices.push_back (neighbor_voxel_data.idx_);
        //Also check its neighbors
        for (typename LeafContainerT::const_iterator neighb_neighb_itr=(*neighb_itr)->cbegin (); neighb_neighb_itr!=(*neighb_itr)->cend (); ++neighb_neighb_itr)
        {
          VoxelData& neighb_neighb_voxel_data = (*neighb_neighb_itr)->getData ();
          if (neighb_neighb_voxel_data.owner_ == this)
            indices.push_back (neighb_neighb_voxel_data.idx_);
        }


      }
    }
    //Compute normal
    pcl::computePointNormal (*parent_->voxel_centroid_cloud_, indices, voxel_data.normal_, voxel_data.curvature_);
    pcl::flipNormalTowardsViewpoint (parent_->voxel_centroid_cloud_->points[voxel_data.idx_], 0.0f,0.0f,0.0f, voxel_data.normal_);
    voxel_data.normal_[3] = 0.0f;
    voxel_data.normal_.normalize ();
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
ORBSLAM2::SupervoxelClustering<PointT>::SupervoxelHelper::updateCentroid ()
{
  centroid_.normal_ = Eigen::Vector4f::Zero ();
  centroid_.xyz_ = Eigen::Vector3f::Zero ();
  centroid_.rgb_ = Eigen::Vector3f::Zero ();
  typename SupervoxelHelper::iterator leaf_itr = leaves_.begin ();
  for ( ; leaf_itr!= leaves_.end (); ++leaf_itr)
  {
    const VoxelData& leaf_data = (*leaf_itr)->getData ();
    centroid_.normal_ += leaf_data.normal_;
    centroid_.xyz_ += leaf_data.xyz_;
    centroid_.rgb_ += leaf_data.rgb_;
  }
  centroid_.normal_.normalize ();
  centroid_.xyz_ /= static_cast<float> (leaves_.size ());
  centroid_.rgb_ /= static_cast<float> (leaves_.size ());

}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
ORBSLAM2::SupervoxelClustering<PointT>::SupervoxelHelper::getVoxels (typename pcl::PointCloud<PointT>::Ptr &voxels) const
{
  voxels.reset (new pcl::PointCloud<PointT>);
  voxels->clear ();
  voxels->resize (leaves_.size ());
  typename pcl::PointCloud<PointT>::iterator voxel_itr = voxels->begin ();
  typename SupervoxelHelper::const_iterator leaf_itr;
  for ( leaf_itr = leaves_.begin (); leaf_itr != leaves_.end (); ++leaf_itr, ++voxel_itr)
  {
    const VoxelData& leaf_data = (*leaf_itr)->getData ();
    leaf_data.getPoint (*voxel_itr);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
ORBSLAM2::SupervoxelClustering<PointT>::SupervoxelHelper::getNormals (typename pcl::PointCloud<pcl::Normal>::Ptr &normals) const
{
  normals.reset (new pcl::PointCloud<pcl::Normal>);
  normals->clear ();
  normals->resize (leaves_.size ());
  typename SupervoxelHelper::const_iterator leaf_itr;
  typename pcl::PointCloud<pcl::Normal>::iterator normal_itr = normals->begin ();
  for (leaf_itr = leaves_.begin (); leaf_itr != leaves_.end (); ++leaf_itr, ++normal_itr)
  {
    const VoxelData& leaf_data = (*leaf_itr)->getData ();
    leaf_data.getNormal (*normal_itr);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
ORBSLAM2::SupervoxelClustering<PointT>::SupervoxelHelper::getNeighborLabels (std::set<uint32_t> &neighbor_labels) const
{
  neighbor_labels.clear ();
  //For each leaf belonging to this supervoxel
  typename SupervoxelHelper::const_iterator leaf_itr;
  for (leaf_itr = leaves_.begin (); leaf_itr != leaves_.end (); ++leaf_itr)
  {
    //for each neighbor of the leaf
    for (typename LeafContainerT::const_iterator neighb_itr=(*leaf_itr)->cbegin (); neighb_itr!=(*leaf_itr)->cend (); ++neighb_itr)
    {
      //Get a reference to the data contained in the leaf
      VoxelData& neighbor_voxel = ((*neighb_itr)->getData ());
      //If it has an owner, and it's not us - get it's owner's label insert into set
      if (neighbor_voxel.owner_ != this && neighbor_voxel.owner_)
      {
        neighbor_labels.insert (neighbor_voxel.owner_->getLabel ());
      }
    }
  }
}


#endif    // PCL_SUPERVOXEL_CLUSTERING_HPP_

