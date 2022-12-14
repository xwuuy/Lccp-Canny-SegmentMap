/*
* Software License Agreement (BSD License)
*
*  Point Cloud Library (PCL) - www.pointclouds.org
*  Copyright (c) 2014-, Open Perception, Inc.
*
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the copyright holder(s) nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
*/

// Stdlib
#include <cstdlib>
#include <cmath>
#include <climits>
#include <thread>

#include <boost/format.hpp>


// PCL input/output
#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>

//PCL other
#include <pcl/filters/passthrough.h>
#include "Thirdpartylib\supervoxel_clustering.h"

// The segmentation class this example is for
#include "Thirdpartylib\lccp_segmentation.h"
#include "MarginVoxelLccp.hpp"
// VTK
#include <vtkImageReader2Factory.h>
#include <vtkImageReader2.h>
#include <vtkImageData.h>
#include <vtkImageFlip.h>
#include <vtkPolyLine.h>

#include "SegImageRead.hpp"

using namespace std::chrono_literals;

/// *****  Type Definitions ***** ///

using PointT = pcl::PointXYZRGBA;  // The point type used for input
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointNormal PointNT;
typedef pcl::PointCloud<PointNT> PointNCloudT;
typedef pcl::PointXYZL PointLT;
typedef pcl::PointCloud<PointLT> PointLCloudT;
typedef pcl::Normal NormalT;
typedef pcl::PointCloud<NormalT> NormalCloudT;

using SuperVoxelAdjacencyList = pcl::LCCPSegmentation<PointT>::SupervoxelAdjacencyList;

/// Callback and variables

bool show_normals = false, normals_changed = false;
bool show_adjacency = false;
bool show_supervoxels = false;
bool show_help = true;
float normals_scale;

/** \brief Callback for setting options in the visualizer via keyboard.
*  \param[in] event_arg Registered keyboard event  */
void
keyboardEventOccurred(const pcl::visualization::KeyboardEvent& event_arg,
	void*)
{
	int key = event_arg.getKeyCode();

	if (event_arg.keyUp())
		switch (key)
		{
		case (int) '1':
			show_normals = !show_normals;
			normals_changed = true;
			break;
		default:
			break;
		}
}

/// *****  Prototypes helper functions***** ///

/** \brief Displays info text in the specified PCLVisualizer
*  \param[in] viewer_arg The PCLVisualizer to modify  */
void
printText(pcl::visualization::PCLVisualizer::Ptr viewer_arg);

/** \brief Removes info text in the specified PCLVisualizer
*  \param[in] viewer_arg The PCLVisualizer to modify  */
void
removeText(pcl::visualization::PCLVisualizer::Ptr viewer_arg);


/// ---- main ---- ///
int
main(int argc,
	char ** argv)
{
	if (argc < 2)  /// Print Info
	{
		pcl::console::print_info(
			\
			"\n\
-- pcl::LCCPSegmentation example -- :\n\
\n\
Syntax: %s input.pcd  [Options] \n\
\n\
Output:\n\
  -o <outname> \n\
          Write segmented point cloud to disk (Type XYZL). If this option is specified without giving a name, the <outputname> defaults to <inputfilename>_out.pcd.\n\
          The content of the file can be changed with the -add and -bin flags\n\
  -novis  Disable visualization\n\
Output options:\n\
  -add    Instead of XYZL, append a label field to the input point cloud which holds the segmentation results (<input_cloud_type>+L)\n\
          If a label field already exists in the input point cloud it will be overwritten by the segmentation\n\
  -bin    Save a binary pcd-file instead of an ascii file \n\
  -so     Additionally write the colored supervoxel image to <outfilename>_svcloud.pcd\n\
  \n\
Supervoxel Parameters: \n\
  -v <voxel resolution> \n\
  -s <seed resolution> \n\
  -c <color weight> \n\
  -z <spatial weight> \n\
  -n <normal_weight> \n\
  -tvoxel - Use single-camera-transform for voxels (Depth-Dependent-Voxel-Grid)\n\
  -refine - Use supervoxel refinement\n\
  -nonormals - Ignore the normals from the input pcd file\n\
  \n\
LCCPSegmentation Parameters: \n\
  -ct <concavity tolerance angle> - Angle threshold for concave edges to be treated as convex. \n\
  -st <smoothness threshold> - Invalidate steps. Value from the interval [0,1], where 0 is the strictest and 1 equals 'no smoothness check' \n\
  -ec - Use extended (less local) convexity check\n\
  -sc - Use sanity criterion to invalidate singular connected patches\n\
  -smooth <mininmal segment size>  - Merge small segments which have fewer points than minimal segment size\n\
    \n",
			argv[0]);
		getchar(); return (1);
	}

	/// -----------------------------------|  Preparations  |-----------------------------------

	bool sv_output_specified = pcl::console::find_switch(argc, argv, "-so");
	bool show_visualization = (!pcl::console::find_switch(argc, argv, "-novis"));
	bool ignore_provided_normals = pcl::console::find_switch(argc, argv, "-nonormals");
	bool add_label_field = pcl::console::find_switch(argc, argv, "-add");
	bool save_binary_pcd = pcl::console::find_switch(argc, argv, "-bin");

	/// Create variables needed for preparations
	std::string outputname;
	pcl::PointCloud<PointT>::Ptr input_cloud_ptr(new pcl::PointCloud<PointT>);
	pcl::PointCloud<pcl::Normal>::Ptr input_normals_ptr(new pcl::PointCloud<pcl::Normal>);
	bool has_normals = false;

	/// Get pcd path from command line
	bool pcdfile_bool = pcl::console::find_switch(argc, argv, "-p");
	std::string pcd_filename;
	cv::Rect rect;
	cv::Mat objmask;
	PointCloudT::Ptr objrgbaEdgeCloud = boost::shared_ptr<PointCloudT>(new PointCloudT);
	PointCloudT::Ptr objRGBCloud;
	if (pcdfile_bool)
		pcl::console::parse(argc, argv, "-p", pcd_filename);
	else {
		std::string rgb_num("1305031928.969097");
		bool rgb_file_specified = pcl::console::find_switch(argc, argv, "-num");
		if (rgb_file_specified)
			pcl::console::parse(argc, argv, "-num", rgb_num);
		vector<double>  vstrImageFilenamesRGB, vstrImageFilenamesD, vTimestamps;
		DebugObj::LoadImages("C:/Users/16694/source/repos/Project1/rgbd_dataset_freiburg1_room/associate.txt", vstrImageFilenamesRGB, vstrImageFilenamesD, vTimestamps);
		vector<Eigen::Matrix4d> vFrameT;
		DebugObj::LoadTrace("C:/Users/16694/source/repos/Project1/rgbd_dataset_freiburg1_room/groundtruth.txt", vFrameT);
		vector<vector<ObjInfo*>> vinfos;
		cv::Mat  mask, object_mask, rgb, depth;
		stringstream ss;
		ss << rgb_num;
		double tempframe;
		ss >> tempframe;
		vector<double>::iterator t = find(vstrImageFilenamesRGB.begin(), vstrImageFilenamesRGB.end(), tempframe);
		int d = distance(vstrImageFilenamesRGB.begin(), t);
		double dt = vstrImageFilenamesD.at(d);
		std::string depth_num;
		stringstream sss;
		sss << std::fixed << std::setprecision(6) << dt;
		sss >> depth_num;
		vector<ObjInfo*> vinfotemp;
		DebugObj::imgread(rgb_num, depth_num, mask, object_mask, rgb, depth, vinfotemp);
		bool index_specified = pcl::console::find_switch(argc, argv, "-index");
		int index = 0;
		if (index_specified)
			pcl::console::parse(argc, argv, "-index", index);
		auto info = vinfotemp[index];
		input_cloud_ptr = info->objrgbaCloud;
		rect = info->ojbROI;
		objrgbaEdgeCloud = info->objrgbaEdgeCloud;
		objmask = info->objMat;
		objRGBCloud = info->objMaskrgbaCloud;
		cv::imshow("edge", info->objedge);
		cv::imshow("rgb", info->Rectrgb);
		cv::imshow("objmask", info->objMat);
	}
	PCL_INFO("Loading pointcloud\n");

	
	pcl::PCLPointCloud2 input_pointcloud2;
	
	PCL_INFO("Done making cloud\n");

	///  Create outputname if not given
	bool output_specified = pcl::console::find_switch(argc, argv, "-o");
	if (output_specified)
	{
		pcl::console::parse(argc, argv, "-o", outputname);

		// If no filename is given, get output filename from inputname (strip separators and file extension)
		if (outputname.empty() || (outputname.at(0) == '-'))
		{
			outputname = pcd_filename;
			std::size_t sep = outputname.find_last_of('/');
			if (sep != std::string::npos)
				outputname = outputname.substr(sep + 1, outputname.size() - sep - 1);

			std::size_t dot = outputname.find_last_of('.');
			if (dot != std::string::npos)
				outputname = outputname.substr(0, dot);
		}
	}

	/// -----------------------------------|  Main Computation  |-----------------------------------

	///  Default values of parameters before parsing
	// Supervoxel Stuff
	float voxel_resolution = 0.004f;
	float seed_resolution = 0.08f;
	float color_importance = 0.4f;
	float spatial_importance = 0.4f;
	float normal_importance = 0.6f;
	bool use_single_cam_transform = false;
	bool use_supervoxel_refinement = false;

	// LCCPSegmentation Stuff
	float concavity_tolerance_threshold = 30;
	float smoothness_threshold = 0.01;
	std::uint32_t min_segment_size = 0;
	bool use_extended_convexity = false;
	bool use_sanity_criterion = false;

	///  Parse Arguments needed for computation
	//Supervoxel Stuff
	use_single_cam_transform = pcl::console::find_switch(argc, argv, "-tvoxel");
	use_supervoxel_refinement = pcl::console::find_switch(argc, argv, "-refine");

	pcl::console::parse(argc, argv, "-v", voxel_resolution);
	pcl::console::parse(argc, argv, "-s", seed_resolution);
	pcl::console::parse(argc, argv, "-c", color_importance);
	pcl::console::parse(argc, argv, "-z", spatial_importance);
	pcl::console::parse(argc, argv, "-n", normal_importance);

	normals_scale = seed_resolution / 2.0;

	// Segmentation Stuff
	pcl::console::parse(argc, argv, "-ct", concavity_tolerance_threshold);
	pcl::console::parse(argc, argv, "-st", smoothness_threshold);
	use_extended_convexity = pcl::console::find_switch(argc, argv, "-ec");
	unsigned int k_factor = 0;
	if (use_extended_convexity)
		k_factor = 1;
	use_sanity_criterion = pcl::console::find_switch(argc, argv, "-sc");
	pcl::console::parse(argc, argv, "-smooth", min_segment_size);
	pcl::StopWatch timer_;
	timer_.reset ();
	double t_start = timer_.getTime ();
	ORBSLAM2::MarginVoxelLccp mvl (voxel_resolution, seed_resolution,input_cloud_ptr, objmask, rect);
	mvl.setSuperVoxelAdaptive(color_importance,spatial_importance,normal_importance);
	mvl.createLccp(concavity_tolerance_threshold,smoothness_threshold,min_segment_size,use_extended_convexity,use_sanity_criterion,k_factor);
	auto objcloud= mvl.segmentObjPoinCloud();
	pcl::PointCloud<pcl::PointXYZL>::Ptr lccp_labeled_cloud=mvl.relabelCloud();
	pcl::io::savePCDFile("obj.pcd", *objcloud);
	double t_end = timer_.getTime();
	 std::cout << "Total run time                                 ="<< t_end -t_start<<" ms\n";
	objcloud->width = objcloud->size();
	objcloud->height = 1;

	/// -----------------------------------|  Visualization  |-----------------------------------

	if (show_visualization)
	{
		/// Calculate visualization of adjacency graph
		// Using lines this would be VERY slow right now, because one actor is created for every line (may be fixed in future versions of PCL)
		// Currently this is a work-around creating a polygon mesh consisting of two triangles for each edge

		/// Configure Visualizer
		pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
		int v1(0);  //创建新的视口
		viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
		viewer->setBackgroundColor(0, 0, 0, v1);
		viewer->registerKeyboardCallback(keyboardEventOccurred, nullptr);
		pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> rgba(objcloud);
		viewer->addPointCloud(objcloud, rgba, "segcloud", v1);
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "segcloud", v1);
		int v2(0);  //创建新的视口
		viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
		viewer->setBackgroundColor(0, 0, 0, v2);
		viewer->registerKeyboardCallback(keyboardEventOccurred, nullptr);
		pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> objrgba(objRGBCloud);
		viewer->addPointCloud(objRGBCloud, objrgba, "origincloud", v2);
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "origincloud", v2);
		/// Visualization Loop
		PCL_INFO("Loading viewer\n");
		while (!viewer->wasStopped())
		{
			viewer->spinOnce(100);
			//if (!viewer->updatePointCloud(lccp_labeled_cloud, "lccpCloud"))
			//{
			//	viewer->addPointCloud(lccp_labeled_cloud, "lccpCloud");
			//	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3.0, "lccpCloud", v1);
			//}
			/*if (!show_normals) {
				if (!viewer->updatePointCloud(objcloud, "segcloud"))
				{
					pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> rgba(objcloud);
					viewer->addPointCloud(objcloud, rgba, "segcloud", v1);
					viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "segcloud", v1);
				}
			}
			else {
				
			}*/
			/// Show Segmentation or Supervoxels
			//viewer->updatePointCloud(input_cloud_ptr, "maincloud");

		

			std::this_thread::sleep_for(100ms);
		}
	}  /// END if (show_visualization)

	getchar(); return (0);

}  /// END main

   /// -------------------------| Definitions of helper functions|-------------------------

void
printText(pcl::visualization::PCLVisualizer::Ptr viewer_arg)
{
	std::string on_str = "ON";
	std::string off_str = "OFF";
	if (!viewer_arg->updateText("Press (1-n) to show different elements (d) to disable this", 5, 72, 12, 1.0, 1.0, 1.0, "hud_text"))
		viewer_arg->addText("Press (1-n) to show different elements", 5, 72, 12, 1.0, 1.0, 1.0, "hud_text");

	std::string temp = "(1) Supervoxel Normals, currently " + ((show_normals) ? on_str : off_str);
	if (!viewer_arg->updateText(temp, 5, 60, 10, 1.0, 1.0, 1.0, "normals_text"))
		viewer_arg->addText(temp, 5, 60, 10, 1.0, 1.0, 1.0, "normals_text");

	temp = "(2) Adjacency Graph, currently " + ((show_adjacency) ? on_str : off_str) + "\n      White: convex; Red: concave";
	if (!viewer_arg->updateText(temp, 5, 38, 10, 1.0, 1.0, 1.0, "graph_text"))
		viewer_arg->addText(temp, 5, 38, 10, 1.0, 1.0, 1.0, "graph_text");

	temp = "(3) Press to show " + ((show_supervoxels) ? std::string("SEGMENTATION") : std::string("SUPERVOXELS"));
	if (!viewer_arg->updateText(temp, 5, 26, 10, 1.0, 1.0, 1.0, "supervoxel_text"))
		viewer_arg->addText(temp, 5, 26, 10, 1.0, 1.0, 1.0, "supervoxel_text");

	temp = "(4/5) Press to increase/decrease normals scale, currently " + boost::str(boost::format("%.3f") % normals_scale);
	if (!viewer_arg->updateText(temp, 5, 14, 10, 1.0, 1.0, 1.0, "normals_scale_text"))
		viewer_arg->addText(temp, 5, 14, 10, 1.0, 1.0, 1.0, "normals_scale_text");
}

void
removeText(pcl::visualization::PCLVisualizer::Ptr viewer_arg)
{
	viewer_arg->removeShape("hud_text");
	viewer_arg->removeShape("normals_text");
	viewer_arg->removeShape("graph_text");
	viewer_arg->removeShape("supervoxel_text");
	viewer_arg->removeShape("normals_scale_text");
}