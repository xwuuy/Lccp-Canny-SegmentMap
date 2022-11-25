#include <pcl/common/time.h>
#include <pcl/console/parse.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/png_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
//#include <pcl/segmentation/supervoxel_clustering.h>
#include "Thirdpartylib/lccp_segmentation.h"

#include "Thirdpartylib/supervoxel_clustering.h"
#include <vtkImageReader2Factory.h>
#include <vtkImageReader2.h>
#include <vtkImageData.h>
#include <vtkImageFlip.h>
#include <vtkPolyLine.h>
#include<unordered_set>
#include <opencv2/core/core.hpp>  
#include <opencv2/highgui/highgui.hpp>  
#include<opencv2/imgcodecs.hpp>
#include <opencv2/opencv.hpp>
#include "SegImageRead.hpp"

// Types
typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointNormal PointNT;
typedef pcl::PointCloud<PointNT> PointNCloudT;
typedef pcl::PointXYZL PointLT;
typedef pcl::PointCloud<PointLT> PointLCloudT;
typedef pcl::Normal NormalT;
typedef pcl::PointCloud<NormalT> NormalCloudT;

bool show_voxel_centroids = true;
bool show_supervoxels = true;
bool show_supervoxel_normals = false;
bool show_graph = false;
bool show_normals = false;
bool show_refined = false;
bool show_help = true;

/** \brief Callback for setting options in the visualizer via keyboard.
*  \param[in] event Registered keyboard event  */
void keyboard_callback(const pcl::visualization::KeyboardEvent& event, void*)
{
	int key = event.getKeyCode();

	if (event.keyUp())
		switch (key)
		{
		case (int)'1': show_voxel_centroids = !show_voxel_centroids; break;
		case (int)'2': show_supervoxels = !show_supervoxels; break;
		case (int)'3': show_graph = !show_graph; break;
		case (int)'4': show_normals = !show_normals; break;
		case (int)'5': show_supervoxel_normals = !show_supervoxel_normals; break;
		case (int)'0': show_refined = !show_refined; break;
		case (int)'h': case (int)'H': show_help = !show_help; break;
		default: break;
		}

}

void addSupervoxelConnectionsToViewer(PointT &supervoxel_center,
	PointCloudT &adjacent_supervoxel_centers,
	std::string supervoxel_name,
	boost::shared_ptr<pcl::visualization::PCLVisualizer> & viewer);

/** \brief Displays info text in the specified PCLVisualizer
*  \param[in] viewer_arg The PCLVisualizer to modify  */
void printText(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer);

/** \brief Removes info text in the specified PCLVisualizer
*  \param[in] viewer_arg The PCLVisualizer to modify  */
void removeText(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer);

/** \brief Checks if the PCLPointCloud2 pc2 has the field named field_name
* \param[in] pc2 PCLPointCloud2 to check
* \param[in] field_name Fieldname to check
* \return True if field has been found, false otherwise */
bool hasField(const pcl::PCLPointCloud2 &pc2, const std::string field_name);


using namespace pcl;

int main(int argc, char ** argv)
{
	if (argc < 2)
	{
		pcl::console::print_info("Syntax is: %s {-p <pcd-file> OR -r <rgb-file> -d <depth-file>} \n --NT  (disables use of single camera transform) \n -o <output-file> \n -O <refined-output-file> \n-l <output-label-file> \n -L <refined-output-label-file> \n-v <voxel resolution> \n-s <seed resolution> \n-c <color weight> \n-z <spatial weight> \n-n <normal_weight>] \n", argv[0]);
		system("PAUSE"); return (1);
	}

	///  //
	//  //
	// THIS IS ALL JUST INPUT HANDLING - Scroll down until 
	// ORBSLAM2::SupervoxelClustering<pcl::PointXYZRGB> super
	//  //
	std::string rgb_path;
	bool rgb_file_specified = pcl::console::find_switch(argc, argv, "-r");
	if (rgb_file_specified)
		pcl::console::parse(argc, argv, "-r", rgb_path);

	std::string depth_path;
	bool depth_file_specified = pcl::console::find_switch(argc, argv, "-d");
	if (depth_file_specified)
		pcl::console::parse(argc, argv, "-d", depth_path);

	PointCloudT::Ptr cloud = boost::shared_ptr<PointCloudT>(new PointCloudT);
	PointCloudT::Ptr objrgbaEdgeCloud = boost::shared_ptr<PointCloudT>(new PointCloudT);
	NormalCloudT::Ptr input_normals = boost::make_shared < NormalCloudT >();

	bool pcd_file_specified = pcl::console::find_switch(argc, argv, "-p");
	std::string pcd_path;
	std::string rgb_num("1305031910.835208");
	bool rgb_num_specified = pcl::console::find_switch(argc, argv, "-num");
	if (rgb_num_specified)
		pcl::console::parse(argc, argv, "-num", rgb_num);
	if ( !rgb_num_specified)
	{
		std::cout << "Using point cloud\n";
		if (!pcd_file_specified)
		{
			std::cout << "No cloud specified!\n";
			system("PAUSE"); return (1);
		}
		else
		{
			pcl::console::parse(argc, argv, "-p", pcd_path);
		}
	}
	

	bool disable_transform = pcl::console::find_switch(argc, argv, "--NT");
	bool ignore_provided_normals = pcl::console::find_switch(argc, argv, "--nonormals");
	bool has_normals = false;

	std::string out_path = "test_output.png";;
	pcl::console::parse(argc, argv, "-o", out_path);

	std::string out_label_path = "test_output_labels.png";
	pcl::console::parse(argc, argv, "-l", out_label_path);

	std::string refined_out_path = "refined_test_output.png";
	pcl::console::parse(argc, argv, "-O", refined_out_path);

	std::string refined_out_label_path = "refined_test_output_labels.png";;
	pcl::console::parse(argc, argv, "-L", refined_out_label_path);

	float voxel_resolution = 0.004f;
	pcl::console::parse(argc, argv, "-v", voxel_resolution);

	float seed_resolution = 0.08f;
	pcl::console::parse(argc, argv, "-s", seed_resolution);

	float color_importance = 0.4f;
	pcl::console::parse(argc, argv, "-c", color_importance);

	float spatial_importance = 0.4f;
	pcl::console::parse(argc, argv, "-z", spatial_importance);

	float normal_importance = 0.7f;
	pcl::console::parse(argc, argv, "-n", normal_importance);

	cv::Rect rect;
	if (!pcd_file_specified) {
		std::string rgb_num("1305031910.835208");
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
		auto info = vinfotemp[0];
		cloud = info->objrgbaCloud;
		rect = info->ojbROI;
		objrgbaEdgeCloud = info->objrgbaEdgeCloud;
		pcl::io::savePCDFile("origin.pcd", *info->objMaskrgbaCloud);
		cv::imshow("edge", info->objedge);
		cv::imshow("rgb", info->Rectrgb);
	}
	else
	{
		/// check if the provided pcd file contains normals
		pcl::PCLPointCloud2 input_pointcloud2;
		if (pcl::io::loadPCDFile(pcd_path, input_pointcloud2))
		{
			PCL_ERROR("ERROR: Could not read input point cloud %s.\n", pcd_path.c_str());
			return (3);
		}
		pcl::fromPCLPointCloud2(input_pointcloud2, *cloud);
		if (!ignore_provided_normals)
		{
			if (hasField(input_pointcloud2, "normal_x"))
			{
				std::cout << "Using normals contained in file. Set --nonormals option to disable this.\n";
				pcl::fromPCLPointCloud2(input_pointcloud2, *input_normals);
				has_normals = true;
			}
		}
	}
	std::cout << "Done making cloud!\n";

	///  //
	//  //
	// This is how to use supervoxels
	//  //
	//  //
	
	// If the cloud is organized and we haven't disabled the transform we need to
	// check that there are no negative z values, since we use log(z)
	if (cloud->isOrganized() && !disable_transform)//判断当前点云是否为单相机通过相机模型转换而来
	{
		for (PointCloudT::iterator cloud_itr = cloud->begin(); cloud_itr != cloud->end(); ++cloud_itr)
			if (cloud_itr->z < 0)
			{
				PCL_ERROR("Points found with negative Z values, this is not compatible with the single camera transform!\n");
				PCL_ERROR("Set the --NT option to disable the single camera transform!\n");
				return 1;
			}
		std::cout << "You have the single camera transform enabled - this should be used with point clouds captured from a single camera.\n";
		std::cout << "You can disable the transform with the --NT flag\n";
	}
	ORBSLAM2::SupervoxelClustering<PointT> super(voxel_resolution, seed_resolution);
	//If we manually disabled the transform then do so, otherwise the default 
	//behavior will take place (true for organized, false for unorganized)
	if (disable_transform)
		super.setUseSingleCameraTransform(false);
	super.setInputCloud(cloud);
	if (has_normals)
		super.setNormalCloud(input_normals);
	super.setColorImportance(color_importance);
	super.setSpatialImportance(spatial_importance);
	super.setNormalImportance(normal_importance);
	std::map <uint32_t, pcl::Supervoxel<PointT>::Ptr > supervoxel_clusters;

	std::cout << "Extracting supervoxels!\n";
	super.extract(supervoxel_clusters);
	std::cout << "Found " << supervoxel_clusters.size() << " Supervoxels!\n";

	PointLCloudT::Ptr labeled_voxel_cloud = super.getLabeledVoxelCloud();
	PointCloudT::Ptr voxel_centroid_cloud = super.getVoxelCentroidCloud();
	PointNCloudT::Ptr sv_normal_cloud = super.makeSupervoxelNormalCloud(supervoxel_clusters);
	PointLCloudT::Ptr full_labeled_cloud = super.getLabeledCloud();
	//pcl::io::savePCDFileASCII("labeled_voxel_cloud.pcd", *labeled_voxel_cloud);

	std::map <uint32_t, pcl::Supervoxel<PointT>::Ptr > refined_supervoxel_clusters;
	std::cout << "Refining supervoxels \n";
	super.refineSupervoxels(2, refined_supervoxel_clusters);//再精细
	super.setMarginVoxel(rect, 5);//边缘体素提取
	std::cout << "Getting supervoxel adjacency\n";
	std::multimap<uint32_t, uint32_t> label_adjacency;
	super.getSupervoxelAdjacency(label_adjacency);//体素邻接关系获取
	PointLCloudT::Ptr refined_labeled_voxel_cloud = super.getLabeledVoxelCloud();

	PointNCloudT::Ptr refined_sv_normal_cloud = super.makeSupervoxelNormalCloud(refined_supervoxel_clusters);

	PointLCloudT::Ptr refined_full_labeled_cloud = super.getLabeledCloud();
	


	// LCCPSegmentation Stuff
	////float concavity_tolerance_threshold = 8;
	////float smoothness_threshold = 0.01;
	////std::uint32_t min_segment_size = 0;
	////bool use_extended_convexity = false;
	////bool use_sanity_criterion = false;
	////pcl::console::parse(argc, argv, "-ct", concavity_tolerance_threshold);
	////pcl::console::parse(argc, argv, "-st", smoothness_threshold);
	////use_extended_convexity = pcl::console::find_switch(argc, argv, "-ec");
	////unsigned int k_factor = 0;
	////if (use_extended_convexity)
	////	k_factor = 1;
	////use_sanity_criterion = pcl::console::find_switch(argc, argv, "-sc");
	////pcl::console::parse(argc, argv, "-smooth", min_segment_size);
	//////lccp segment
	////pcl::LCCPSegmentation<PointT> lccp;
	////lccp.setConcavityToleranceThreshold(concavity_tolerance_threshold);
	////lccp.setSanityCheck(use_sanity_criterion);
	////lccp.setSmoothnessCheck(true, voxel_resolution, seed_resolution, smoothness_threshold);
	////lccp.setKFactor(k_factor);
	////lccp.setInputSupervoxels(supervoxel_clusters, label_adjacency);
	////lccp.setMinSegmentSize(min_segment_size);
	////lccp.segment();

	// THESE ONLY MAKE SENSE FOR ORGANIZED CLOUDS
	if (cloud->isOrganized())
	{
		pcl::io::savePNGFile(out_label_path, *full_labeled_cloud, "label");
		pcl::io::savePNGFile(refined_out_label_path, *refined_full_labeled_cloud, "label");
		//Save RGB from labels
		pcl::io::PointCloudImageExtractorFromLabelField<PointLT> pcie(pcl::io::PointCloudImageExtractorFromLabelField<PointLT>::COLORS_RGB_GLASBEY);
		//We need to set this to account for NAN points in the organized cloud
		pcie.setPaintNaNsWithBlack(true);
		pcl::PCLImage image;
		pcie.extract(*full_labeled_cloud, image);
		pcl::io::savePNGFile(out_path, image);
		pcie.extract(*refined_full_labeled_cloud, image);
		pcl::io::savePNGFile(refined_out_path, image);
	}

	std::cout << "Constructing Boost Graph Library Adjacency List...\n";
	typedef boost::adjacency_list<boost::setS, boost::setS, boost::undirectedS, uint32_t, float> VoxelAdjacencyList;
	VoxelAdjacencyList supervoxel_adjacency_list;
	super.getSupervoxelAdjacencyList(supervoxel_adjacency_list);


	std::cout << "Loading visualization...\n";
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);
	viewer->registerKeyboardCallback(keyboard_callback, 0);


	bool refined_normal_shown = show_refined;
	bool refined_sv_normal_shown = show_refined;
	bool sv_added = false;
	bool normals_added = false;
	bool graph_added = false;
	std::vector<std::string> poly_names;
	std::cout << "Loading viewer...\n";
	while (!viewer->wasStopped())
	{
		/*if (1) {
			if (!viewer->updatePointCloud(objrgbaEdgeCloud, "colored cloud"))
			{
				viewer->addPointCloud(objrgbaEdgeCloud, "colored cloud");
				viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3.0, "colored cloud");
			}
		}*/
		if (show_supervoxels)
		{
			if (!viewer->updatePointCloud((show_refined) ? refined_labeled_voxel_cloud : labeled_voxel_cloud, "colored voxels"))
			{
				viewer->addPointCloud((show_refined) ? refined_labeled_voxel_cloud : labeled_voxel_cloud, "colored voxels");
				viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3.0, "colored voxels");
			}
		}
		else
		{
			viewer->removePointCloud("colored voxels");
		}

		if (show_voxel_centroids)
		{
			if (!viewer->updatePointCloud(voxel_centroid_cloud, "voxel centroids"))
			{
				viewer->addPointCloud(voxel_centroid_cloud, "voxel centroids");
				viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2.0, "voxel centroids");
			}
		}
		else
		{
			viewer->removePointCloud("voxel centroids");
		}

		if (show_supervoxel_normals)
		{
			if (refined_sv_normal_shown != show_refined || !sv_added)
			{
				viewer->removePointCloud("supervoxel_normals");
				viewer->addPointCloudNormals<PointNormal>((show_refined) ? refined_sv_normal_cloud : sv_normal_cloud, 1, 0.05f, "supervoxel_normals");
				sv_added = true;
			}
			refined_sv_normal_shown = show_refined;
		}
		else if (!show_supervoxel_normals)
		{
			viewer->removePointCloud("supervoxel_normals");
		}

		if (show_normals)
		{
			std::map <uint32_t, pcl::Supervoxel<PointT>::Ptr>::iterator sv_itr, sv_itr_end;
			sv_itr = ((show_refined) ? refined_supervoxel_clusters.begin() : supervoxel_clusters.begin());
			sv_itr_end = ((show_refined) ? refined_supervoxel_clusters.end() : supervoxel_clusters.end());
			for (; sv_itr != sv_itr_end; ++sv_itr)
			{
				std::stringstream ss;
				ss << sv_itr->first << "_normal";
				if (refined_normal_shown != show_refined || !normals_added)
				{
					viewer->removePointCloud(ss.str());
					viewer->addPointCloudNormals<PointT, Normal>((sv_itr->second)->voxels_, (sv_itr->second)->normals_, 10, 0.02f, ss.str());
					//  std::cout << (sv_itr->second)->normals_->points[0]<<"\n";

				}

			}
			normals_added = true;
			refined_normal_shown = show_refined;
		}
		else if (!show_normals)
		{
			std::map <uint32_t, pcl::Supervoxel<PointT>::Ptr>::iterator sv_itr, sv_itr_end;
			sv_itr = ((show_refined) ? refined_supervoxel_clusters.begin() : supervoxel_clusters.begin());
			sv_itr_end = ((show_refined) ? refined_supervoxel_clusters.end() : supervoxel_clusters.end());
			for (; sv_itr != sv_itr_end; ++sv_itr)
			{
				std::stringstream ss;
				ss << sv_itr->first << "_normal";
				viewer->removePointCloud(ss.str());
			}
		}

		if (show_graph && !graph_added)
		{
			poly_names.clear();
			std::multimap<uint32_t, uint32_t>::iterator label_itr = label_adjacency.begin();
			for (; label_itr != label_adjacency.end(); )
			{
				//First get the label 
				uint32_t supervoxel_label = label_itr->first;
				//Now get the supervoxel corresponding to the label
				pcl::Supervoxel<PointT>::Ptr supervoxel = supervoxel_clusters.at(supervoxel_label);
				//Now we need to iterate through the adjacent supervoxels and make a point cloud of them
				PointCloudT adjacent_supervoxel_centers;
				std::multimap<uint32_t, uint32_t>::iterator adjacent_itr = label_adjacency.equal_range(supervoxel_label).first;
				for (; adjacent_itr != label_adjacency.equal_range(supervoxel_label).second; ++adjacent_itr)
				{
					pcl::Supervoxel<PointT>::Ptr neighbor_supervoxel = supervoxel_clusters.at(adjacent_itr->second);
					adjacent_supervoxel_centers.push_back(neighbor_supervoxel->centroid_);
				}
				//Now we make a name for this polygon
				std::stringstream ss;
				ss << "supervoxel_" << supervoxel_label;
				poly_names.push_back(ss.str());
				addSupervoxelConnectionsToViewer(supervoxel->centroid_, adjacent_supervoxel_centers, ss.str(), viewer);
				//Move iterator forward to next label
				label_itr = label_adjacency.upper_bound(supervoxel_label);
			}

			graph_added = true;
		}
		else if (!show_graph && graph_added)
		{
			for (std::vector<std::string>::iterator name_itr = poly_names.begin(); name_itr != poly_names.end(); ++name_itr)
			{
				viewer->removeShape(*name_itr);
			}
			graph_added = false;
		}

		if (show_help)
		{
			viewer->removeShape("help_text");
			printText(viewer);
		}
		else
		{
			removeText(viewer);
			if (!viewer->updateText("Press h to show help", 5, 10, 12, 1.0, 1.0, 1.0, "help_text"))
				viewer->addText("Press h to show help", 5, 10, 12, 1.0, 1.0, 1.0, "help_text");
		}


		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));

	}
	return (0);
}

void addSupervoxelConnectionsToViewer(PointT &supervoxel_center,
	PointCloudT &adjacent_supervoxel_centers,
	std::string supervoxel_name,
	boost::shared_ptr<pcl::visualization::PCLVisualizer> & viewer)
{
	vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
	vtkSmartPointer<vtkCellArray> cells = vtkSmartPointer<vtkCellArray>::New();
	vtkSmartPointer<vtkPolyLine> polyLine = vtkSmartPointer<vtkPolyLine>::New();

	//Iterate through all adjacent points, and add a center point to adjacent point pair
	PointCloudT::iterator adjacent_itr = adjacent_supervoxel_centers.begin();
	for (; adjacent_itr != adjacent_supervoxel_centers.end(); ++adjacent_itr)
	{
		points->InsertNextPoint(supervoxel_center.data);
		points->InsertNextPoint(adjacent_itr->data);
	}
	// Create a polydata to store everything in
	vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();
	// Add the points to the dataset
	polyData->SetPoints(points);
	polyLine->GetPointIds()->SetNumberOfIds(points->GetNumberOfPoints());
	for (unsigned int i = 0; i < points->GetNumberOfPoints(); i++)
		polyLine->GetPointIds()->SetId(i, i);
	cells->InsertNextCell(polyLine);
	// Add the lines to the dataset
	polyData->SetLines(cells);
	viewer->addModelFromPolyData(polyData, supervoxel_name);
}


void printText(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer)
{
	std::string on_str = "on";
	std::string off_str = "off";
	if (!viewer->updateText("Press (1-n) to show different elements (h) to disable this", 5, 72, 12, 1.0, 1.0, 1.0, "hud_text"))
		viewer->addText("Press 1-n to show different elements", 5, 72, 12, 1.0, 1.0, 1.0, "hud_text");

	std::string temp = "(1) Voxels currently " + ((show_voxel_centroids) ? on_str : off_str);
	if (!viewer->updateText(temp, 5, 60, 10, 1.0, 1.0, 1.0, "voxel_text"))
		viewer->addText(temp, 5, 60, 10, 1.0, 1.0, 1.0, "voxel_text");

	temp = "(2) Supervoxels currently " + ((show_supervoxels) ? on_str : off_str);
	if (!viewer->updateText(temp, 5, 50, 10, 1.0, 1.0, 1.0, "supervoxel_text"))
		viewer->addText(temp, 5, 50, 10, 1.0, 1.0, 1.0, "supervoxel_text");

	temp = "(3) Graph currently " + ((show_graph) ? on_str : off_str);
	if (!viewer->updateText(temp, 5, 40, 10, 1.0, 1.0, 1.0, "graph_text"))
		viewer->addText(temp, 5, 40, 10, 1.0, 1.0, 1.0, "graph_text");

	temp = "(4) Voxel Normals currently " + ((show_normals) ? on_str : off_str);
	if (!viewer->updateText(temp, 5, 30, 10, 1.0, 1.0, 1.0, "voxel_normals_text"))
		viewer->addText(temp, 5, 30, 10, 1.0, 1.0, 1.0, "voxel_normals_text");

	temp = "(5) Supervoxel Normals currently " + ((show_supervoxel_normals) ? on_str : off_str);
	if (!viewer->updateText(temp, 5, 20, 10, 1.0, 1.0, 1.0, "supervoxel_normals_text"))
		viewer->addText(temp, 5, 20, 10, 1.0, 1.0, 1.0, "supervoxel_normals_text");

	temp = "(0) Showing " + std::string((show_refined) ? "" : "UN-") + "refined supervoxels and normals";
	if (!viewer->updateText(temp, 5, 10, 10, 1.0, 1.0, 1.0, "refined_text"))
		viewer->addText(temp, 5, 10, 10, 1.0, 1.0, 1.0, "refined_text");
}

void removeText(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer)
{
	viewer->removeShape("hud_text");
	viewer->removeShape("voxel_text");
	viewer->removeShape("supervoxel_text");
	viewer->removeShape("graph_text");
	viewer->removeShape("voxel_normals_text");
	viewer->removeShape("supervoxel_normals_text");
	viewer->removeShape("refined_text");
}

bool
hasField(const pcl::PCLPointCloud2 &pc2, const std::string field_name)
{
	for (size_t cf = 0; cf < pc2.fields.size(); ++cf)
		if (pc2.fields[cf].name == field_name)
			return true;
	return false;
}

