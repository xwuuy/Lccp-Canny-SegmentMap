#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include<iostream>
#include<string>
#include<fstream>
#include <algorithm>
#include <iomanip>
#include<yaml-cpp/yaml.h>
#include<stdio.h>
#include<vector>
#include <pcl/point_types.h>
#include<pcl/io/pcd_io.h>
#include<pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

const static float fx = 517.306408, fy = 516.469215, cx = 318.643040, cy = 255.313989;
using namespace std;
class ObjInfo {
public:
	ObjInfo(cv::Rect ojbROIs, float scores, int objClasss) {
		ojbROI = ojbROIs;
		score = scores;
		objClass = objClasss;
	}
	cv::Rect ojbROI;
	float score;
	int objClass;
	cv::Mat objMat;
	cv::Mat Rectrgb;
	cv::Mat Rectdepth;
	cv::Mat objedge;
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr objrgbaCloud;
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr objrgbaEdgeCloud;
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr objMaskrgbaCloud;
};

//class KeyFrame {
//public:
//	KeyFrame() {}
//	cv::Mat imLeftRgb;
//	cv::Mat imDepth;
//	Eigen::Matrix4d pose;
//	std::multimap<int, ObjInfo*> infos;
//	Eigen::Isometry3d toIs3d() {
//		Eigen::Isometry3d T1 = Eigen::Isometry3d::Identity();
//		T1(0, 0) = pose(0, 0), T1(0, 1) = pose(0, 1), T1(0, 2) = pose(0, 2), T1(0, 3) = pose(0, 3);
//		T1(1, 0) = pose(1, 0), T1(1, 1) = pose(1, 1), T1(1, 2) = pose(1, 2), T1(1, 3) = pose(1, 3);
//		T1(2, 0) = pose(2, 0), T1(2, 1) = pose(2, 1), T1(2, 2) = pose(2, 2), T1(2, 3) = pose(2, 3);
//		T1(3, 0) = 0, T1(3, 1) = 0, T1(3, 2) = 0, T1(3, 3) = 1;
//		return T1;
//	}
//	Eigen::Matrix4d GetPose() {
//		return pose;
//	}
//	Eigen::Matrix4d GetPoseInverse() {
//		return toIs3d().inverse().matrix();
//	}
//};
class DebugObj {
public:
	static  void  LoadTrace(const string &strTraceFilename, vector<Eigen::Matrix4d> &vFrameT) {
		ifstream fAssociation;
		fAssociation.open(strTraceFilename.c_str());
		if (!fAssociation.good())
		{
			cout << "strTraceFilename:" << strTraceFilename << endl;
			cout << "fAssociation.rdstate():" << fAssociation.rdstate() << endl;
			return;
		}
		while (!fAssociation.eof())
		{
			string s;
			getline(fAssociation, s);
			if (!s.empty())
			{
				stringstream ss;
				ss << s;
				double frame, x, y, z, qx, qy, qz, qw;
				ss >> frame;
				ss >> x;
				ss >> y;
				ss >> z;
				ss >> qx;
				ss >> qy;
				ss >> qz;
				ss >> qw;
				Eigen::Quaterniond quaternion(qw, qx, qy, qz);
				Eigen::Matrix3d rotation_matrix = quaternion.toRotationMatrix();
				Eigen::Matrix4d T;
				T << rotation_matrix(0, 0), rotation_matrix(0, 1), rotation_matrix(0, 2), x,
					rotation_matrix(1, 0), rotation_matrix(1, 1), rotation_matrix(1, 2), y,
					rotation_matrix(2, 0), rotation_matrix(2, 1), rotation_matrix(2, 2), z,
					0, 0, 0, 1;
				vFrameT.emplace_back(T);
			}
		}
	}
	static void LoadImages(const string &strAssociationFilename, vector<double> &vstrImageFilenamesRGB,
		vector<double> &vstrImageFilenamesD, vector<double> &vTimestamps)
	{
		ifstream fAssociation;
		fAssociation.open(strAssociationFilename.c_str());
		if (!fAssociation.good())
		{
			cout << "strAssociationFilename:" << strAssociationFilename << endl;
			cout << "fAssociation.rdstate():" << fAssociation.rdstate() << endl;
			return;
		}
		while (!fAssociation.eof())
		{
			string s;
			getline(fAssociation, s);
			if (!s.empty())
			{
				stringstream ss;
				ss << s;
				double t;
				string sRGB, sD;
				ss >> t;
				vTimestamps.push_back(t);
				ss >> sRGB;
				vstrImageFilenamesRGB.push_back(t);
				ss >> t;
				ss >> sD;
				vstrImageFilenamesD.push_back(t);
			}
		}
	}

	static pcl::PointCloud<pcl::PointXYZRGBA>::Ptr generatePointCloud(cv::Mat edge, cv::Mat rgb, cv::Mat depth, cv::Mat mask, cv::Rect rect, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &objrgbaEdgeCloud, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &maskpointcloude) {
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr rgbaCloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr objrgbaEdge(new pcl::PointCloud<pcl::PointXYZRGBA>);
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr  maskpointcloud(new pcl::PointCloud<pcl::PointXYZRGBA>());
		for (int m = 0; m < depth.rows; m+=1)
		{
			for (int n = 0; n < depth.cols; n+=1)
			{
				float d = depth.ptr<float>(m)[n] ;
				uchar c = mask.ptr<uchar>(m)[n];
				if (d < 0.05 || d > 6 )
					continue;
				pcl::PointXYZRGBA p;
				pcl::PointXYZRGBA p1;
				p.z = d;
				p.x = (n+ rect.x - cx) * p.z / fx;//(p.x*kf->fx)/p.z+kf->cx
				p.y = (m + rect .y - cy) * p.z / fy;//(p.y*kf->fy)/p.z+kf->cy
				p1 = p;
				
				if (edge.ptr<uchar>(m)[n] > 0) {
					p.a = 1;
					/*p1.b =255;
					p1.g = 255;
					p1.r = 255;*/
				}
				else {
					p.a = 0;
				}
				p.b = rgb.ptr<uchar>(m)[n * 3];
				p.g = rgb.ptr<uchar>(m)[n * 3 + 1];
				p.r = rgb.ptr<uchar>(m)[n * 3 + 2];
				rgbaCloud->push_back(p);
				objrgbaEdge->push_back(p1);
				if (c > 0) {
					maskpointcloud->push_back(p);
				}
			}
		}
		objrgbaEdgeCloud=objrgbaEdge;
		maskpointcloude = maskpointcloud;
		return rgbaCloud;
	}
	static void loadObjinfo(std::string infopath,cv::Mat object_mask, cv::Mat rgb,cv::Mat depth,cv::Mat edgeimg,vector<ObjInfo*> &vinfos) {
		YAML::Node objinfo = YAML::LoadFile(infopath);
		std::vector<std::vector<int>> boxes = objinfo["boxes"].as<std::vector<std::vector<int>>>();
		std::vector<int> classes = objinfo["classes"].as<std::vector<int>>();
		std::vector<float> scores = objinfo["scores"].as<std::vector<float>>();
		int maskheight = 0;
		for (size_t num = 0; num < scores.size(); num++)
		{
			std::vector<int> box = boxes.at(num);
			ObjInfo* info = new ObjInfo(cv::Rect(box.at(0), box.at(1), box.at(2), box.at(3)), scores.at(num), classes.at(num));
			cv::Rect rect;
			if (maskheight == 0)
			{
				rect = cv::Rect(0, maskheight, info->ojbROI.width, info->ojbROI.height);
			}
			else
			{
				if (maskheight + info->ojbROI.height >= object_mask.size().height)
				{

					rect = cv::Rect(0, maskheight, info->ojbROI.width, object_mask.size().height - maskheight - 1);
				}
				else {
					rect = cv::Rect(0, maskheight, info->ojbROI.width, info->ojbROI.height);
				}
			}
			info->objMat = object_mask(rect).clone();
			info->Rectrgb = rgb(info->ojbROI).clone();
			info->Rectdepth = depth(info->ojbROI).clone();
			info->objedge= info->objMat.mul(edgeimg(info->ojbROI));

			/*cv::Mat edgergb;
			cvtColor(info->objedge, edgergb, cv::COLOR_GRAY2BGR, 3);
			cv::imshow("edge", edgergb);
			int gridheight = edgergb.size().height;
			int gridwidth = edgergb.size().width;
			for (size_t y = 0; y < gridheight; y++)
			{
				bool one = true;
				bool two = true;
				for (size_t x = 0, bx = gridwidth - 1; x < gridwidth && bx >= 0 && (one || two); )
				{
					if (one) {
						uchar gray = edgergb.ptr<uchar>(y)[3*x];
						if (gray>0) {
							edgergb.ptr<uchar>(y)[3*x+2] = 198;
							edgergb.ptr<uchar>(y)[3 * x + 1] = 23;
							edgergb.ptr<uchar>(y)[3 * x ] = 19;
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
						uchar gray = edgergb.ptr<uchar>(y)[3*bx];
						if (gray>0) {
							edgergb.ptr<uchar>(y)[3 * bx + 2] = 198;
							edgergb.ptr<uchar>(y)[3 * bx + 1] = 23;
							edgergb.ptr<uchar>(y)[3 * bx] = 19;
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
				for (size_t y = 0, by = gridheight - 1; y < gridheight && by >= 0 && (one || two);)
				{
					if (one) {
						uchar gray = edgergb.ptr<uchar>(y)[3*x];
						if (gray>0) {
							edgergb.ptr<uchar>(y)[3 * x + 2] = 198;
							edgergb.ptr<uchar>(y)[3 * x + 1] = 23;
							edgergb.ptr<uchar>(y)[3 * x] = 19;
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
						uchar gray = edgergb.ptr<uchar>(by)[3*x];
						if (gray>0) {
							edgergb.ptr<uchar>(by)[3 * x + 2] = 198;
							edgergb.ptr<uchar>(by)[3 * x + 1] = 23;
							edgergb.ptr<uchar>(by)[3 * x] = 19;
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
			cv::imshow("margin", edgergb);
			while (cv::waitKey(0) != 'd')
				;*/
			info->objrgbaCloud = generatePointCloud(info->objedge, info->Rectrgb, info->Rectdepth,info->objMat,info->ojbROI,info->objrgbaEdgeCloud, info->objMaskrgbaCloud);
			maskheight += (info->ojbROI.height + 1);
			/*cv::imshow("objedge", info->objedge);
			cv::imshow("edgeimg", edgeimg(info->ojbROI).clone());
			cv::imshow("Rectrgb", info->Rectrgb);
			cv::imshow("objMat", info->objMat);
			while (cv::waitKey(0) != 'd')
				;*/
			/*pcl::visualization::CloudViewer viewer("viewer");
			viewer.showCloud(info->objrgbaCloud);
			while (!viewer.wasStopped())
			{

			}*/
			vinfos.push_back(info);
		}
	}
	static void imgread(std::string path, std::string depthpath, cv::Mat &mask, cv::Mat &object_mask, cv::Mat &rgb, cv::Mat &depth, vector<ObjInfo*> &vinfos) {
		std::string str1 = "C:/Users/16694/source/repos/Project1/rgbd_dataset_freiburg1_room/object-mask/" + path + ".png";
		object_mask = cv::imread(str1, cv::IMREAD_UNCHANGED);
		str1 = "C:/Users/16694/source/repos/Project1/rgbd_dataset_freiburg1_room/rgb/" + path + ".png";
		rgb = cv::imread(str1, cv::IMREAD_UNCHANGED);
		cv::imshow("rgb", rgb);
		cv::Mat cn, gray;
		cv::cvtColor(rgb, gray, cv::COLOR_BGR2GRAY);
		cv::Canny(gray, cn, 50, 180);
		cv::Mat element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
		cv::dilate(cn, cn, element);
		str1 = "C:/Users/16694/source/repos/Project1/rgbd_dataset_freiburg1_room/depth/" + depthpath + ".png";
		depth = cv::imread(str1, cv::IMREAD_UNCHANGED);
		cv::imshow("depth", depth);
		std::cout<<"depth type="<<depth.type();
		depth.convertTo(depth, CV_32F);
		depth /= 5000;
		str1 = "C:/Users/16694/source/repos/Project1/rgbd_dataset_freiburg1_room/yaml/" + path + ".yml";
		cv::imshow("canny", cn);
	/*	cv::imshow("object_mask",object_mask);
		cv::imshow("rgb",rgb);
		cv::imshow("canny", cn);
		cv::imshow("depth", depth);
		while(cv::waitKey(0)!='d')
			;*/
		loadObjinfo(str1, object_mask,rgb,depth, cn,vinfos);
	}
};
//int main(int argc, char** argv)
//{
//	vector<double>  vstrImageFilenamesRGB, vstrImageFilenamesD, vTimestamps;
//	DebugObj::LoadImages("C:/Users/16694/source/repos/Project1/rgbd_dataset_freiburg1_room/associate.txt", vstrImageFilenamesRGB, vstrImageFilenamesD, vTimestamps);
//	vector<Eigen::Matrix4d> vFrameT;
//	DebugObj::LoadTrace("C:/Users/16694/source/repos/Project1/rgbd_dataset_freiburg1_room/groundtruth.txt", vFrameT);
//	vector<vector<ObjInfo*>> vinfos;
//	for (int il = 2; il<argc; il++)
//	{
//			std::string str("1305031910.835208");
//			cv::Mat  mask, object_mask, rgb, depth;
//			stringstream ss;
//			ss << str;
//			double tempframe;
//			ss >> tempframe;
//			vector<double>::iterator t = find(vstrImageFilenamesRGB.begin(), vstrImageFilenamesRGB.end(), tempframe);
//			int d = distance(vstrImageFilenamesRGB.begin(), t);
//			double dt = vstrImageFilenamesD.at(d);
//			std::string ssrt;
//			stringstream sss;
//			sss << std::fixed << std::setprecision(6) << dt;
//			sss >> ssrt;
//			std::cout << ssrt << endl;
//			vector<ObjInfo*> vinfotemp;
//			DebugObj::imgread(str, ssrt, mask, object_mask, rgb, depth, vinfotemp);
//
//	}
//	return 1;
//}