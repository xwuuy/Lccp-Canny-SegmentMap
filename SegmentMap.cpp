
#include <iostream>
#include <vector>
#include <thread>
#include <string>
#include <QDir>
#include <QString>
#include <chrono>

// opencv 用于图像数据读取与处理
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// 使用Eigen的Geometry模块处理3d运动
#include <Eigen/Core>
#include <Eigen/Geometry>

// pcl
#include <pcl/common/transforms.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/cloud_viewer.h>
// boost.format 字符串处理
#include <boost/format.hpp>
#include <boost/program_options.hpp>

#include "ReadFile.hpp"
#include<boost/unordered_map.hpp>
#include<boost/filesystem.hpp>

#include "DivideObj/MarginVoxelLccp.h"
using namespace std;


namespace po = boost::program_options;
bool nextSeg=false;
void
keyboardEventOccurred(const pcl::visualization::KeyboardEvent& event_arg,
                      void*)
{
    int key = event_arg.getKeyCode();

    if (event_arg.keyUp())
        switch (key)
        {
        case (int) 'd':
            nextSeg=true;
            break;
        default:
            break;
        }
}
int main( int argc, char** argv )
{

    po::options_description parser("test");
    po::variables_map vm;
    parser.add_options()("Dataset,D",po::value<std::string>(),"the Dataset name which want to be found")
            ("Trackfile,t",po::value<std::string>(),"the Trackfile name which want to be found")
            ("color_importance,c",po::value<float>(),"the color_importance name which want to be found")
            ("spatial_importance,s",po::value<float>(),"the spatial_importance name which want to be found")
            ("normal_importance,n",po::value<float>(),"the normal_importance name which want to be found")
            ("concavity_tolerance_threshold,C",po::value<float>(),"the concavity_tolerance_threshold name which want to be found")
            ("smoothness_threshold,S",po::value<float>(),"the smoothness_threshold name which want to be found")
            ("connect_MarginVoxel,M",po::value<bool>(),"the PointOrVoxel name which want to be found")
            ("useLCCP,L" , "the PointOrVoxel name which want to be found")
            ("useK_connect,K" , "the PointOrVoxel name which want to be found");

    try{
        //parse_command_line()对输入的选项做解析
        //store()将解析后的结果存入选项存储器
        po::store(po::parse_command_line(argc, argv, parser), vm);
    }
    catch(...){
        std::cout << "输入的参数中存在未定义的选项！\n";
        return 0;
    }
    string DatasetPath;
    string track_path;
    float spatial_importance=0.6f, color_importance=0.4f,normal_importance=0.4f ,smoothness_threshold=0.01f, concavity_tolerance_threshold=30;
    std::uint32_t K_connect=0;
    if(vm.count("useK_connect")){
        K_connect=1;
    }
    if(vm.count("Dataset")&&vm.count("Trackfile")){
        DatasetPath=vm["Dataset"].as<string>();
        track_path=vm["Trackfile"].as<string>();
    }else
        return -1;
    bool connect_MarginVoxel=true;
    if(vm.count("connect_MarginVoxel")){
        connect_MarginVoxel=vm["connect_MarginVoxel"].as<bool>();
    }
    bool useLCCP=false;
    if(vm.count("useLCCP")){
        useLCCP=true;
    }
    if(vm.count("color_importance")&&vm.count("spatial_importance")&&vm.count("normal_importance")
            &&vm.count("concavity_tolerance_threshold")&&vm.count("smoothness_threshold")){
        spatial_importance=vm["spatial_importance"].as<float>();
        color_importance=vm["color_importance"].as<float>();
        normal_importance=vm["normal_importance"].as<float>();
        concavity_tolerance_threshold=vm["concavity_tolerance_threshold"].as<float>();
        smoothness_threshold=vm["smoothness_threshold"].as<float>();
    }
    FrameTrackIDMap vstrTrackPoses;
    ReadFile::setCameraPara(1);
    ReadFile::LoadTrack(track_path,vstrTrackPoses);

    cout<<"load total "<<vstrTrackPoses.size()<<" keyframes. "<<endl;

    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->initCameraParameters();
    int v1(0);  //创建新的视口
    viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
    viewer->setBackgroundColor(0.4, 0.4, 0.4, v1);
    viewer->registerKeyboardCallback(keyboardEventOccurred, nullptr);

    int v2(0);  //创建新的视口
    viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
    viewer->setBackgroundColor(0.4, 0.4, 0.4, v2);
    //    viewer->addCoordinateSystem(1.0);

    FrameIDMap vstrImageFilenamesRGBD;
    vector<string> vTimestamps;

    ReadFile::LoadImages(DatasetPath,vstrImageFilenamesRGBD,vTimestamps);

    // 拼合全局地图
    pcl::PointCloud<pcl::PointXYZRGBA> globalcloud, segmetcloud, segmetRefinecloud;
    // 注意我们的做法是先把图像转换至pcl的点云，进行姿态变换，最后存储成octomap
    // 因为octomap的颜色信息不是特别方便处理，所以采用了这种迂回的方式
    // 所以，如果不考虑颜色，那不必转成pcl点云，而可以直接使用octomap::Pointcloud结构
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGBA> statistical_filter(true);
    pcl::VoxelGrid<pcl::PointXYZRGBA> voxel;
    statistical_filter.setMeanK(30);
    statistical_filter.setStddevMulThresh(1.0);
    voxel.setLeafSize(0.01,0.01,0.01);
    //   boost::unordered_map<int,octomap::OcTree * > segMap;
    string folderPath = boost::filesystem::current_path().generic_string()+"/objshow/";
    QDir dir;
    if(!dir.exists(QString(folderPath.c_str()))){
        std::cout<<"file not ext"<<folderPath<<std::endl;
        dir.mkdir(QString(folderPath.c_str()));//创建所属类别的文件夹
    }
    string oriPath = boost::filesystem::current_path().generic_string()+"/objshow/origin/";
    if(!dir.exists(QString(oriPath.c_str()))){
        std::cout<<"file not ext"<<oriPath<<std::endl;
        dir.mkdir(QString(oriPath.c_str()));//创建所属类别的文件夹
    }
    string refPath = boost::filesystem::current_path().generic_string()+"/objshow/refine/";
    if(!dir.exists(QString(refPath.c_str()))){
        std::cout<<"file not ext"<<refPath<<std::endl;
        dir.mkdir(QString(refPath.c_str()));//创建所属类别的文件夹
    }
    float totalcus=0;
    size_t numT=0;
    size_t objtotal=0;
    size_t objSegRetain=0;
    for (auto frame:vstrTrackPoses){
        pcl::PointCloud<pcl::PointXYZRGBA> cloud;
        cout<<"converting "<<frame.first<<"th keyframe ..." <<endl;
        Eigen::Isometry3d& pose = frame.second;
        auto imgFile=vstrImageFilenamesRGBD.find(frame.first);
        string img_path=imgFile->second.first;
        cv::Mat rgb=cv::imread(DatasetPath+imgFile->second.first,cv::IMREAD_UNCHANGED);
        cv::Mat depth=cv::imread(DatasetPath+imgFile->second.second,cv::IMREAD_UNCHANGED);
        vector<ObjInfo*> vinfos;
        cv::Mat  mask,object_mask;
<<<<<<< HEAD
        ReadFile::imgread(DatasetPath,imgFile->first,imgFile->second.second,mask,object_mask,rgb,depth,vinfos,!useLCCP);
=======
        ReadFile::imgread(DatasetPath,imgFile->first,imgFile->second.second,mask,object_mask,rgb,depth,vinfos,useLCCP);
>>>>>>> a3115f6c65b33c0d712af46cfa3ac4ad0d401c4f
        cv::Mat personMask;
        ReadFile::generatePersonMask(personMask,vinfos);
        cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(51, 51));
        cv::dilate(personMask, personMask, element);
        pcl::PointCloud<pcl::PointXYZRGBA> orisegObjcloud,refsegObjcloud;
        for(auto info:vinfos){
            //obj pointcloud
            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr tempObj1( new pcl::PointCloud<pcl::PointXYZRGBA>() );
            pcl::transformPointCloud( *info->objCloud, *tempObj1, pose.matrix() );
            voxel.setInputCloud(tempObj1);
            voxel.filter(*tempObj1);
            orisegObjcloud+=*tempObj1;

            auto start = std::chrono::high_resolution_clock::now();
            ORBSLAM2::MarginVoxelLccp mvl( info->objrgbaCloud, info->expandROI , info->objMat, info->ojbROI,
                                          ReadFile::camera_fx, ReadFile::camera_fy, ReadFile::camera_cx, ReadFile::camera_cy);
            //                if (use_Origin_LCCP) {
            //                    mvl.setSuperVoxelWithoutMargin(color_importance, spatial_importance, normal_importance);
            //                }
            //                else {
            mvl.setSuperVoxelAdaptive(color_importance, spatial_importance, normal_importance);
            //                }
<<<<<<< HEAD
            mvl.createLccp(concavity_tolerance_threshold,smoothness_threshold,connect_MarginVoxel,K_connect);
=======
            mvl.createLccp(concavity_tolerance_threshold,smoothness_threshold,min_segment_size,connect_MarginVoxel);
>>>>>>> a3115f6c65b33c0d712af46cfa3ac4ad0d401c4f
//            auto objcloud= mvl.segmentObjPoinCloud();
            auto objcloud= mvl.segmentObjVoxelCloud();
            auto end = std::chrono::high_resolution_clock::now();
            totalcus += std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

            objtotal++;
            if(!objcloud->empty())
                objSegRetain++;

            //refine pointcloud
            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr tempObj( new pcl::PointCloud<pcl::PointXYZRGBA>() );
            pcl::transformPointCloud( *objcloud, *tempObj, pose.matrix() );
            voxel.setInputCloud(tempObj);
            voxel.filter(*tempObj);
            refsegObjcloud+=*tempObj;

            //            pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> segrgba(objcloud);
            //            viewer->addPointCloud(objcloud, segrgba, "segcloud", v1);
            //            viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "segcloud", v1);

            //            pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> orgrgba(info->objrgbaCloud);
            //            viewer->addPointCloud(info->objrgbaCloud, orgrgba, "origincloud", v2);
            //            viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "origincloud", v2);
            //            while (!viewer->wasStopped())
            //            {
            //                viewer->updatePointCloud(objcloud, segrgba,"segcloud");
            //                viewer->updatePointCloud(info->objrgbaCloud, orgrgba,"origincloud");
            //                if(nextSeg==true){
            //                    nextSeg=false;
            //                    break;
            //                }
            //                viewer->spinOnce(100);
            //            }
            //            viewer->removeAllPointClouds(v1);
            //            viewer->removeAllPointClouds(v2);
        }
        ++numT;
        if(!orisegObjcloud.empty())
        {
            pcl::io::savePCDFile(oriPath+frame.first+".pcd",orisegObjcloud);
        }
        if(!refsegObjcloud.empty())
        {
            pcl::io::savePCDFile(refPath+frame.first+".pcd",refsegObjcloud);
        }
        segmetcloud+=orisegObjcloud;
        segmetRefinecloud+=refsegObjcloud;
        for ( int m=0; m<depth.rows; m+=2 )
            for ( int n=0; n<depth.cols; n+=2 )
            {
                float d = depth.ptr<float> (m) [n];
                if (d < 0.05 || d > 6)
                    continue;

                if(personMask.ptr<uchar> (m) [n]>0){
                    continue;
                }
                pcl::PointXYZRGBA p;
                uchar* rgbdata = &rgb.ptr<uchar>(m)[n*3];
                uchar b = rgbdata[0];
                uchar g = rgbdata[1];
                uchar r = rgbdata[2];
                float z = d ;
                float x = (n - ReadFile::camera_cx) * z / ReadFile::camera_fx;
                float y = (m - ReadFile::camera_cy) * z / ReadFile::camera_fy;
                p.x = x; p.y = y; p.z = z;
                p.r = r; p.g = g; p.b = b;

                for(auto info:vinfos){
                    if(info->isInobj(m,n)){
                        {
                            //                            if(segMap.find(info->objClass)==segMap.end()){
                            //                                octomap::OcTree *tree=new octomap::OcTree( 0.02 );
                            //                                segMap.insert({info->objClass,tree});
                            //                            }
                            //                            Eigen::Vector3d v_3d(p.x, p.y ,p.z);
                            //                            v_3d=pose*v_3d;
                            //                            segMap[info->objClass]->updateNode(v_3d[0],v_3d[1],v_3d[2],info->score);
                            p.r = COLORS[info->objClass%19][0];
                            p.g = COLORS[info->objClass%19][1];
                            p.b = COLORS[info->objClass%19][2];
                        }
                        break;
                    }
                }
                if (d < 0.05 || d > 6)
                    continue;
                cloud.points.push_back( p );
            }
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr temp( new pcl::PointCloud<pcl::PointXYZRGBA>() );
        pcl::transformPointCloud( cloud, *temp, pose.matrix() );
        voxel.setInputCloud(temp);
        voxel.filter(*temp);
        globalcloud+=*temp;
    }
    std::cout << "函数运行时间，总："<<totalcus<<",每帧" << totalcus/numT <<",per obj" << totalcus/objtotal << "毫秒/frame"<<", 点云留存率"<< (float)objSegRetain/(float)objtotal<< std::endl;
    pcl::io::savePCDFile("segmentPointMap.pcd",segmetcloud);
    pcl::io::savePCDFile("refinesegmentPointMap.pcd",segmetRefinecloud);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr global(&segmetcloud),segcloud(&segmetRefinecloud);

    //   auto itseg=segMap.begin();
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> orgrgba(global);
    viewer->addPointCloud(global, orgrgba, "origincloud", v2);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "origincloud", v2);

    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> segrgba(segcloud);
    viewer->addPointCloud(segcloud, segrgba, "segcloud", v1);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "segcloud", v1);
    while (!viewer->wasStopped())
    {
        //        if(nextSeg==true){
        //            if(viewer->)
        //           viewer->removePointCloud("origincloud",v2);

        //        }
        viewer->spinOnce(100);
    }
    cout<<"done."<<endl;

    return 0;

}
