/*************************************************************************
    > File Name: ReadFile.hpp
    > Author: Xu Wen Yu
    > Mail: xwy17671242087@163.com
    > Created Time: 2023-03-01
 ************************************************************************/

#ifndef READFILE_HPP
#define READFILE_HPP
#include<algorithm>
#include<fstream>
#include<chrono>
#include<string>
#include<iostream>
#include<iomanip>
// boost.format 字符串处理
#include <boost/format.hpp>

// 使用Eigen的Geometry模块处理3d运动
#include <Eigen/Core>
#include <Eigen/Geometry>

#include<unordered_map>
#include<map>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include<yaml-cpp/yaml.h>

using namespace std ;

float camera_scale_tum1  = 5000;
float camera_cx_tum1     = 318.643040;
float camera_cy_tum1     = 255.313989;
float camera_fx_tum1     = 517.306408;
float camera_fy_tum1     = 516.469215;

float camera_scale_tum2  = 5208;
float camera_cx_tum2     = 325.141442;
float camera_cy_tum2     = 249.701764;
float camera_fx_tum2     = 520.908620;
float camera_fy_tum2     = 521.007327;

float camera_scale_tum3  = 5000;
float camera_cx_tum3     = 320.1;
float camera_cy_tum3     = 247.6;
float camera_fx_tum3     = 535.4;
float camera_fy_tum3     = 539.2;

int COLORS[19][3]={
    {244,  67,  54},
    {233,  30,  99},
    {156,  39, 176},
    {103,  58, 183},
    { 63,  81, 181},
    { 33, 150, 243},
    {  3, 169, 244},
    {  2, 188, 212},
    {  1, 150, 136},
    { 76, 175,  80},
    {139, 195,  74},
    {205, 220,  57},
    {255, 235,  59},
    {255, 193,   7},
    {255, 152,   1},
    {255,  87,  34},
    {121,  85,  72},
    {158, 158, 158},
    { 96, 125, 139}};

class ObjInfo {
public:
    ObjInfo(cv::Rect ojbROIs, float scores, int objClasss) {
        ojbROI = ojbROIs;
        score = scores;
        objClass = objClasss;
    }
    cv::Rect ojbROI;
    cv::Rect expandROI;
    float score;
    int objClass;
    cv::Mat objMat;
    cv::Mat Rectrgb;
    cv::Mat Rectdepth;
    cv::Mat objedge;
    bool isInobj(int m,int n){
     if(ojbROI.contains(cv::Point(n,m)))
     {
         n-=ojbROI.tl().x;
         m-=ojbROI.tl().y;
         return objMat.ptr<uchar>(m)[n]>0;
     }
     else
         return false;
    }

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr objrgbaCloud;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr objCloud;
    //    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr objrgbaEdgeCloud;
    //    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr objMaskrgbaCloud;
};
struct CompareFrameID{
    bool operator()(const string &__t,const string &__u) const
    {

        double t=atof(__t.c_str()), u=atof(__u.c_str());
        return t < u;
    }
};
typedef std::map<string,std::pair<string,string>,CompareFrameID> FrameIDMap;
typedef std::map<string,Eigen::Isometry3d,CompareFrameID,
    Eigen::aligned_allocator<std::pair<const string, Eigen::Isometry3d> >> FrameTrackIDMap;

class ReadFile
{
public:
    ReadFile(){

    }
   static void setCameraPara(int TumNUm) {
        switch (TumNUm) {
        case 1:
            camera_scale=camera_scale_tum1;
            camera_cx   =camera_cx_tum1   ;
            camera_cy   =camera_cy_tum1   ;
            camera_fx   =camera_fx_tum1   ;
            camera_fy   =camera_fy_tum1   ;
            break;
        case 2:
            camera_scale=camera_scale_tum2  ;
            camera_cx   =camera_cx_tum2     ;
            camera_cy   =camera_cy_tum2     ;
            camera_fx   =camera_fx_tum2     ;
            camera_fy   =camera_fy_tum2     ;
            break;
        case 3:
            camera_scale=camera_scale_tum3 ;
            camera_cx   =camera_cx_tum3    ;
            camera_cy   =camera_cy_tum3    ;
            camera_fx   =camera_fx_tum3    ;
            camera_fy   =camera_fy_tum3    ;
            break;
        default:
            break;
        }

    }
    static void LoadImages(const string &strAssociationFilename, vector<string> &vstrImageFilenamesRGB,
                           vector<string> &vstrImageFilenamesD, vector<double> &vTimestamps);
    static void LoadImages(const string &strAssociationFilename, FrameIDMap &vstrImageFilenamesRGBD
                           , vector<string> &vTimestamps);
    static  void LoadTrack(const string &strTrackFilename, FrameTrackIDMap &vstrTrackPoses);

    static void imgread(string objpath, std::string Filename, std::string depthpath, cv::Mat &mask, cv::Mat &object_mask, cv::Mat &rgb, cv::Mat &depth, vector<ObjInfo*> &vinfos,bool isUseCanny=true);

    static void generatePersonMask(cv::Mat &personMask,  vector<ObjInfo*> &vinfos);

    static pcl::PointCloud<pcl::PointXYZRGBA>::Ptr generateMaskPointCloud(cv::Mat depth ,cv::Mat rgb,  cv::Mat mask,cv::Rect rect);

    static pcl::PointCloud<pcl::PointXYZRGBA>::Ptr generatePointCloud(cv::Mat depth ,cv::Mat rgb);

    static float camera_scale;
    static float camera_cx;
    static float camera_cy;
    static float camera_fx;
    static float camera_fy;
private:
    static void loadObjinfo(std::string infopath,cv::Mat object_mask, cv::Mat rgb,cv::Mat depth, vector<ObjInfo*> &vinfos);
    static void loadObjinfoWithEdge(std::string infopath,cv::Mat object_mask, cv::Mat rgb,cv::Mat depth,cv::Mat edgeimg, vector<ObjInfo*> &vinfos);
    static cv::Rect expandROI(cv::Rect ROI,cv::Size imgsz, int &offestX,int &offestY,int &offestW,int &offestH,float scale=0.15){
        assert(scale<1.0&&scale>=0);
        int meanside=((ROI.width+ROI.height)/2)*scale;
        cv::Rect expandR;
        expandR.x=ROI.x-meanside;
        expandR.x=expandR.x<0?0:expandR.x;
        offestX=ROI.x-expandR.x;

        expandR.y=ROI.y-meanside;
        expandR.y=expandR.y<0?0:expandR.y;
        offestY=ROI.y-expandR.y;

        expandR.width=ROI.width+meanside;
        expandR.width=expandR.width>imgsz.width?imgsz.width:expandR.width;
        offestW=ROI.width-expandR.width;

        expandR.height=ROI.height+meanside;
        expandR.height=expandR.height>imgsz.height?imgsz.height:expandR.height;
        offestH=ROI.height-expandR.height;
        return expandR;
    }
    static pcl::PointCloud<pcl::PointXYZRGBA>::Ptr generatePointCloudInRect(cv::Mat edge, cv::Mat rgb, cv::Mat depth, cv::Rect rect);
};

float ReadFile::camera_scale  = 5000;
float ReadFile::camera_cx     = 320.1;
float ReadFile::camera_cy     = 247.6;
float ReadFile::camera_fx     = 535.4;
float ReadFile::camera_fy     = 539.2;

void ReadFile::LoadImages(const string &strAssociationFilename, vector<string> &vstrImageFilenamesRGB, vector<string> &vstrImageFilenamesD, vector<double> &vTimestamps)
{
    ifstream fAssociation;
    fAssociation.open(strAssociationFilename.c_str());
    if(!fAssociation.good())
    {
        cout<<"strAssociationFilename:"<<strAssociationFilename<<endl;
        cout<<"fAssociation.rdstate():"<<fAssociation.rdstate()<<endl;
        return ;
    }
    while(!fAssociation.eof())
    {
        std::string s;
        std::getline(fAssociation,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            string sRGB, sD;
            ss >> t;
            vTimestamps.push_back(t);
            ss >> sRGB;
            vstrImageFilenamesRGB.push_back(sRGB);
            ss >> t;
            ss >> sD;
            vstrImageFilenamesD.push_back(sD);

        }
    }
}

void ReadFile::LoadImages(const string &strAssociationFilename, FrameIDMap &vstrImageFilenamesRGBD
                          , vector<string> &vTimestamps)
{
    ifstream fAssociation;
    fAssociation.open((strAssociationFilename+"associations.txt").c_str());
    if(!fAssociation.good())
    {
        fAssociation.open((strAssociationFilename+"associate.txt").c_str());
        if(!fAssociation.good())
        {
        cout<<"strAssociationFilename:"<<strAssociationFilename<<endl;
        cout<<"fAssociation.rdstate():"<<fAssociation.rdstate()<<endl;
        return ;
        }
    }
    while(!fAssociation.eof())
    {
        std::string s;
        std::getline(fAssociation,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            string t,deptht;
            string sRGB, sD;
            ss >> t;
            vTimestamps.push_back(t);
            ss >> sRGB;
            ss >> deptht;
            ss >> sD;
            auto its= vstrImageFilenamesRGBD.insert({t,{sRGB,sD}});

        }
    }
}

void ReadFile::LoadTrack(const string &strTrackFilename, FrameTrackIDMap &vstrTrackPoses)
{
    ifstream fAssociation;
    fAssociation.open(strTrackFilename.c_str());
    if(!fAssociation.good())
    {
        cout<<"strAssociationFilename:"<<strTrackFilename<<endl;
        cout<<"fAssociation.rdstate():"<<fAssociation.rdstate()<<endl;
        return ;
    }
    while(!fAssociation.eof())
    {
        std::string s;
        std::getline(fAssociation,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            string index_keyframe;
            ss>>index_keyframe;
            float data[7]; // 三个位置加一个姿态四元数x,y,z, w,ux,uy,uz
            for ( int i=0; i<7; i++ )
            {
                ss>>data[i];
//                cout<<data[i]<<" ";
            }
//            cout<<endl;
            if (fAssociation.fail()) break;
            // 注意这里的顺序。g2o文件四元数按 qx, qy, qz, qw来存
            // 但Eigen初始化按照qw, qx, qy, qz来做
            Eigen::Quaterniond q( data[6], data[3], data[4], data[5] );
            Eigen::Isometry3d t(q);
            t(0,3) = data[0]; t(1,3) = data[1]; t(2,3) = data[2];
            vstrTrackPoses.insert(std::make_pair(index_keyframe,t));
        }
    }
    fAssociation.close();
}

pcl::PointCloud<pcl::PointXYZRGBA>::Ptr ReadFile::generatePointCloudInRect(cv::Mat edge, cv::Mat rgb, cv::Mat depth, cv::Rect rect)
{
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr objrgbaEdge(new pcl::PointCloud<pcl::PointXYZRGBA>);
    for (int m = 0; m < depth.rows; m+=1)
    {
        for (int n = 0; n < depth.cols; n+=1)
        {
            float d = depth.ptr<float>(m)[n] ;
            if (d < 0.05 || d > 6 )
                continue;
            pcl::PointXYZRGBA p;
            p.z = d;
            p.x = (n+ rect.x - camera_cx) * p.z / camera_fx;//(p.x*kf->fx)/p.z+kf->cx
            p.y = (m + rect .y - camera_cy) * p.z / camera_fy;//(p.y*kf->fy)/p.z+kf->cy

            if (edge.ptr<uchar>(m)[n] > 0) {
                p.a = 1;
            }
            else {
                p.a = 0;
            }
            p.b = rgb.ptr<uchar>(m)[n * 3];
            p.g = rgb.ptr<uchar>(m)[n * 3 + 1];
            p.r = rgb.ptr<uchar>(m)[n * 3 + 2];
            objrgbaEdge->push_back(p);
        }
    }
    return objrgbaEdge;
}
void ReadFile::loadObjinfoWithEdge(string infopath, cv::Mat object_mask, cv::Mat rgb, cv::Mat depth, cv::Mat edgeimg, vector<ObjInfo *> &vinfos)
{
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

                rect = cv::Rect(0, maskheight, info->ojbROI.width, object_mask.size().height - maskheight );
                info->ojbROI.height=info->ojbROI.height!=rect.height?rect.height:rect.height;
            }
            else {
                rect = cv::Rect(0, maskheight, info->ojbROI.width, info->ojbROI.height);
            }
        }

        info->objMat = object_mask(rect).clone();
//        if(info->objClass==0)
//        {
//            cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT,cv::Size(25, 25));
//            cv::dilate(info->objMat,info->objMat,element);
//        }
        int ofx,ofy,ofw,ofh;
        info->expandROI=expandROI(info->ojbROI,rgb.size(),ofx,ofy,ofw,ofh);
        info->Rectrgb = rgb(info->ojbROI).clone();
        info->Rectdepth = depth(info->ojbROI).clone();
        info->objedge= info->objMat.mul(edgeimg(info->ojbROI));
        cv::Mat expandObjedge=cv::Mat::zeros(info->expandROI.size(),CV_8U);
        info->objedge.copyTo(expandObjedge(cv::Rect(ofx,ofy,info->ojbROI.width,info->ojbROI.height)));

//        cv::Mat tempma,tempma1;
//        cv::cvtColor(expandObjedge,tempma,cv::COLOR_GRAY2BGR);
//        cv::cvtColor(info->objedge,tempma1,cv::COLOR_GRAY2BGR);
//        tempma.copyTo(rgb(info->expandROI));
//        tempma1.copyTo(rgb(info->ojbROI));
//        cv::imshow("testexpand",rgb);
//        cv::imwrite("tempma.jpg",rgb);
//        cv::imshow("testexpand1",info->objedge);
//        cv::waitKey(0);

        info->objrgbaCloud=generatePointCloudInRect(expandObjedge,rgb(info->expandROI),depth(info->expandROI),info->expandROI);
        info->objCloud=generateMaskPointCloud(depth(info->ojbROI),rgb(info->ojbROI),info->objMat,info->ojbROI);
        maskheight += (info->ojbROI.height );
        vinfos.push_back(info);
    }
}

void ReadFile::loadObjinfo(string infopath, cv::Mat object_mask, cv::Mat rgb, cv::Mat depth, vector<ObjInfo *> &vinfos)
{
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

                rect = cv::Rect(0, maskheight, info->ojbROI.width, object_mask.size().height - maskheight );
                info->ojbROI.height=info->ojbROI.height!=rect.height?rect.height:rect.height;
            }
            else {
                rect = cv::Rect(0, maskheight, info->ojbROI.width, info->ojbROI.height);
            }
        }

        info->objMat = object_mask(rect).clone();
//        if(info->objClass==0)
//        {
//            cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT,cv::Size(25, 25));
//            cv::dilate(info->objMat,info->objMat,element);
//        }
        int ofx,ofy,ofw,ofh;
        info->expandROI=expandROI(info->ojbROI,rgb.size(),ofx,ofy,ofw,ofh);
        info->Rectrgb = rgb(info->ojbROI).clone();
        info->Rectdepth = depth(info->ojbROI).clone();
        info->objedge=cv::Mat::zeros(info->objMat.size(),CV_8U);
        cv::Mat expandObjedge=cv::Mat::zeros(info->expandROI.size(),CV_8U);
        info->objedge.copyTo(expandObjedge(cv::Rect(ofx,ofy,info->ojbROI.width,info->ojbROI.height)));

//        cv::Mat tempma,tempma1;
//        cv::cvtColor(expandObjedge,tempma,cv::COLOR_GRAY2BGR);
//        cv::cvtColor(info->objedge,tempma1,cv::COLOR_GRAY2BGR);
//        tempma.copyTo(rgb(info->expandROI));
//        tempma1.copyTo(rgb(info->ojbROI));
//        cv::imshow("testexpand",rgb);
//        cv::imwrite("tempma.jpg",rgb);
//        cv::imshow("testexpand1",info->objedge);
//        cv::waitKey(0);

        info->objrgbaCloud=generatePointCloudInRect(expandObjedge,rgb(info->expandROI),depth(info->expandROI),info->expandROI);
        info->objCloud=generateMaskPointCloud(depth(info->ojbROI),rgb(info->ojbROI),info->objMat,info->ojbROI);
        maskheight += (info->ojbROI.height);
        vinfos.push_back(info);
    }
}

void ReadFile::imgread(string objpath, std::string Filename, std::string depthpath,  cv::Mat &mask, cv::Mat &object_mask, cv::Mat &rgb, cv::Mat &depth, vector<ObjInfo*> &vinfos,bool isUseCanny) {
    std::string path = objpath+"/object-mask/" + Filename + ".png";
    object_mask = cv::imread(path, cv::IMREAD_UNCHANGED);//读取实例对象图像
    path =  objpath+"/rgb/" + Filename + ".png";
    rgb = cv::imread(path, cv::IMREAD_UNCHANGED);//读取彩色图像
    path = objpath+depthpath ;
    depth = cv::imread(path, cv::IMREAD_UNCHANGED);//读取深度图像
    depth.convertTo(depth, CV_32F);//转换深度图像单位
    depth /= camera_scale;//转换深度图像比例
    path = objpath+"/yaml/" + Filename + ".yml";//读取配置文件
    if(!object_mask.empty()){
        if(isUseCanny)
        {
            cv::Mat cn, gray;
            cv::cvtColor(rgb, gray, cv::COLOR_BGR2GRAY);
            cv::Canny(gray, cn, 50, 180);
            cv::Mat element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
            cv::dilate(cn, cn, element);
            loadObjinfoWithEdge(path, object_mask,rgb,depth,cn,vinfos);
        }else{
            loadObjinfo(path, object_mask,rgb,depth,vinfos);
        }
    }
}

void ReadFile::generatePersonMask(cv::Mat &personMask, vector<ObjInfo *> &vinfos)
{
    personMask=cv::Mat::zeros(480,640,CV_8UC1);
    for(auto info:vinfos){
        if(info->objClass==0){
            info->objMat.copyTo(personMask(info->ojbROI));
        }
    }
//    cv::imshow("personMask",personMask);
//    cv::waitKey(0);
}

pcl::PointCloud<pcl::PointXYZRGBA>::Ptr ReadFile::generateMaskPointCloud(cv::Mat depth, cv::Mat rgb, cv::Mat mask, cv::Rect rect)
{
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr objcloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
    for (int m = 0; m < depth.rows; m+=1)
    {
        for (int n = 0; n < depth.cols; n+=1)
        {
            float d = depth.ptr<float>(m)[n] ;
            uchar c = mask.ptr<uchar>(m)[n];
            if (d < 0.05 || d > 6|| c<1 )
                continue;

            pcl::PointXYZRGBA p;
            p.z = d;
            p.x = (n+ rect.x - camera_cx) * p.z / camera_fx;//(p.x*kf->fx)/p.z+kf->cx
            p.y = (m + rect .y - camera_cy) * p.z / camera_fy;//(p.y*kf->fy)/p.z+kf->cy

            p.b = rgb.ptr<uchar>(m)[n * 3];
            p.g = rgb.ptr<uchar>(m)[n * 3 + 1];
            p.r = rgb.ptr<uchar>(m)[n * 3 + 2];
            objcloud->push_back(p);
        }
    }
    return objcloud;
}

pcl::PointCloud<pcl::PointXYZRGBA>::Ptr ReadFile::generatePointCloud(cv::Mat depth, cv::Mat rgb )
{
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr objcloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
    for (int m = 0; m < depth.rows; m+=1)
    {
        for (int n = 0; n < depth.cols; n+=1)
        {
            float d = depth.ptr<float>(m)[n] ;
            if (d < 0.05 || d > 6 )
                continue;

            pcl::PointXYZRGBA p;
            p.z = d;
            p.x = (n - camera_cx) * p.z / camera_fx;//(p.x*kf->fx)/p.z+kf->cx
            p.y = (m - camera_cy) * p.z / camera_fy;//(p.y*kf->fy)/p.z+kf->cy

            p.b = rgb.ptr<uchar>(m)[n * 3];
            p.g = rgb.ptr<uchar>(m)[n * 3 + 1];
            p.r = rgb.ptr<uchar>(m)[n * 3 + 2];
            objcloud->push_back(p);
        }
    }
    return objcloud;
}



#endif
