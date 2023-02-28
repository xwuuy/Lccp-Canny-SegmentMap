# lccp & Canny SegmentMap 
基于lccp与边缘检测的三维点云分割方法。  
主要内容：改善二维实例分割生成的点云，并生成分割点云地图。
## 实验效果图
    分割结果对比动图：
![SegmentMap项目演示 00_00_00-00_00_30](https://user-images.githubusercontent.com/51278459/219853704-37019f53-a3d0-427b-9f5e-29ecdfa8571a.gif "分割结果对比视频")


    Ours分割结果图：
![VmjjUG80PE](https://user-images.githubusercontent.com/51278459/219712001-74d56ce4-d69e-4c99-97ee-c86031abf2ee.png "Ours分割结果图")

    LCCP分割结果图 ：
![ZVWiJitcQp](https://user-images.githubusercontent.com/51278459/219844418-508c382b-cc2e-4356-a837-24c292d0b58b.png "LCCP分割结果图")

    原始分割结果图：
![CloudCompare_XShUvUT8KK](https://user-images.githubusercontent.com/51278459/219712022-b9059be3-0e34-4766-8d82-43a8bffe3cbe.png "原始分割结果图")

## 环境概览
### 测试环境
系统：win10  
编译器：MSVC2017 64bit、cmake 3.5  
第三方库：pcl 1.9.1、boost 1.6.8、opencv 4.2.0、yaml 0.6.0  
实例分割网络：YOLACT++，权重：yolact_plus_resnet50_54_800000  

### 测试数据集
TUM数据集 rgbd_dataset_freiburg1_room序列  
百度网盘 链接：https://pan.baidu.com/s/1mh8ELvMx0nrTP37GEwXyNA  
提取码：in26

### 使用方法：
编译文件

    首先编译yaml_cpp  
    cd yaml_cpp  
    mkdir build  
    cd build  
    cmake [-G generator] [-DBUILD_SHARED_LIBS=ON|OFF] ..  
    make  
    编译本工程  
    mkdir build  
    cd build
    cmake [-G generator] [-DBUILD_SHARED_LIBS=ON|OFF] ..  
    make  
    
执行命令  

    "SegmentMap.exe --Dataset" 数据集目录 -t 数据集目录/KeyFrameTrajectory.txt -c 0.2 -s 0.4 -n 0.6 -C 15 -S 0.01"
