# lccp & Canny SegmentMap
基于lccp与边缘检测的三维点云分割方法
改善二维实例分割生成的点云。
## 实验效果图
    Ours分割结果：
![OursSegmentPointCloud 00_00_00-00_00_30](https://user-images.githubusercontent.com/51278459/219711628-438bc954-5d7e-4409-98dc-9d3585e894df.gif "Ours分割结果")

    LCCP分割结果：
![LccpSegmentPointCloud 00_00_00-00_00_30](https://user-images.githubusercontent.com/51278459/219711607-eccbc0c1-4d69-40f1-b527-d549ba2ca589.gif "LCCP分割结果")

    原始分割结果：
![SegmentPointCloud 00_00_00-00_00_30](https://user-images.githubusercontent.com/51278459/219711635-e0e60b12-3ea3-4de4-a443-cca95a673ada.gif "原始分割结果")

    Ours分割结果图：
![VmjjUG80PE](https://user-images.githubusercontent.com/51278459/219712001-74d56ce4-d69e-4c99-97ee-c86031abf2ee.png "Ours分割结果图")

    LCCP分割结果图 ：
![ZVWiJitcQp](https://user-images.githubusercontent.com/51278459/219844418-508c382b-cc2e-4356-a837-24c292d0b58b.png "LCCP分割结果图")

    原始分割结果图：
![CloudCompare_XShUvUT8KK](https://user-images.githubusercontent.com/51278459/219712022-b9059be3-0e34-4766-8d82-43a8bffe3cbe.png)

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
执行

    "SegmentMap.exe --Dataset" 数据集目录 -t 数据集目录/KeyFrameTrajectory.txt -c 0.2 -s 0.4 -n 0.6 -C 15 -S 0.01"
