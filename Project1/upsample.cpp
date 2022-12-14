//#include <pcl/point_types.h>
//#include <pcl/io/pcd_io.h>
//#include <pcl/features/normal_3d.h>
//#include <pcl/kdtree/kdtree_flann.h>
//#include <pcl/surface/mls.h>
//#include <pcl/console/time.h>
//#include <pcl/visualization/cloud_viewer.h>
//#include <pcl/visualization/pcl_visualizer.h> //包含基本可视化类
//#include <boost/thread/thread.hpp>
//#include <pcl/point_cloud.h>
//#include <list>
//using namespace std;
//typedef pcl::PointXYZ point;
//typedef pcl::PointCloud<point> pointcloud;
//typedef pcl::PointCloud<pcl::PointXYZRGB> pointcloudRGB;
//
//int main(int argc, char **argv)
//{
//	pointcloud::Ptr cloud(new pointcloud);
//	pointcloudRGB::Ptr colorcloud(new pointcloudRGB);
//	pcl::io::loadPCDFile(argv[1], *colorcloud);
//	cout << "points size is:" << colorcloud->size() << endl;
//	for (pcl::PointXYZRGB pc : colorcloud->points) {
//		point p(pc.x, pc.y, pc.z);
//		cloud->push_back(p);
//	}
//	pcl::search::KdTree<point>::Ptr tree(new pcl::search::KdTree<point>);
//
//	//创建存储的mls对象
//	pcl::PointCloud<pcl::PointNormal> mls_points;
//	//   pcl::PointCloud<point> mls_points;
//
//	//创建mls对象
//	pcl::MovingLeastSquares<point, pcl::PointNormal> mls;
//
//	//   pcl::MovingLeastSquares<point,point> mls;
//
//	mls.setComputeNormals(true);
//	mls.setInputCloud(cloud);
//	mls.setPolynomialFit(true); //设置为true则在平滑过程中采用多项式拟合来提高精度
//	mls.setPolynomialOrder(2); //MLS拟合的阶数，默认是2
//	mls.setSearchMethod(tree);
//	mls.setSearchRadius(5.1);  //搜索半径
//
//	mls.process(mls_points);
//	pcl::PointCloud<pcl::PointNormal>::Ptr mls_points_normal(new pcl::PointCloud<pcl::PointNormal>);
//	mls_points_normal = mls_points.makeShared();
//
//	cout << "mls poits size is: " << mls_points.size() << endl;
//
//
//	boost::shared_ptr<pcl::visualization::PCLVisualizer> view(new pcl::visualization::PCLVisualizer("test"));
//	view->setBackgroundColor(0.0, 0, 0);
//	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> v(mls_points_normal, 0, 250, 0);
//	view->addPointCloud<pcl::PointNormal>(mls_points_normal, v, "sample");
//	view->addPointCloudNormals<pcl::PointNormal>(mls_points_normal, 10, 10, "normal");
//	view->addCoordinateSystem(1.0); //建立空间直角坐标系
//	view->spin();
//
//
//	// Save output
//	pcl::io::savePCDFile("mid-mls.pcd", mls_points);
//
//}