#include <iostream> //标准输入输出流
#include <pcl/io/pcd_io.h>//PCL的PCD格式文件的输入输出头文件
#include <pcl/io/ply_io.h>//PCL的PLY格式文件的输入输出头文件
#include <pcl/point_types.h>//PCL中支持的点类型文件
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>//可视化头文件

//对输入点云的过程进行一个包装。
bool myLoadPly(const std::string fileName, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) {
	if ((pcl::io::loadPLYFile<pcl::PointXYZRGB>(fileName, *cloud) < 0)) {
		std::cout << "文件读取失败！" << std::endl;
		return false;
	}
	else {
		std::cout << "文件读取成功！" << std::endl;
		return false;

	};
}
//对点云进行可视化
bool myViewer(const std::string dataName, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) {
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer(dataName));
	//背景色
	viewer->setBackgroundColor(0, 0, 0);
	//数据
	viewer->addPointCloud<pcl::PointXYZRGB>(cloud, dataName);
	//点大小
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, dataName);
	//添加坐标系（即红绿蓝三色轴，放置在原点）
	//viewer->addCoordinateSystem(3.0);//3.0指轴的长度
	//viewer->addCoordinateSystem (3.0,1,2,3);一个重载函数，3.0指轴的长度，放置在（1，2，3）位置
	//初始化默认相机参数
	viewer->initCameraParameters();
	while (!viewer->wasStopped())
	{
		viewer->spinOnce();
		
	}
	return true;
}
int main() {

	//创建共享指针并实例化指针
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new  pcl::PointCloud<pcl::PointXYZRGB>);
	std::string fileName = "longdress_vox10_1051.ply";
	//读取ply文件
	bool isload = myLoadPly(fileName, cloud);
	//可视化点云
	//bool isView = myViewer("longDress_1051", cloud);
	system("pause");
	return 0;
}
