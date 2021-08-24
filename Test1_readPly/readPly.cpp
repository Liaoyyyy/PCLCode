#include <iostream> //标准输入输出流
#include <pcl/io/pcd_io.h>//PCL的PCD格式文件的输入输出头文件
#include <pcl/io/ply_io.h>//PCL的PLY格式文件的输入输出头文件
#include <pcl/point_types.h>//PCL中支持的点类型文件
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>//可视化头文件
#include <pcl/features/normal_3d.h>//法线计算头文件
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/console/parse.h>
#include <pcl/io/obj_io.h>
#include <string>
#include <sstream>

//输入点云
bool myLoadPly(const std::string fileName, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) {
	if ((pcl::io::loadPLYFile<pcl::PointXYZRGB>(fileName, *cloud) < 0)) {
		//std::cout << "文件读取失败！" << std::endl;
		return false;
	}
	else {
		//std::cout << "文件读取成功！" << std::endl;
		return false;

	};
	//写法二
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOrigin(new pcl::PointCloud<pcl::PointXYZ>);
	//pcl::PLYReader reader;
	//reader.read(filename, *cloud_normals);
}

//点云进行可视化
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

//输出点云
bool myWritePly(const std::string fileName, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) {
	if (pcl::io::savePLYFile<pcl::PointXYZRGB>(fileName, *cloud) < 0)
	{
		//std::cout << "输出失败！" << endl;
		return false;
	}
	else {
		//std::cout << "输出成功！" << endl;
		return true;
	}
	
}

//计算法线并输出
bool myNormal(const std::string fileName,pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) {

	////计算法线
	//pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> n;
	//pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	////建立kdtree来进行近邻点集搜索
	//pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
	////为kdtree添加点运数据
	//tree->setInputCloud(cloud);
	//n.setInputCloud(cloud);
	//n.setSearchMethod(tree);
	////点云法向计算时，需要所搜的近邻点大小
	//n.setKSearch(20);
	////开始进行法向计算
	//n.compute(*normals);
	//pcl::io::savePLYFile<pcl::Normal>(fileName, *normals);
	//法二
	//string inputFile = "bunny.ply", outputFile;
	//PointCloud<PointXYZ>::Ptr source_cloud(new PointCloud<PointXYZ>());
	//if (pcl::io::loadPLYFile(inputFile, *source_cloud) < 0) {
	//	std::cout << "Error loading point cloud " << std::endl;
	//	return -1;
	//}//判别条件是读取ply文件失败
	//std::cout << "source_cloud_size" << source_cloud->points.size() << endl;

	//PointCloud<PointXYZ>::Ptr filtered_cloud(new PointCloud<PointXYZ>())
	// Create the normal estimation class, and pass the input dataset to it
	pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
	ne.setInputCloud(cloud);
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
	tree->setInputCloud(cloud);
	ne.setSearchMethod(tree);
	ne.setKSearch(20);
	// Output datasets
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
	ne.compute(*cloud_normals);
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr pncloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	concatenateFields(*cloud, *cloud_normals, *pncloud);
	//stringstream ss;
	//ss << "ne_K_" << 20 << "_pn_" << inputFile;
	//outputFile = ss.str();
	//outputFile.replace(outputFile.length() - 3, 3, "ply");
	pcl::io::savePLYFile(fileName, *pncloud);
	/*pcl::visualization::PCLVisualizer viewer;
	viewer.setBackgroundColor(0.0, 0.0, 0.5);
	viewer.addPointCloudNormals<pcl::PointXYZ,pcl::Normal>(source_cloud, cloud_normals);
	while (!viewer.wasStopped()){
		viewer.spinOnce();
	}
	std::cout << "cloud_normals.size" << cloud_normals->points.size() << " " << "source_cloud.size" << source_cloud->points.size() << endl;
	for (int i = 0; i < cloud_normals->points.size(); i++){
		cout << cloud_normals->points[i] << endl;
	}*/
	// cloud_normals->points.size () should have the same size as the input cloud->points.size ()
	return 0;
	//return true;
}
int main() {

	//创建共享指针并实例化指针
	std::cout << "===========生成点云法线文件======================" << std::endl;
	for (int i = 1051; i <= 1100; i++) {
		std::cout << "........第" << i << "帧........"<< std::endl;
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new  pcl::PointCloud<pcl::PointXYZRGB>);
		std::string frameNum = std::to_string(i);
		std::string fileNameOri = "longdress_vox10_" + frameNum + ".ply";
		std::string fileNameNor = "longdres_vox10_normal_" + frameNum + ".ply";
		//读取ply文件
		bool isload = myLoadPly(fileNameOri, cloud);
		//可视化点云
		//bool isView = myViewer("longDress_1051", cloud);
		//输出点云数据
		//bool isSave = myWritePly("re_longdress.ply", cloud);
		//计算法线并输出
		myNormal(fileNameNor, cloud);
		cloud->clear();
	}
	//int frameNumInt = 1051;
	
	system("pause");
	return 0;
}
