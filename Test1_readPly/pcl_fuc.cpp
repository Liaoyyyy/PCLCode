#include <iostream> //标准输入输出流
#include <pcl/io/pcd_io.h>//PCL的PCD格式文件的输入输出头文件
#include <pcl/io/ply_io.h>//PCL的PLY格式文件的输入输出头文件
#include <pcl/point_types.h>//PCL中支持的点类型文件
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>//可视化头文件
#include <pcl/features/normal_3d.h>//法线计算头文件
#include <pcl/filters/radius_outlier_removal.h>  //滤波相关
#include <pcl/filters/statistical_outlier_removal.h>//统计学滤波
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/console/parse.h>
#include <pcl/io/obj_io.h>
#include <string>
#include <sstream>
#include<pcl/point_cloud.h>
#include<pcl/octree/octree_search.h>
#include<vector>
#include<math.h>

//输入点云
bool myLoadPly(const std::string fileName, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) {
	if ((pcl::io::loadPLYFile<pcl::PointXYZRGB>(fileName, *cloud) < 0)) {
		std::cout << "文件读取失败！" << std::endl;
		return false;
	}
	else {
		std::cout << "文件读取成功！" << std::endl;
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
bool myNormal(const std::string fileName, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) {

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

//循环多帧点云法线输出代码
bool normalWriter(const int frames_beg, const int frame_end,
				const std::string data_Name, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) {

	std::cout << "==========法线生成器开始运行===========" << std::endl;
	for (int i = frames_beg; i <= frame_end; i++) {
		std::cout << ".............第" << i << "帧............." << std::endl;
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new  pcl::PointCloud<pcl::PointXYZRGB>);
		std::string frameNum = std::to_string(i);
		std::string fileNameOri = data_Name + frameNum + ".ply";
		std::string fileNameNor = data_Name+"_normal" + frameNum + ".ply";
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
	std::cout << "==============法线生成器运行结束==============" << std::endl;
	return true;
}

//点云的离群点去除算法
bool RadiusOutlierRemoval(const std::string file_Name, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) {

 		
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filter(new pcl::PointCloud<pcl::PointXYZRGB>);
		//半径滤波
		//pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> sor;
		//统计学滤波
		pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
		sor.setInputCloud(cloud);
		//sor.setRadiusSearch(50);//半径滤波参数
		//sor.setMinNeighborsInRadius(5);//半径滤波参数
		 // 设置过滤邻域K
		sor.setMeanK(16);
		// 设置标准差的系数, 值越大，丢掉的点越少
		sor.setStddevMulThresh(5.0f);
		sor.filter(*cloud_filter);
		sor.setNegative(true);
		//sor.filter(*cloud_filter);
		std::string FileName = file_Name + "_filted.ply";

		myWritePly(FileName, cloud_filter);
		return true;

}

bool GridSegment(const std::string file_Name, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, const int grid_size) {
	//对传入的点云数据依据坐标信息进行等间隔的网格分割
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr gridcloud(new  pcl::PointCloud<pcl::PointXYZRGB>);
	int point_Count = 0;
	int x_max = 0;
	int y_max = 0;
	int z_max = 0;
	std::vector<int> index;
	index.reserve(1000000);
	//先计算输入点云的坐标区间：
	for (int i = 0; i < cloud->size(); i++) {
		x_max = ((cloud->points[i].x) > x_max ? (cloud->points[i].x) : x_max);
		y_max = ((cloud->points[i].y) > y_max ? (cloud->points[i].y) : y_max);
		z_max = ((cloud->points[i].z) > z_max ? (cloud->points[i].z) : z_max);
	}
	//计算网格划分数量
	int x_size = floor(x_max / grid_size);
	int y_size = floor(y_max / grid_size);
	int z_size = floor(z_max / grid_size);
	int grid_Num = x_size * y_size*z_size;
	//接下来对每一个网格的点云做分别的输出

	for (int i = 0; i < cloud->size(); i++) {
		std::cout << "范围：0-256" << std::endl;

		if (cloud->points[i].x < 128 && cloud->points[i].y < 256 && cloud->points[i].z <128 ) {
			point_Count++;
			index.push_back(i);
			std::cout << "找到" << point_Count << "个点" << std::endl;
		}

	}
	gridcloud->resize(point_Count);
	for (int j = 0; j < index.size(); j++) {
		pcl::PointXYZRGB gridpoint;
		gridpoint.x = cloud->points[index[j]].x;
		gridpoint.y = cloud->points[index[j]].y;
		gridpoint.z = cloud->points[index[j]].z;
		gridpoint.rgb = cloud->points[index[j]].rgb;
		gridpoint.a = cloud->points[index[j]].a;
		gridcloud->push_back(gridpoint);
	}
	myWritePly(file_Name, gridcloud);
	return true;
}


