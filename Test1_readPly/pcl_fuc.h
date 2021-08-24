#include <pcl/features/normal_3d.h>//法线计算头文件
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/console/parse.h>
#include <pcl/io/obj_io.h>
#include <string>
#include <sstream>

//ply格式XYZRGB点云的载入函数
bool myLoadPly(const std::string fileName, 
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);

//ply格式XYZRGB点云进行可视化
bool myViewer(const std::string dataName,
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);

//ply格式XYZRGB点云输出
bool myWritePly(const std::string fileName,
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);

//计算ply格式XYZRGB点云法线并输出ply格式XYZRGBNoamal点云
bool myNormal(const std::string fileName,
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);

//多帧法线生成器
bool normalWriter(const int frames_beg, const int frame_end,
	const std::string data_Name, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);

//半径滤波算法
bool RadiusOutlierRemoval(const std::string file_Name, 
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
#pragma once
