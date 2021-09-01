#include <pcl/features/normal_3d.h>//法线计算头文件
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/console/parse.h>
#include <pcl/io/obj_io.h>
#include <string>
#include <sstream>
#include<pcl/point_cloud.h>
#include<pcl/octree/octree_search.h>
#include<vector>
#include <pcl/surface/mls.h>//平滑操作相关头文件
#include <iostream> //标准输入输出流
#include <pcl/io/pcd_io.h>//PCL的PCD格式文件的输入输出头文件
#include <pcl/io/ply_io.h>//PCL的PLY格式文件的输入输出头文件
#include <pcl/point_types.h>//PCL中支持的点类型文件
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>//可视化头文件
#include <pcl/features/normal_3d.h>//法线计算头文件
#include <pcl/filters/radius_outlier_removal.h>  //滤波相关
#include <pcl/filters/statistical_outlier_removal.h>//统计学滤波
#include <pcl/surface/mls.h>//平滑操作相关头文件
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/console/parse.h>
#include <pcl/io/obj_io.h>
#include <string>
#include <sstream>
#include<pcl/point_cloud.h>
#include<pcl/octree/octree_search.h>
#include<vector>
#include<math.h>
//#include<iostream>
#include<fstream>
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

bool GridSegment(const std::string file_Name, 
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, const int grid_size);
#pragma once

//点云表面的平滑处理
pcl::PointCloud<pcl::PointXYZRGB>::Ptr SurfaceSmooth(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, const int r, const int p);

//点云坐标的四舍五入取整
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_Int(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);

//输出xyz数据的点云
bool myWriteXYZPly(const std::string fileName, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

//对输入的点云提取其坐标信息输出为没有颜色的新点云
pcl::PointCloud<pcl::PointXYZ>::Ptr DeleteRGB(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);

//对xyz格式点云转ply格式输出
pcl::PointCloud < pcl::PointXYZ>::Ptr xyz_to_ply(char* fname);

//对输入的xyzply点云加上空的颜色信息（用于质量评测）
pcl::PointCloud<pcl::PointXYZRGB>::Ptr plyxyz_to_xyzrgb(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);