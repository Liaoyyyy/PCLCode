#include <pcl/features/normal_3d.h>//���߼���ͷ�ļ�
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/console/parse.h>
#include <pcl/io/obj_io.h>
#include <string>
#include <sstream>
#include<pcl/point_cloud.h>
#include<pcl/octree/octree_search.h>
#include<vector>
#include <pcl/surface/mls.h>//ƽ���������ͷ�ļ�
#include <iostream> //��׼���������
#include <pcl/io/pcd_io.h>//PCL��PCD��ʽ�ļ����������ͷ�ļ�
#include <pcl/io/ply_io.h>//PCL��PLY��ʽ�ļ����������ͷ�ļ�
#include <pcl/point_types.h>//PCL��֧�ֵĵ������ļ�
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>//���ӻ�ͷ�ļ�
#include <pcl/features/normal_3d.h>//���߼���ͷ�ļ�
#include <pcl/filters/radius_outlier_removal.h>  //�˲����
#include <pcl/filters/statistical_outlier_removal.h>//ͳ��ѧ�˲�
#include <pcl/surface/mls.h>//ƽ���������ͷ�ļ�
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
//ply��ʽXYZRGB���Ƶ����뺯��
bool myLoadPly(const std::string fileName, 
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);

//ply��ʽXYZRGB���ƽ��п��ӻ�
bool myViewer(const std::string dataName,
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);

//ply��ʽXYZRGB�������
bool myWritePly(const std::string fileName,
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);

//����ply��ʽXYZRGB���Ʒ��߲����ply��ʽXYZRGBNoamal����
bool myNormal(const std::string fileName,
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);

//��֡����������
bool normalWriter(const int frames_beg, const int frame_end,
	const std::string data_Name, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);

//�뾶�˲��㷨
bool RadiusOutlierRemoval(const std::string file_Name, 
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);

bool GridSegment(const std::string file_Name, 
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, const int grid_size);
#pragma once

//���Ʊ����ƽ������
pcl::PointCloud<pcl::PointXYZRGB>::Ptr SurfaceSmooth(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, const int r, const int p);

//�����������������ȡ��
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_Int(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);

//���xyz���ݵĵ���
bool myWriteXYZPly(const std::string fileName, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

//������ĵ�����ȡ��������Ϣ���Ϊû����ɫ���µ���
pcl::PointCloud<pcl::PointXYZ>::Ptr DeleteRGB(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);

//��xyz��ʽ����תply��ʽ���
pcl::PointCloud < pcl::PointXYZ>::Ptr xyz_to_ply(char* fname);

//�������xyzply���Ƽ��Ͽյ���ɫ��Ϣ�������������⣩
pcl::PointCloud<pcl::PointXYZRGB>::Ptr plyxyz_to_xyzrgb(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
//�������һ�����ĵ��ƣ������������������ϡ�
pcl::PointCloud<pcl::PointXYZRGB>::Ptr LinkPoint(std::vector<std::string> FileName);

//��������������kd�����зָÿ��1024���㣬�ֱ𽫶��ڵ������ͷ���������Ӧ����ֵ���б���

//bool kd_seg_xyzn(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud) {
//
//}
//�԰˲���Ϊ�����Ե��ƽ��зָ�
bool octree_Seg(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);