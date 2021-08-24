#include <pcl/features/normal_3d.h>//���߼���ͷ�ļ�
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/console/parse.h>
#include <pcl/io/obj_io.h>
#include <string>
#include <sstream>

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
#pragma once
