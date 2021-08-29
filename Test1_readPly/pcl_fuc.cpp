#include <iostream> //��׼���������
#include <pcl/io/pcd_io.h>//PCL��PCD��ʽ�ļ����������ͷ�ļ�
#include <pcl/io/ply_io.h>//PCL��PLY��ʽ�ļ����������ͷ�ļ�
#include <pcl/point_types.h>//PCL��֧�ֵĵ������ļ�
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>//���ӻ�ͷ�ļ�
#include <pcl/features/normal_3d.h>//���߼���ͷ�ļ�
#include <pcl/filters/radius_outlier_removal.h>  //�˲����
#include <pcl/filters/statistical_outlier_removal.h>//ͳ��ѧ�˲�
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/console/parse.h>
#include <pcl/io/obj_io.h>
#include <string>
#include <sstream>
#include<pcl/point_cloud.h>
#include<pcl/octree/octree_search.h>
#include<vector>
#include<math.h>

//�������
bool myLoadPly(const std::string fileName, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) {
	if ((pcl::io::loadPLYFile<pcl::PointXYZRGB>(fileName, *cloud) < 0)) {
		std::cout << "�ļ���ȡʧ�ܣ�" << std::endl;
		return false;
	}
	else {
		std::cout << "�ļ���ȡ�ɹ���" << std::endl;
		return false;

	};
	//д����
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOrigin(new pcl::PointCloud<pcl::PointXYZ>);
	//pcl::PLYReader reader;
	//reader.read(filename, *cloud_normals);
}

//���ƽ��п��ӻ�
bool myViewer(const std::string dataName, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) {
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer(dataName));
	//����ɫ
	viewer->setBackgroundColor(0, 0, 0);
	//����
	viewer->addPointCloud<pcl::PointXYZRGB>(cloud, dataName);
	//���С
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, dataName);
	//�������ϵ������������ɫ�ᣬ������ԭ�㣩
	//viewer->addCoordinateSystem(3.0);//3.0ָ��ĳ���
	//viewer->addCoordinateSystem (3.0,1,2,3);һ�����غ�����3.0ָ��ĳ��ȣ������ڣ�1��2��3��λ��
	//��ʼ��Ĭ���������
	viewer->initCameraParameters();
	while (!viewer->wasStopped())
	{
		viewer->spinOnce();

	}
	return true;
}

//�������
bool myWritePly(const std::string fileName, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) {
	if (pcl::io::savePLYFile<pcl::PointXYZRGB>(fileName, *cloud) < 0)
	{
		//std::cout << "���ʧ�ܣ�" << endl;
		return false;
	}
	else {
		//std::cout << "����ɹ���" << endl;
		return true;
	}

}

//���㷨�߲����
bool myNormal(const std::string fileName, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) {

	////���㷨��
	//pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> n;
	//pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	////����kdtree�����н��ڵ㼯����
	//pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
	////Ϊkdtree��ӵ�������
	//tree->setInputCloud(cloud);
	//n.setInputCloud(cloud);
	//n.setSearchMethod(tree);
	////���Ʒ������ʱ����Ҫ���ѵĽ��ڵ��С
	//n.setKSearch(20);
	////��ʼ���з������
	//n.compute(*normals);
	//pcl::io::savePLYFile<pcl::Normal>(fileName, *normals);
	//����
	//string inputFile = "bunny.ply", outputFile;
	//PointCloud<PointXYZ>::Ptr source_cloud(new PointCloud<PointXYZ>());
	//if (pcl::io::loadPLYFile(inputFile, *source_cloud) < 0) {
	//	std::cout << "Error loading point cloud " << std::endl;
	//	return -1;
	//}//�б������Ƕ�ȡply�ļ�ʧ��
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

//ѭ����֡���Ʒ����������
bool normalWriter(const int frames_beg, const int frame_end,
				const std::string data_Name, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) {

	std::cout << "==========������������ʼ����===========" << std::endl;
	for (int i = frames_beg; i <= frame_end; i++) {
		std::cout << ".............��" << i << "֡............." << std::endl;
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new  pcl::PointCloud<pcl::PointXYZRGB>);
		std::string frameNum = std::to_string(i);
		std::string fileNameOri = data_Name + frameNum + ".ply";
		std::string fileNameNor = data_Name+"_normal" + frameNum + ".ply";
		//��ȡply�ļ�
		bool isload = myLoadPly(fileNameOri, cloud);
		//���ӻ�����
		//bool isView = myViewer("longDress_1051", cloud);
		//�����������
		//bool isSave = myWritePly("re_longdress.ply", cloud);
		//���㷨�߲����
		myNormal(fileNameNor, cloud);
		cloud->clear();
	}
	//int frameNumInt = 1051;
	std::cout << "==============�������������н���==============" << std::endl;
	return true;
}

//���Ƶ���Ⱥ��ȥ���㷨
bool RadiusOutlierRemoval(const std::string file_Name, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) {

 		
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filter(new pcl::PointCloud<pcl::PointXYZRGB>);
		//�뾶�˲�
		//pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> sor;
		//ͳ��ѧ�˲�
		pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
		sor.setInputCloud(cloud);
		//sor.setRadiusSearch(50);//�뾶�˲�����
		//sor.setMinNeighborsInRadius(5);//�뾶�˲�����
		 // ���ù�������K
		sor.setMeanK(16);
		// ���ñ�׼���ϵ��, ֵԽ�󣬶����ĵ�Խ��
		sor.setStddevMulThresh(5.0f);
		sor.filter(*cloud_filter);
		sor.setNegative(true);
		//sor.filter(*cloud_filter);
		std::string FileName = file_Name + "_filted.ply";

		myWritePly(FileName, cloud_filter);
		return true;

}

bool GridSegment(const std::string file_Name, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, const int grid_size) {
	//�Դ���ĵ�����������������Ϣ���еȼ��������ָ�
	double x_max = 0;
	double y_max = 0;
	double z_max = 0;
	//std::vector<int> index;
	//index.reserve(1000000);
	//�ȼ���������Ƶ��������䣺
	for (int i = 0; i < cloud->size(); i++) {
		x_max = ((cloud->points[i].x) > x_max ? (cloud->points[i].x) : x_max);
		y_max = ((cloud->points[i].y) > y_max ? (cloud->points[i].y) : y_max);
		z_max = ((cloud->points[i].z) > z_max ? (cloud->points[i].z) : z_max);
	}
	//�������񻮷�����
	int x_size = floor(x_max / grid_size);
	int y_size = floor(y_max / grid_size);
	int z_size = floor(z_max / grid_size);
	int grid_Num = x_size * y_size*z_size;
	int x_cur_size = ceil(x_max / x_size);
	int y_cur_size = ceil(y_max / y_size);
	int z_cur_size = ceil(z_max / z_size);
	//��������ÿһ������ĵ������ֱ�����
	int cur_grid_num = 1;
	//���зָ�
	for (int i = 0; i < x_size; i++) {
		for (int j = 0; j < y_size; j++) {
			for (int k = 0; k < z_size; k++) {
				std::cout << "����" << grid_Num << "��������ƣ���ǰ�����" << cur_grid_num << "������." << std::endl;
				//�������ƶ������ڴ�ż���õ����������
				int x_right = (i+1)* x_cur_size;
				int x_left = i*x_cur_size;
				int y_right = (j+1) * y_cur_size;
				int y_left = j*y_cur_size;
				int z_right = (k+1) * z_cur_size;
				int z_left = k*z_cur_size;
				pcl::PointCloud<pcl::PointXYZRGB>::Ptr gridcloud(new  pcl::PointCloud<pcl::PointXYZRGB>);
				gridcloud->reserve(500000);
				//�����е����ѭ��
				int point_Count = 0;
				for (int index = 0; index < cloud->size(); index++) {
					int x = cloud->points[index].x;
					int y = cloud->points[index].y;
					int z = cloud->points[index].z;
					bool isx = (x > x_left) && (x <= x_right);
					bool isy = (y > y_left) && (y <= y_right);
					bool isz = (z > z_left) && (z <= z_right);
					if ( isx&&isy&&isz
						//(x>x_right)&&(x<=x_left)&&(y>y_left)&&(y<=y_left)&&(z>z_right)&&(z<=z_left)
						/*(x > (i*x_cur_size)) && (cloud->points[i].x <= ((i + 1)*x_cur_size)))
						&& ((cloud->points[i].y > (j*y_cur_size)) && (cloud->points[i].y <= ((j + 1)*y_cur_size)))
						&& (cloud->points[i].z > (k*z_cur_size)) && (cloud->points[i].z <= ((k + 1)*z_cur_size))*/
						) {
						pcl::PointXYZRGB gridpoint;
						gridpoint.x = cloud->points[index].x;
						gridpoint.y = cloud->points[index].y;
						gridpoint.z = cloud->points[index].z;
						gridpoint.rgb = cloud->points[index].rgb;
						gridpoint.a = cloud->points[index].a;
						gridcloud->push_back(gridpoint);
						std::cout << "��" << cur_grid_num << "�����ȡ����" << point_Count << "����" << std::endl;

						//gridcloud->push_back(cloud->points[i]);
						point_Count++;

					}
				}
				//�������õ��ĵ��ơ�
				if (!(gridcloud->empty())) {
					std::string grid_file_Name = file_Name + "Num_"+std::to_string(cur_grid_num)+"_x_"
						+ std::to_string(x_left) + "-" + std::to_string(x_right)
						+ "_y_" + std::to_string(y_left) + "-" + std::to_string(y_right)
						+ "_z_" + std::to_string(z_left) + "-" + std::to_string(z_right) + "_p_"+std::to_string(point_Count)+".ply";

					myWritePly(grid_file_Name, gridcloud);

					gridcloud->clear();
					std::cout << "��" << cur_grid_num << "��������������ɣ�" << "����" << point_Count << "����." << std::endl;
					cur_grid_num++;

				}
				else {
					std::cout << "��" << cur_grid_num << "���������Ϊ�ա�"<< std::endl;
					cur_grid_num++;

				}

			}
		}
	}
	return true;
	/*for (int i = 0; i < grid_Num; i++) {
		switch (i)
		{
		case 1:
			std::cout << "��һ������" << std::endl;
			if (cloud->points[i].x < grid_size && cloud->points[i].y < grid_size  && cloud->points[i].z < grid_size) {
				point_Count++;
				index.push_back(i);
				std::cout << "�ҵ�" << point_Count << "����" << std::endl;
			}
		default:
			break;
		}
	}
	for (int i = 0; i < cloud->size(); i++) {
		std::cout << "��Χ��0-256" << std::endl;

		if (cloud->points[i].x < 128 && cloud->points[i].y < 256 && cloud->points[i].z <128 ) {
			point_Count++;
			index.push_back(i);
			std::cout << "�ҵ�" << point_Count << "����" << std::endl;
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
	myWritePly(file_Name, gridcloud);*/

}

//���Ƶ�ƴ�Ӻ���
pcl::PointCloud<pcl::PointXYZRGB>::Ptr linkPoint(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud1, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2) {
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr linked_Cloud(new  pcl::PointCloud<pcl::PointXYZRGB>);
	*linked_Cloud = *cloud1 + *cloud2;
	return linked_Cloud;
}