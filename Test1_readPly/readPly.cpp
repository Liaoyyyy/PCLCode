#include <iostream> //��׼���������
#include <pcl/io/pcd_io.h>//PCL��PCD��ʽ�ļ����������ͷ�ļ�
#include <pcl/io/ply_io.h>//PCL��PLY��ʽ�ļ����������ͷ�ļ�
#include <pcl/point_types.h>//PCL��֧�ֵĵ������ļ�
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>//���ӻ�ͷ�ļ�
#include <pcl/features/normal_3d.h>//���߼���ͷ�ļ�
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/console/parse.h>
#include <pcl/io/obj_io.h>
#include <string>
#include <sstream>

//�������
bool myLoadPly(const std::string fileName, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) {
	if ((pcl::io::loadPLYFile<pcl::PointXYZRGB>(fileName, *cloud) < 0)) {
		//std::cout << "�ļ���ȡʧ�ܣ�" << std::endl;
		return false;
	}
	else {
		//std::cout << "�ļ���ȡ�ɹ���" << std::endl;
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
bool myNormal(const std::string fileName,pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) {

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
int main() {

	//��������ָ�벢ʵ����ָ��
	std::cout << "===========���ɵ��Ʒ����ļ�======================" << std::endl;
	for (int i = 1051; i <= 1100; i++) {
		std::cout << "........��" << i << "֡........"<< std::endl;
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new  pcl::PointCloud<pcl::PointXYZRGB>);
		std::string frameNum = std::to_string(i);
		std::string fileNameOri = "longdress_vox10_" + frameNum + ".ply";
		std::string fileNameNor = "longdres_vox10_normal_" + frameNum + ".ply";
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
	
	system("pause");
	return 0;
}
