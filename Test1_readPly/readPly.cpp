#include <iostream> //��׼���������
#include <pcl/io/pcd_io.h>//PCL��PCD��ʽ�ļ����������ͷ�ļ�
#include <pcl/io/ply_io.h>//PCL��PLY��ʽ�ļ����������ͷ�ļ�
#include <pcl/point_types.h>//PCL��֧�ֵĵ������ļ�
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>//���ӻ�ͷ�ļ�

//��������ƵĹ��̽���һ����װ��
bool myLoadPly(const std::string fileName, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) {
	if ((pcl::io::loadPLYFile<pcl::PointXYZRGB>(fileName, *cloud) < 0)) {
		std::cout << "�ļ���ȡʧ�ܣ�" << std::endl;
		return false;
	}
	else {
		std::cout << "�ļ���ȡ�ɹ���" << std::endl;
		return false;

	};
}
//�Ե��ƽ��п��ӻ�
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
int main() {

	//��������ָ�벢ʵ����ָ��
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new  pcl::PointCloud<pcl::PointXYZRGB>);
	std::string fileName = "longdress_vox10_1051.ply";
	//��ȡply�ļ�
	bool isload = myLoadPly(fileName, cloud);
	//���ӻ�����
	//bool isView = myViewer("longDress_1051", cloud);
	system("pause");
	return 0;
}
