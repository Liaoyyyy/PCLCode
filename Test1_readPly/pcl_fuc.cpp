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
#include <pcl/octree/octree_search.h>//八叉树库
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
//#include <string>
//#include <vector>
//#include <pcl/io/pcd_io.h>
//#include <pcl/point_types.h>
//#include<pcl/visualization/cloud_viewer.h>

typedef struct tagPOINT_3D
{
	double x;  //mm world coordinate x
	double y;  //mm world coordinate y
	double z;  //mm world coordinate z
	double r;
}POINT_WORLD;


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

//统计学滤波
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
	double x_max = 0;
	double y_max = 0;
	double z_max = 0;
	//std::vector<int> index;
	//index.reserve(1000000);
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
	int x_cur_size = ceil(x_max / x_size);
	int y_cur_size = ceil(y_max / y_size);
	int z_cur_size = ceil(z_max / z_size);
	//接下来对每一个网格的点云做分别的输出
	int cur_grid_num = 1;
	int seg_num = 1;
	//进行分割
	for (int i = 0; i < x_size; i++) {
		for (int j = 0; j < y_size; j++) {
			for (int k = 0; k < z_size; k++) {
				std::cout << "共有" << grid_Num << "个网格点云，当前输出第" << cur_grid_num << "个网格." << std::endl;
				//创建点云对象，用于存放计算得到的网格点云
				int x_right = (i+1)* x_cur_size;
				int x_left = i*x_cur_size;
				int y_right = (j+1) * y_cur_size;
				int y_left = j*y_cur_size;
				int z_right = (k+1) * z_cur_size;
				int z_left = k*z_cur_size;
				pcl::PointCloud<pcl::PointXYZRGB>::Ptr gridcloud(new  pcl::PointCloud<pcl::PointXYZRGB>);
				gridcloud->reserve(500000);
				//对所有点进行循环
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
						std::cout << "第" << cur_grid_num << "网格读取到第" << point_Count << "个点" << std::endl;

						//gridcloud->push_back(cloud->points[i]);
						point_Count++;

					}
				}
				//输出计算得到的点云。
				if (!(gridcloud->empty())) {
					/*std::string grid_file_Name = file_Name + "Num_"+std::to_string(cur_grid_num)+"_x_"
						+ std::to_string(x_left) + "-" + std::to_string(x_right)
						+ "_y_" + std::to_string(y_left) + "-" + std::to_string(y_right)
						+ "_z_" + std::to_string(z_left) + "-" + std::to_string(z_right) + "_p_"+std::to_string(point_Count)+".ply";*/
					
					std::string grid_file_Name = file_Name + "Num_" + std::to_string(cur_grid_num) + ".ply";
					seg_num++;

					myWritePly(grid_file_Name, gridcloud);

					gridcloud->clear();
					std::cout << "第" << cur_grid_num << "个网格点云输出完成，" << "包含" << point_Count << "个点." << std::endl;
					cur_grid_num++;

				}
				else {
					std::cout << "第" << cur_grid_num << "个网格点云为空。"<< std::endl;
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
			std::cout << "第一个网格：" << std::endl;
			if (cloud->points[i].x < grid_size && cloud->points[i].y < grid_size  && cloud->points[i].z < grid_size) {
				point_Count++;
				index.push_back(i);
				std::cout << "找到" << point_Count << "个点" << std::endl;
			}
		default:
			break;
		}
	}
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
	myWritePly(file_Name, gridcloud);*/

}

//点云的拼接函数
pcl::PointCloud<pcl::PointXYZRGB>::Ptr linkPoint(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud1,
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2) {
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr linked_Cloud(new  pcl::PointCloud<pcl::PointXYZRGB>);
	*linked_Cloud = *cloud1 + *cloud2;
	return linked_Cloud;
}

//点云表面的平滑处理
pcl::PointCloud<pcl::PointXYZRGB>::Ptr SurfaceSmooth(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,const int r,const int p) {
	std::cout << "对点云开始平滑处理！"<<std::endl;

	//// 对点云重采样  
	//pcl::search::KdTree<pcl::PointXYZRGB>::Ptr treeSampling(new pcl::search::KdTree<pcl::PointXYZRGB>); // 创建用于最近邻搜索的KD-Tree
	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr mls_points(new pcl::search::KdTree<pcl::PointXYZRGB>);   //输出MLS
	//pcl::MovingLeastSquares<pcl::PointXYZRGB, pcl::PointXYZRGB> mls;  // 定义最小二乘实现的对象mls
	//mls.setComputeNormals(false);  //设置在最小二乘计算中是否需要存储计算的法线
	//mls.setInputCloud(cloud);        //设置待处理点云
	//mls.setPolynomialOrder(2);             // 拟合2阶多项式拟合，一般取2-5
	//mls.setPolynomialFit(false);  // 设置为false可以 加速 smooth
	//mls.setSearchMethod(treeSampling);    // 设置KD-Tree作为搜索方法
	//mls.setSearchRadius(0.05); // 单位m.设置用于拟合的K近邻半径，越大平滑力度越大
	//mls.process(*mls_points);        //输出
	//myWritePly("平滑后点云数据.ply", mls_points);
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_tgt(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGBNormal> mls_points;
	pcl::MovingLeastSquares<pcl::PointXYZRGB, pcl::PointXYZRGBNormal> mls;
	mls.setComputeNormals(true);
	mls.setInputCloud(cloud);
	mls.setPolynomialFit(true);
	mls.setSearchMethod(tree);
	mls.setSearchRadius(r);
	mls.setPolynomialOrder(p);
	mls.process(mls_points);
	std::cout << "将mls后的结果xyzrgbNormal转换成为xyzrgb" << std::endl;

	// 将mls后的结果xyzrgbNormal转换成为xyzrgb
	cloud_tgt->points.resize(mls_points.size());
	for (size_t i = 0; i < mls_points.size(); ++i)
	{
		cloud_tgt->points[i].x = mls_points.points[i].x;
		cloud_tgt->points[i].y = mls_points.points[i].y;
		cloud_tgt->points[i].z = mls_points.points[i].z;
		cloud_tgt->points[i].r = mls_points.points[i].r;
		cloud_tgt->points[i].g = mls_points.points[i].g;
		cloud_tgt->points[i].b = mls_points.points[i].b;
	}
	cloud_tgt->width = mls_points.size();
	cloud_tgt->height = 1;
	std::cout << "处理结束！" << std::endl;

	//pcl::io::savePCDFile("qq0p.pcd", *cloud_tgt);
	return cloud_tgt;
}


//点云坐标取整
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_Int(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) {
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_Int(new pcl::PointCloud<pcl::PointXYZRGB>);
	
	cloud_Int->reserve(cloud->size());
	int point_Num = cloud->size();
	
	for (int i = 0; i < point_Num; i++) {
		pcl::PointXYZRGB point_Int;
		point_Int.x =(int)((cloud->points[i].x)+0.5);
		point_Int.y = (int)((cloud->points[i].y) + 0.5);
		point_Int.z = (int)((cloud->points[i].z) + 0.5);
		point_Int.rgb = cloud->points[i].rgb;
		cloud_Int->push_back(point_Int);

	}
	return cloud_Int;
}

//对输入的点云提取其坐标信息输出为没有颜色的新点云
pcl::PointCloud<pcl::PointXYZ>::Ptr DeleteRGB(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) {
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_Int(new pcl::PointCloud<pcl::PointXYZ>);
	cloud_Int->reserve(cloud->size());
	int point_Num = cloud->size();
	for (int i = 0; i < point_Num; i++) {
		pcl::PointXYZ point_Int;
		point_Int.x = cloud->points[i].x;
		point_Int.y = cloud->points[i].y;
		point_Int.z = cloud->points[i].z;
		cloud_Int->push_back(point_Int);
	}
	return cloud_Int;
}

//输出xyz数据的点云
bool myWriteXYZPly(const std::string fileName, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
	if (pcl::io::savePLYFile<pcl::PointXYZ>(fileName, *cloud) < 0)
	{
		//std::cout << "输出失败！" << endl;
		return false;
	}
	else {
		//std::cout << "输出成功！" << endl;
		return true;
	}
}

//对输入的xyz格式点云做ply格式输出
pcl::PointCloud < pcl::PointXYZ>::Ptr xyz_to_ply(char* fname) {

	//////计算有几个点（多少行）
	////int Point_Num = 0;
	////int scaner = 0;
	////FILE *fp;
	////fp = fopen(fname, "r");
	////do {
	////	scaner = fgetc(fp);
	////	if (scaner == '\n')
	////	{
	////		++Point_Num;
	////	}
	////} while (scaner != EOF);
	//////fclose(fp);
	////cout << "there are " << Point_Num << " points in the file..." << endl;
	//////新建一个点云文件，然后将结构中获取的xyz值传递到点云指针cloud中。
	////pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	//////pcl::PointCloud<pcl::PointXYZ> cloud;
	////cloud->width = Point_Num;
	////cloud->height = 1;
	////cloud->is_dense = false;
	////cloud->points.resize(cloud->width * cloud->height);
	//////将点云读入并赋给新建点云指针的xyz      
	////double x, y, z;
	////std::string cx;
	////std::string cy;
	////std::string cz;
	////for (int i = 0; i < Point_Num; i++) {
	////	fscanf(fp, "%s %s %s\n", &cx, &cy, &cz);
	////	cloud->points[i].x = atof(cx.c_str());
	////	cloud->points[i].y = atof(cy.c_str());
	////	cloud->points[i].z = atof(cz.c_str());
	////}
	//////int i = 0;
	//////while (3 == fscanf(fp, "%lf %lf %lf\n", &x, &y, &z))
	//////{
	//////	cout << x << " " << y << " " << z << endl;//这句要不要都行，如果输出的话转换的速度会很慢，因为每个点都会读取出来
	//////	cloud->points[i].x = x;
	//////	cloud->points[i].y = y;
	//////	cloud->points[i].z = z;
	//////	++i;
	//////}
	//// 加载txt数据
	////typedef struct tagPOINT_3D
	////{
	////	double x;  //mm world coordinate x
	////	double y;  //mm world coordinate y
	////	double z;  //mm world coordinate z
	////	double r;
	////}POINT_WORLD;

	//int number_Txt;
	//FILE *fp_txt;
	////tagPOINT_3D TxtPoint;
	////std::vector<tagPOINT_3D> m_vTxtPoints;
	//pcl::PointXYZ TxtPoint;
	//fp_txt = fopen(fname, "r");//这个地方填文件的位置
	//if (fp_txt)
	//{
	//	while (fscanf(fp_txt, "%lf %lf %lf", &TxtPoint.x, &TxtPoint.y, &TxtPoint.z) != EOF)
	//	{
	//		.push_back(TxtPoint);
	//	}
	//}
	//else
	//	cout << "txt数据加载失败！" << endl;
	//number_Txt = m_vTxtPoints.size();
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;

	//// Fill in the cloud data
	//cloud->width = number_Txt;
	//cloud->height = 1;
	//cloud->is_dense = false;
	//cloud->points.resize(cloud->width * cloud->height);
	//for (size_t i = 0; i < cloud->points.size(); ++i)
	//{
	//	cloud->points[i].x = m_vTxtPoints[i].x;
	//	cloud->points[i].y = m_vTxtPoints[i].y;
	//	cloud->points[i].z = m_vTxtPoints[i].z;
	//}
	////fclose(fp);
	//return cloud;
	// 加载txt数据
	int number_Txt;
	FILE *fp_txt;
	tagPOINT_3D TxtPoint;
	std::vector<tagPOINT_3D> m_vTxtPoints;
	fp_txt = fopen(fname, "r");//这个地方填文件的位置
	if (fp_txt)
	{
		while (fscanf(fp_txt, "%lf %lf %lf", &TxtPoint.x, &TxtPoint.y, &TxtPoint.z) != EOF)
		{
			m_vTxtPoints.push_back(TxtPoint);
		}
	}
	else
		cout << "txt数据加载失败！" << endl;
	number_Txt = m_vTxtPoints.size();
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

	// Fill in the cloud data
	cloud->width = number_Txt;
	cloud->height = 1;
	cloud->is_dense = false;
	cloud->points.resize(cloud->width * cloud->height);
	for (size_t i = 0; i < cloud->points.size(); ++i)
	{
		cloud->points[i].x = m_vTxtPoints[i].x;
		cloud->points[i].y = m_vTxtPoints[i].y;
		cloud->points[i].z = m_vTxtPoints[i].z;
	}
	//pcl::io::savePCDFileASCII("reproject_pcd.pcd", cloud);//这个地方填输出的路径
	//std::cerr << "Saved " << cloud.points.size() << " data points to txt2pcd.pcd." << std::endl;
	//for (size_t i = 0; i < cloud.points.size(); ++i)
	//	std::cerr << "    " << cloud.points[i].x << " " << cloud.points[i].y << " " << cloud.points[i].z << std::endl;

	//system("pause");
	return cloud;

}

//对xyz格式的ply点云输出为xyzrgb格式的点云数据（颜色为空，目的是用于质量评价）

pcl::PointCloud<pcl::PointXYZRGB>::Ptr plyxyz_to_xyzrgb(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZRGB>);
	cloud1->resize(cloud->size());
	for (int i = 0; i < cloud->size(); i++) {
		cloud1->points[i].x = cloud->points[i].x;
		cloud1->points[i].y = cloud->points[i].y;
		cloud1->points[i].z = cloud->points[i].z;
	}
	return cloud1;
}

//点云融合算法
pcl::PointCloud<pcl::PointXYZRGB>::Ptr LinkPoint(std::vector<std::string> FileName) {
	std::vector<pcl::PointCloud<pcl::PointXYZRGB>> PointClouds;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr Cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	std::string name;
	PointClouds.resize(FileName.size());
	//把点云块放到同一个容器中
	for (int i = 0; i < FileName.size(); i++) {
		name = FileName[i];
		myLoadPly(name, Cloud);
		PointClouds.push_back(*Cloud);
	}
	//开始拼接
	for (int i = 0; i < PointClouds.size(); i++) {

		*Cloud = *Cloud + PointClouds[i];
	}
	return Cloud;
}

//以八叉树为索引对点云进行分割
bool octree_Seg(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) {
	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB> search_Point;
	//创建八叉树
	float resolution = 512.0f; //分辨率，最小体素尺寸
	//定义边界框
	//int minX, minY, minZ, maxX, maxY, maxZ;
	pcl::octree::OctreePointCloud<pcl::PointXYZRGB> octree(resolution);//实例化八叉树对象
	/*pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGB,
	pcl::octree::OctreeContainerPointIndices, pcl::octree::OctreeContainerPointIndices> m_octree(resolution);*/
	//用来放占用网格
	std::vector<pcl::PointXYZRGB, Eigen::aligned_allocator<pcl::PointXYZRGB>> pointGrid;
	//传入输入点数据
	octree.setInputCloud(cloud);
	//m_octree.setInputCloud(cloud);
	//计算输入点云的边界框,调用点云数据集的维度并定义边界框
	octree.defineBoundingBox();
	//手动定义点云边界框
	//octree.defineBoundingBox(minX, minY, minZ, maxX, maxY, maxZ);
	//输入点添加到octree
	octree.addPointsFromInputCloud();
	//获得占用的体素中心
	octree.getOccupiedVoxelCenters(pointGrid);
	//m_octree.addPointsFromInputCloud();
	//增长边界框知道点适合
	//启用动态深度,当叶节点点数超出限定时才会扩张叶节点。
	//octree.enableDynamicDepth(10000);
	//设置被搜索点,用随机数填充
	//pcl::PointXYZRGB searchPoint;
	//std::vector<int> pointIdxVec;
	//searchPoint.x = 1024.0f * rand() / (RAND_MAX + 1.0f);
	//searchPoint.y = 1024.0f * rand() / (RAND_MAX + 1.0f);
	//searchPoint.z = 1024.0f * rand() / (RAND_MAX + 1.0f);
	//searchPoint.rgb = 1024.0f * rand() / (RAND_MAX + 1.0f);
	//if (octree.voxelSearch(searchPoint, pointIdxVec)) {
	//获得八叉树的深度
	int depth = octree.getTreeDepth();
	//获取叶数
	int leafNum = octree.getLeafCount();
	//std::vector<int> idx;
	//auto it = m_octree.breadth_begin();
	//it.getBranchContainer().getPointIndices(idx);

	
	//octree.
	//}
	//获取所有占用体素中心的 PointT 向量。
	int sideLen = octree.getVoxelSquaredSideLen();
	std::cout <<"深度：" <<depth <<"  叶数："<<leafNum<<"  叶级别体素立方体边长:"<<sideLen<< endl;
	//for (int i = 0; i < idx.size(); i++) {
	//	std::cout << idx[i] << endl;
	//}
	return true;       

}