#include "pcl_fuc.h"
#include <stdlib.h>
#include <stdio.h>

int main() {

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_smoothed(new pcl::PointCloud<pcl::PointXYZRGB>);
	//l::PointCloud<pcl::PointXYZ>::Ptr cloud_I(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr Cloud1(new pcl::PointCloud<pcl::PointXYZ>);


	//LoadPly("longdress_1051_r2__f1_rec_1051.ply", cloud);
	/*myLoadPly("loot_seg_1000-2_r3_rec.ply", cloud2);
	myLoadPly("loot_seg_1000-3_r5_rec.ply", cloud3);
*/
	/**linked_Cloud = *cloud1 + *cloud2;
	*linked_Cloud = *linked_Cloud + *cloud3;
	myWritePly("loot_seg_1000_rec_Unit.ply", linked_Cloud);*/

	//bool is_filted = RadiusOutlierRemoval("S", cloud);

	//bool is_Segment = GridSegment("loot_vox10_1000_k=200_", cloud1,200);
	//调用质量测试软件并循环分割块得到最后结果 -a a.ply -b r.ply -c -r 1023 -d -n n.ply
	//system("PAQM.exe -a longdress_vox10_1051.ply -b smooth_longdress_1051_r1_16_1.ply -c -r 1023 -d -n longdres_vox10_normal_1051.ply");
	//对点云进行平滑处理
	//cloud_smoothed = SurfaceSmooth(cloud,6,2);
	//cloud_I = cloud_Int(cloud_smoothed);
	//myWritePly("smooth_longdress_ori_1051_r3_6_2_Int.ply", cloud_I);
	//对输入的ply格式的xyzrgb点云进行格式转换 输出格式为xyz的格式点云
	//oud_I = DeleteRGB(cloud);
	//WriteXYZPly("xzy_Unnoise_longdress_1051_r1.ply", cloud_I);
	//对读取的xyz格式点云数据做PCD输出
	char fileName[]= "xzy_denoise_longdress_1051_r1.xyz";
	Cloud1 = xyz_to_ply(fileName);
	cloud2 = plyxyz_to_xyzrgb(Cloud1);
	//myWriteXYZPly("降噪后输出.ply", cloud2);
	myWritePly("3.ply", cloud2);
	return 0;

}
