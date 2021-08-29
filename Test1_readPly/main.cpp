#include "pcl_fuc.h"


int main() {

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud3(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud4(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud5(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud6(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud7(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud8(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud9(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr linked_Cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr linked_Cloud2(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr linked_Cloud3(new pcl::PointCloud<pcl::PointXYZRGB>);

	myLoadPly("loot_Num_1_x_0-380_y_0-250_z_0-237_p_115869.ply", cloud1);
	myLoadPly("loot_Num_2_x_0-380_y_0-250_z_237-474_p_47758.ply", cloud2);
	myLoadPly("loot_Num_3_x_0-380_y_250-500_z_0-237_p_115726.ply", cloud3);
	myLoadPly("loot_Num_4_x_0-380_y_250-500_z_237-474_p_69167.ply", cloud4);
	myLoadPly("loot_Num_5_x_0-380_y_500-750_z_0-237_p_123641.ply", cloud5);
	myLoadPly("loot_Num_6_x_0-380_y_500-750_z_237-474_p_154447.ply", cloud6);
	myLoadPly("loot_Num_7_x_0-380_y_750-1000_z_0-237_p_68825.ply", cloud7);
	myLoadPly("loot_Num_8_x_0-380_y_750-1000_z_237-474_p_88709.ply", cloud8);
	*linked_Cloud = *cloud1 + *cloud2;
	*linked_Cloud = *linked_Cloud + *cloud3;
	*linked_Cloud = *linked_Cloud + *cloud4;
	*linked_Cloud2 = *cloud5 + *cloud7;
	*linked_Cloud3 = *cloud6 + *cloud8;
	myWritePly("loot_seg_1000-1_r1.ply", linked_Cloud);
	myWritePly("loot_seg_1000-2_r3.ply", linked_Cloud2);
	myWritePly("loot_seg_1000-3_r5.ply", linked_Cloud3);

	//bool is_filted = RadiusOutlierRemoval("S", cloud);
	//bool is_Segment = GridSegment("loot_", cloud,200);
	return 0;

}
