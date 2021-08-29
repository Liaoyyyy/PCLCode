#include "pcl_fuc.h"


int main() {

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	bool is_load = myLoadPly("longdress_vox10_1051.ply", cloud);
	//bool is_filted = RadiusOutlierRemoval("S", cloud);
	bool is_Segment = GridSegment("∑÷∏Ó≤‚ ‘.ply", cloud,128);
	return 0;

}
