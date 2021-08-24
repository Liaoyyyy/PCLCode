#include "pcl_fuc.h"
int main() {
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	bool is_load = myLoadPly("S26C03R03_rec_r1_1051.ply", cloud);
	bool is_filted = RadiusOutlierRemoval("S26C03R03_rec_r1_1051", cloud);
	return 0;
}
