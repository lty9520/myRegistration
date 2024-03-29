#include <iostream>
#include <string>

#include <vtkAutoInit.h>        

VTK_MODULE_INIT(vtkRenderingOpenGL);  //解决方法参考【http://tieba.baidu.com/p/4551950404#93116920200l】

									  //VTK_MODULE_INIT(vtkRenderingOpenGL2);  //自己的PCL1.8.1安装后产生的是OpenGL,所以这一句需要改成OpenGL.

VTK_MODULE_INIT(vtkInteractionStyle); //外部依赖项添加opengl32.lib（我自己的还要添加vfw32.lib你的不知道要不要）

VTK_MODULE_INIT(vtkRenderingFreeType);

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/vtk_lib_io.h>//loadPolygonFileOBJ所属头文件；
#include <pcl/common/time.h>

#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing.h>

using namespace std;
using namespace pcl;

int main()
{

	pcl::StopWatch time;
	//参考点云
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	//对齐点云
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_align(new pcl::PointCloud<pcl::PointXYZ>);


	////创建mesh对象
	//pcl::PolygonMesh mesh;
	////读取polygon文件，obj格式读取为mesh
	//pcl::io::loadPolygonFile("chaijie-65-rotatest.obj", mesh);
	//
	//
	//
	////将mesh格式转换为PointCloud格式 方便读取
	//pcl::fromPCLPointCloud2(mesh.cloud, *cloud_align);
	////转存为可读取的PCD文件格式
	//pcl::io::savePCDFileASCII("chaijie-65-rotatest.pcd", *cloud_align);

	pcl::io::loadPCDFile("chaijie.pcd", *cloud);
	
	pcl::search::Search<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	pcl::PointCloud <pcl::Normal>::Ptr normals(new pcl::PointCloud <pcl::Normal>);
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
	normal_estimator.setInputCloud(cloud);
	normal_estimator.setSearchMethod(tree);
	normal_estimator.setKSearch(50);
	normal_estimator.compute(*normals);

	pcl::IndicesPtr indices(new std::vector <int>);
	pcl::PassThrough<pcl::PointXYZ> pass;
	pass.setInputCloud(cloud);
	pass.setFilterFieldName("z");
	pass.setFilterLimits(0.0, 1.0);
	pass.filter(*indices);

	pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
	reg.setMinClusterSize(100);
	reg.setMaxClusterSize(1000000);
	reg.setSearchMethod(tree);
	reg.setNumberOfNeighbours(10);
	reg.setInputCloud(cloud);
	//reg.setIndices (indices);
	reg.setInputNormals(normals);
	reg.setSmoothnessThreshold(3.0 / 180.0 * M_PI);
	reg.setCurvatureThreshold(0.5);

	std::vector <pcl::PointIndices> clusters;
	reg.extract(clusters);

	std::cout << "Number of clusters is equal to " << clusters.size() << std::endl;
	std::cout << "First cluster has " << clusters[0].indices.size() << " points." << endl;
	std::cout << "These are the indices of the points of the initial" <<
		std::endl << "cloud that belong to the first cluster:" << std::endl;
	int counter = 0;
	while (counter < clusters[0].indices.size())
	{
		std::cout << clusters[0].indices[counter] << ", ";
		counter++;
		if (counter % 10 == 0)
			std::cout << std::endl;
	}
	std::cout << std::endl;

	pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud();

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("myRegistration"));  //定义窗口共享指针
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_normal(new pcl::visualization::PCLVisualizer("normal"));

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red(cloud, 255, 0, 0);  //输入的初始点云相关
	viewer_normal->addPointCloud<pcl::PointXYZ>(cloud, red, "cloud");
	viewer_normal->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud, normals, 3, 1, "cloud normal");
	viewer_normal->setBackgroundColor(0, 0, 0);

	viewer->addPointCloud(colored_cloud, "colored_cloud");
	viewer->setBackgroundColor(1, 1, 1);

	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
	}



	






	return 0;
}