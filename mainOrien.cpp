#include <iostream>
#include <string>

#include <vtkAutoInit.h>        
VTK_MODULE_INIT(vtkRenderingOpenGL);  //解决方法参考【http://tieba.baidu.com/p/4551950404#93116920200l】
									  //VTK_MODULE_INIT(vtkRenderingOpenGL2);  //自己的PCL1.8.1安装后产生的是OpenGL,所以这一句需要改成OpenGL.
VTK_MODULE_INIT(vtkInteractionStyle); //外部依赖项添加opengl32.lib（我自己的还要添加vfw32.lib你的不知道要不要）
VTK_MODULE_INIT(vtkRenderingFreeType);

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>		  
#include <vector>
#include <Eigen/Core>
#include <pcl/common/transforms.h>

using namespace std;
using namespace pcl;

int main()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normal(new pcl::PointCloud<pcl::Normal>());

	
	pcl::io::loadPCDFile("chaijie17-zhongxin.pcd", *cloud);

	Eigen::Vector4f pcaCentroid;
	pcl::compute3DCentroid(*cloud, pcaCentroid);
	Eigen::Matrix3f covariance;
	pcl::computeCovarianceMatrixNormalized(*cloud, pcaCentroid, covariance);
	Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
	Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
	Eigen::Vector3f eigenValuesPCA = eigen_solver.eigenvalues();

	//初始位置时的主方向
	pcl::PointXYZ c;
	c.x = pcaCentroid(0);
	c.y = pcaCentroid(1);
	c.z = pcaCentroid(2);
	pcl::PointXYZ pcZ;
	pcZ.x = 1 * eigenVectorsPCA(0, 0) + c.x;
	pcZ.y = 1 * eigenVectorsPCA(1, 0) + c.y;
	pcZ.z = 1 * eigenVectorsPCA(2, 0) + c.z;
	pcl::PointXYZ pcY;
	pcY.x = 1 * eigenVectorsPCA(0, 1) + c.x;
	pcY.y = 1 * eigenVectorsPCA(1, 1) + c.y;
	pcY.z = 1 * eigenVectorsPCA(2, 1) + c.z;
	pcl::PointXYZ pcX;
	pcX.x = 1 * eigenVectorsPCA(0, 2) + c.x;
	pcX.y = 1 * eigenVectorsPCA(1, 2) + c.y;
	pcX.z = 1 * eigenVectorsPCA(2, 2) + c.z;
	
	pcl::visualization::PCLVisualizer viewer;
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_handler(cloud, 255, 0, 0);
	viewer.addPointCloud(cloud, color_handler, "cloud");

	viewer.addArrow(pcZ, c, 0.0, 0.0, 1.0, false, "arrow_z");
	viewer.addArrow(pcY, c, 0.0, 1.0, 0.0, false, "arrow_y");
	viewer.addArrow(pcX, c, 1.0, 0.0, 0.0, false, "arrow_x");

	viewer.addCoordinateSystem(1);
	viewer.setBackgroundColor(1.0, 1.0, 1.0);
	while (!viewer.wasStopped())
	{
		viewer.spinOnce(100);

	}


	system("pause");
	return 0;
}