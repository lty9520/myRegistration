#include <pcl/features/moment_of_inertia_estimation.h>
#include <vector>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <boost/thread/thread.hpp>

#include <vtkAutoInit.h>        

VTK_MODULE_INIT(vtkRenderingOpenGL);  //解决方法参考【http://tieba.baidu.com/p/4551950404#93116920200l】

									  //VTK_MODULE_INIT(vtkRenderingOpenGL2);  //自己的PCL1.8.1安装后产生的是OpenGL,所以这一句需要改成OpenGL.

VTK_MODULE_INIT(vtkInteractionStyle); //外部依赖项添加opengl32.lib（我自己的还要添加vfw32.lib你的不知道要不要）

VTK_MODULE_INIT(vtkRenderingFreeType);


#include <Eigen/Core>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/vtk_lib_io.h>//loadPolygonFileOBJ所属头文件；
#include <Eigen/src/StlSupport/details.h>
#include <pcl/registration/icp.h>

using namespace std;
using namespace pcl;


struct myMIE
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
	std::vector <float> moment_of_inertia;
	std::vector <float> eccentricity;
	pcl::PointXYZ min_point_AABB;
	pcl::PointXYZ max_point_AABB;
	pcl::PointXYZ min_point_OBB;
	pcl::PointXYZ max_point_OBB;
	pcl::PointXYZ position_OBB;
	Eigen::Matrix3f rotational_matrix_OBB;
	float major_value, middle_value, minor_value;
	Eigen::Vector3f major_vector, middle_vector, minor_vector;
	Eigen::Vector3f mass_center;

};


void grivtyAct(pcl::PointCloud<pcl::PointXYZ>::Ptr & in_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr & out_cloud, Eigen::Vector3f center)
{
	for (int i = 0; i < in_cloud->size(); i++)
	{
		out_cloud->points[i].x = in_cloud->points[i].x - center(0);
		out_cloud->points[i].y = in_cloud->points[i].y - center(1);
		out_cloud->points[i].z = in_cloud->points[i].z - center(2);
	}

}

int main()
{

	//读取
	pcl::PointCloud<pcl::PointXYZ>::Ptr refer_cloud(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr align_cloud(new pcl::PointCloud<pcl::PointXYZ>());

	myMIE myrefer;
	myMIE myalign;

	pcl::io::loadPCDFile("chaijie-65-rotatest.pcd", *align_cloud);
	pcl::io::loadPCDFile("abaqus.pcd", *refer_cloud);

	myrefer.cloud = refer_cloud;
	myalign.cloud = align_cloud;

	//计算各特征值
	pcl::MomentOfInertiaEstimation<pcl::PointXYZ> refer_mie;
	refer_mie.setInputCloud(myrefer.cloud);
	refer_mie.compute();
	pcl::MomentOfInertiaEstimation<pcl::PointXYZ> align_mie;
	align_mie.setInputCloud(myalign.cloud);
	align_mie.compute();

	//计算重心点坐标
	//Eigen::Vector3f mass_center_refer;
	//Eigen::Vector3f mass_center_align;
	refer_mie.getMassCenter(myrefer.mass_center);
	align_mie.getMassCenter(myalign.mass_center);

	//重心化
	grivtyAct(myrefer.cloud, myrefer.cloud, myrefer.mass_center);
	grivtyAct(myalign.cloud, myalign.cloud, myalign.mass_center);

	
	//pcl::io::savePCDFile("abaqus-zhongxin.pcd", *myrefer.cloud);
	pcl::io::savePCDFile("chaijie-65-rotatest-zhongxin.pcd", *myalign.cloud);

	//重新计算各特征值
	
	refer_mie.setInputCloud(myrefer.cloud);
	refer_mie.compute();
	align_mie.setInputCloud(myalign.cloud);
	align_mie.compute();

	align_mie.getMomentOfInertia(myalign.moment_of_inertia);
	align_mie.getEccentricity(myalign.eccentricity);
	align_mie.getAABB(myalign.min_point_AABB, myalign.max_point_AABB);
	align_mie.getOBB(myalign.min_point_OBB, myalign.max_point_OBB, myalign.position_OBB, myalign.rotational_matrix_OBB);
	align_mie.getEigenValues(myalign.major_value, myalign.middle_value, myalign.minor_value);
	align_mie.getEigenVectors(myalign.major_vector, myalign.middle_vector, myalign.minor_vector);
	align_mie.getMassCenter(myalign.mass_center);

	refer_mie.getMomentOfInertia(myrefer.moment_of_inertia);
	refer_mie.getEccentricity(myrefer.eccentricity);
	refer_mie.getAABB(myrefer.min_point_AABB, myrefer.max_point_AABB);
	refer_mie.getOBB(myrefer.min_point_OBB, myrefer.max_point_OBB, myrefer.position_OBB, myrefer.rotational_matrix_OBB);
	refer_mie.getEigenValues(myrefer.major_value, myrefer.middle_value, myrefer.minor_value);
	refer_mie.getEigenVectors(myrefer.major_vector, myrefer.middle_vector, myrefer.minor_vector);
	refer_mie.getMassCenter(myrefer.mass_center);




	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("grvity viewer"));

	viewer->addCoordinateSystem(1);
	viewer->setBackgroundColor(1.0, 1.0, 1.0);
	/*****refer****/
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> green(myrefer.cloud, 0, 255, 0); //转换到原点的点云相关
	viewer->addPointCloud<pcl::PointXYZ>(myrefer.cloud, green, "refer cloud");
	//addCube方法
	//viewer->addCube(min_point_AABB.x, max_point_AABB.x, min_point_AABB.y, max_point_AABB.y, min_point_AABB.z, max_point_AABB.z, 1.0, 1.0, 0.0, "AABB");//AABB盒子

	Eigen::Vector3f position1(myrefer.position_OBB.x - myrefer.mass_center(0), myrefer.position_OBB.y - myrefer.mass_center(1),
		myrefer.position_OBB.z - myrefer.mass_center(2));
	Eigen::Quaternionf quat1(myrefer.rotational_matrix_OBB);
	viewer->addCube(position1, quat1, myrefer.max_point_OBB.x - myrefer.min_point_OBB.x,
		myrefer.max_point_OBB.y - myrefer.min_point_OBB.y, myrefer.max_point_OBB.z - myrefer.min_point_OBB.z, "OBB_refer");//OBB盒子

	viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "OBB_refer");
	viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, "OBB_refer");
	//中心的坐标
	pcl::PointXYZ center1(myrefer.mass_center(0), myrefer.mass_center(1), myrefer.mass_center(2));
	pcl::PointXYZ x_axis1(myrefer.major_vector(0) + myrefer.mass_center(0), myrefer.major_vector(1) + myrefer.mass_center(1),
		myrefer.major_vector(2) + myrefer.mass_center(2));
	pcl::PointXYZ y_axis1(myrefer.middle_vector(0) + myrefer.mass_center(0), myrefer.middle_vector(1) + myrefer.mass_center(1),
		myrefer.middle_vector(2) + myrefer.mass_center(2));
	pcl::PointXYZ z_axis1(myrefer.minor_vector(0) + myrefer.mass_center(0), myrefer.minor_vector(1) + myrefer.mass_center(1),
		myrefer.minor_vector(2) + myrefer.mass_center(2));
	viewer->addLine(center1, x_axis1, 1.0f, 0.0f, 0.0f, "major eigen vector refer");
	viewer->addLine(center1, y_axis1, 0.0f, 1.0f, 0.0f, "middle eigen vector refer");
	viewer->addLine(center1, z_axis1, 0.0f, 0.0f, 1.0f, "minor eigen vector refer");
		
	/****align****/
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red(myalign.cloud, 255, 0, 0); //转换到原点的点云相关
	viewer->addPointCloud<pcl::PointXYZ>(myalign.cloud, red, "align cloud");
	//addCube方法
	//viewer->addCube(min_point_AABB.x, max_point_AABB.x, min_point_AABB.y, max_point_AABB.y, min_point_AABB.z, max_point_AABB.z, 1.0, 1.0, 0.0, "AABB");//AABB盒子

	Eigen::Vector3f position(myalign.position_OBB.x - myalign.mass_center(0), myalign.position_OBB.y - myalign.mass_center(1),
		myalign.position_OBB.z - myalign.mass_center(2));
	Eigen::Quaternionf quat(myalign.rotational_matrix_OBB);
	viewer->addCube(position, quat, myalign.max_point_OBB.x - myalign.min_point_OBB.x,
		myalign.max_point_OBB.y - myalign.min_point_OBB.y, myalign.max_point_OBB.z - myalign.min_point_OBB.z, "OBB_align");//OBB盒子

	viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "OBB_align");
	viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "OBB_align");
	//中心的坐标
	pcl::PointXYZ center(myalign.mass_center(0), myalign.mass_center(1), myalign.mass_center(2));
	pcl::PointXYZ x_axis(myalign.major_vector(0) + myalign.mass_center(0), myalign.major_vector(1) + myalign.mass_center(1),
		myalign.major_vector(2) + myalign.mass_center(2));
	pcl::PointXYZ y_axis(myalign.middle_vector(0) + myalign.mass_center(0), myalign.middle_vector(1) + myalign.mass_center(1),
		myalign.middle_vector(2) + myalign.mass_center(2));
	pcl::PointXYZ z_axis(myalign.minor_vector(0) + myalign.mass_center(0), myalign.minor_vector(1) + myalign.mass_center(1),
		myalign.minor_vector(2) + myalign.mass_center(2));
	viewer->addLine(center, x_axis, 1.0f, 0.0f, 0.0f, "major eigen vector align");
	viewer->addLine(center, y_axis, 0.0f, 1.0f, 0.0f, "middle eigen vector align");
	viewer->addLine(center, z_axis, 0.0f, 0.0f, 1.0f, "minor eigen vector align");

	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}


	return 0;
}


