#include <pcl/features/moment_of_inertia_estimation.h>
#include <vector>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <boost/thread/thread.hpp>

#include <vtkAutoInit.h>        

VTK_MODULE_INIT(vtkRenderingOpenGL);  //��������ο���http://tieba.baidu.com/p/4551950404#93116920200l��

									  //VTK_MODULE_INIT(vtkRenderingOpenGL2);  //�Լ���PCL1.8.1��װ���������OpenGL,������һ����Ҫ�ĳ�OpenGL.

VTK_MODULE_INIT(vtkInteractionStyle); //�ⲿ���������opengl32.lib�����Լ��Ļ�Ҫ���vfw32.lib��Ĳ�֪��Ҫ��Ҫ��

VTK_MODULE_INIT(vtkRenderingFreeType);


#include <Eigen/Core>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/vtk_lib_io.h>//loadPolygonFileOBJ����ͷ�ļ���
#include <Eigen/src/StlSupport/details.h>
#include <pcl/registration/icp.h>

using namespace std;
using namespace pcl;


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

	//��ȡ
	pcl::PointCloud<pcl::PointXYZ>::Ptr refer_cloud(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr align_cloud(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::io::loadPCDFile("huagong-final-1.5.pcd", *align_cloud);
	pcl::io::loadPCDFile("proj-final-fbx-dian.pcd", *refer_cloud);

	//���������ֵ
	pcl::MomentOfInertiaEstimation<pcl::PointXYZ> refer_mie;
	refer_mie.setInputCloud(refer_cloud);
	refer_mie.compute();
	pcl::MomentOfInertiaEstimation<pcl::PointXYZ> align_mie;
	align_mie.setInputCloud(align_cloud);
	align_mie.compute();

	//�������ĵ�����
	Eigen::Vector3f mass_center_refer;
	Eigen::Vector3f mass_center_align;
	refer_mie.getMassCenter(mass_center_refer);
	align_mie.getMassCenter(mass_center_align);

	//���Ļ�
	grivtyAct(refer_cloud, refer_cloud, mass_center_refer);
	grivtyAct(align_cloud, align_cloud, mass_center_align);

	//���¼��������ֵ
	refer_mie.setInputCloud(refer_cloud);
	refer_mie.compute();
	align_mie.setInputCloud(align_cloud);
	align_mie.compute();





	return 0;
}


