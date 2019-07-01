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
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr grvity_cloud(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::io::loadPCDFile("chaijie-17.pcd", *cloud);

	grvity_cloud = cloud;
	
	//实例化一个Momentof...
	pcl::MomentOfInertiaEstimation <pcl::PointXYZ> feature_extractor;
	feature_extractor.setInputCloud(cloud);
	feature_extractor.compute();
	//声明一些必要的变量
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
	//计算描述符和其他的特征
	feature_extractor.getMomentOfInertia(moment_of_inertia);
	feature_extractor.getEccentricity(eccentricity);
	feature_extractor.getAABB(min_point_AABB, max_point_AABB);
	feature_extractor.getOBB(min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);
	feature_extractor.getEigenValues(major_value, middle_value, minor_value);
	feature_extractor.getEigenVectors(major_vector, middle_vector, minor_vector);
	feature_extractor.getMassCenter(mass_center);
	
	cout << "gravity center : " << mass_center(0) << ", " << mass_center(1) << ", " << mass_center(2) << endl;
	grivtyAct(cloud, grvity_cloud, mass_center);

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_gravity(new pcl::visualization::PCLVisualizer("grvity viewer"));
	viewer_gravity->addCoordinateSystem(1.0);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red(cloud, 255, 0, 0);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> green(grvity_cloud, 0, 255, 0);
	viewer_gravity->addPointCloud(cloud, red, "origin cloud");
	viewer_gravity->addPointCloud(grvity_cloud, green, "gravity act point cloud");

	
	//显示
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);
	viewer->addCoordinateSystem(1.0);
	viewer->initCameraParameters();
	viewer->addPointCloud<pcl::PointXYZ>(cloud, "sample cloud");
	//addCube方法
	//viewer->addCube(min_point_AABB.x, max_point_AABB.x, min_point_AABB.y, max_point_AABB.y, min_point_AABB.z, max_point_AABB.z, 1.0, 1.0, 0.0, "AABB");//AABB盒子

	Eigen::Vector3f position(position_OBB.x - mass_center(0), position_OBB.y - mass_center(1), position_OBB.z - mass_center(2));
	Eigen::Quaternionf quat(rotational_matrix_OBB);
	viewer->addCube(position, quat, max_point_OBB.x - min_point_OBB.x, max_point_OBB.y - min_point_OBB.y, max_point_OBB.z - min_point_OBB.z, "OBB");//OBB盒子
																																					
	viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "OBB");
	viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, "OBB");
	//中心的坐标
	pcl::PointXYZ center(mass_center(0), mass_center(1), mass_center(2));
	pcl::PointXYZ x_axis(major_vector(0) + mass_center(0), major_vector(1) + mass_center(1), major_vector(2) + mass_center(2));
	pcl::PointXYZ y_axis(middle_vector(0) + mass_center(0), middle_vector(1) + mass_center(1), middle_vector(2) + mass_center(2));
	pcl::PointXYZ z_axis(minor_vector(0) + mass_center(0), minor_vector(1) + mass_center(1), minor_vector(2) + mass_center(2));
	viewer->addLine(center, x_axis, 1.0f, 0.0f, 0.0f, "major eigen vector");
	viewer->addLine(center, y_axis, 0.0f, 1.0f, 0.0f, "middle eigen vector");
	viewer->addLine(center, z_axis, 0.0f, 0.0f, 1.0f, "minor eigen vector");

	//addLine画线

	//Eigen::Vector3f p1 (min_point_OBB.x, min_point_OBB.y, min_point_OBB.z);
	//Eigen::Vector3f p2 (min_point_OBB.x, min_point_OBB.y, max_point_OBB.z);
	//Eigen::Vector3f p3 (max_point_OBB.x, min_point_OBB.y, max_point_OBB.z);
	//Eigen::Vector3f p4 (max_point_OBB.x, min_point_OBB.y, min_point_OBB.z);
	//Eigen::Vector3f p5 (min_point_OBB.x, max_point_OBB.y, min_point_OBB.z);
	//Eigen::Vector3f p6 (min_point_OBB.x, max_point_OBB.y, max_point_OBB.z);
	//Eigen::Vector3f p7 (max_point_OBB.x, max_point_OBB.y, max_point_OBB.z);
	//Eigen::Vector3f p8 (max_point_OBB.x, max_point_OBB.y, min_point_OBB.z);

	//p1 = rotational_matrix_OBB * p1 + position;
	//p2 = rotational_matrix_OBB * p2 + position;
	//p3 = rotational_matrix_OBB * p3 + position;
	//p4 = rotational_matrix_OBB * p4 + position;
	//p5 = rotational_matrix_OBB * p5 + position;
	//p6 = rotational_matrix_OBB * p6 + position;
	//p7 = rotational_matrix_OBB * p7 + position;
	//p8 = rotational_matrix_OBB * p8 + position;

	//pcl::PointXYZ pt1 (p1 (0), p1 (1), p1 (2));
	//pcl::PointXYZ pt2 (p2 (0), p2 (1), p2 (2));
	//pcl::PointXYZ pt3 (p3 (0), p3 (1), p3 (2));
	//pcl::PointXYZ pt4 (p4 (0), p4 (1), p4 (2));
	//pcl::PointXYZ pt5 (p5 (0), p5 (1), p5 (2));
	//pcl::PointXYZ pt6 (p6 (0), p6 (1), p6 (2));
	//pcl::PointXYZ pt7 (p7 (0), p7 (1), p7 (2));
	//pcl::PointXYZ pt8 (p8 (0), p8 (1), p8 (2));

	//viewer->addLine (pt1, pt2, 1.0, 0.0, 0.0, "1 edge");
	//viewer->addLine (pt1, pt4, 1.0, 0.0, 0.0, "2 edge");
	//viewer->addLine (pt1, pt5, 1.0, 0.0, 0.0, "3 edge");
	//viewer->addLine (pt5, pt6, 1.0, 0.0, 0.0, "4 edge");
	//viewer->addLine (pt5, pt8, 1.0, 0.0, 0.0, "5 edge");
	//viewer->addLine (pt2, pt6, 1.0, 0.0, 0.0, "6 edge");
	//viewer->addLine (pt6, pt7, 1.0, 0.0, 0.0, "7 edge");
	//viewer->addLine (pt7, pt8, 1.0, 0.0, 0.0, "8 edge");
	//viewer->addLine (pt2, pt3, 1.0, 0.0, 0.0, "9 edge");
	//viewer->addLine (pt4, pt8, 1.0, 0.0, 0.0, "10 edge");
	//viewer->addLine (pt3, pt4, 1.0, 0.0, 0.0, "11 edge");
	//viewer->addLine (pt3, pt7, 1.0, 0.0, 0.0, "12 edge");



	Eigen::Vector4f pcaCentroid;
	//pcl::PointCloud<pcl::PointXYZ>::Ptr ori_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	//pointcloud.cloud = ori_cloud;
	pcl::compute3DCentroid(*cloud, pcaCentroid);
	Eigen::Matrix3f covariance;
	pcl::computeCovarianceMatrixNormalized(*cloud, pcaCentroid, covariance);
	Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
	Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
	Eigen::Vector3f eigenValuesPCA = eigen_solver.eigenvalues();
	eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1)); //校正主方向间垂直
	eigenVectorsPCA.col(0) = eigenVectorsPCA.col(1).cross(eigenVectorsPCA.col(2));
	eigenVectorsPCA.col(1) = eigenVectorsPCA.col(2).cross(eigenVectorsPCA.col(0));

	std::cout << "特征值va(3x1):\n" << eigenValuesPCA << std::endl;
	std::cout << "特征向量ve(3x3):\n" << eigenVectorsPCA << std::endl;
	std::cout << "质心点(4x1):\n" << pcaCentroid << std::endl;
	/*
	// 另一种计算点云协方差矩阵特征值和特征向量的方式:通过pcl中的pca接口，如下，这种情况得到的特征向量相似特征向量
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPCAprojection (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PCA<pcl::PointXYZ> pca;
	pca.setInputCloud(cloudSegmented);
	pca.project(*cloudSegmented, *cloudPCAprojection);
	std::cerr << std::endl << "EigenVectors: " << pca.getEigenVectors() << std::endl;//计算特征向量
	std::cerr << std::endl << "EigenValues: " << pca.getEigenValues() << std::endl;//计算特征值
	*/
	Eigen::Matrix4f tm = Eigen::Matrix4f::Identity();
	Eigen::Matrix4f tm_inv = Eigen::Matrix4f::Identity();
	tm.block<3, 3>(0, 0) = eigenVectorsPCA.transpose();   //R.
	tm.block<3, 1>(0, 3) = -1.0f * (eigenVectorsPCA.transpose()) *(pcaCentroid.head<3>());//  -R*t
	tm_inv = tm.inverse();

	std::cout << "变换矩阵tm(4x4):\n" << tm << std::endl;
	std::cout << "逆变矩阵tm'(4x4):\n" << tm_inv << std::endl;

	pcl::PointCloud<pcl::PointXYZ>::Ptr tfCloud(new pcl::PointCloud<pcl::PointXYZ>);
	//pointcloud.transformedCloud = tfCloud;
	//pcl::transformPointCloud(*pointcloud.cloud, *pointcloud.transformedCloud, tm);

	pcl::PointXYZ min_p1, max_p1;
	Eigen::Vector3f c1, c;
	pcl::getMinMax3D(*cloud, min_p1, max_p1);
	c1 = 0.5f*(min_p1.getVector3fMap() + max_p1.getVector3fMap());

	std::cout << "型心c1(3x1):\n" << c1 << std::endl;

	Eigen::Affine3f tm_inv_aff(tm_inv);
	//pcl::transformPoint(c1, c, tm_inv_aff);

	Eigen::Vector3f whd;
	whd = max_p1.getVector3fMap() - min_p1.getVector3fMap();
	
	float sc = (whd(0) + whd(1) + whd(2)) / 3;  //点云平均尺度，用于设置主方向箭头大小

	std::cout << "width1=" << whd(0) << endl;
	std::cout << "heght1=" << whd(1) << endl;
	std::cout << "depth1=" << whd(2) << endl;
	std::cout << "scale1=" << sc << endl;

	Eigen::Quaternionf bboxQ = Eigen::Quaternionf::Identity();
	Eigen::Vector3f bboxT = c1;

	const Eigen::Quaternionf bboxQ1(tm_inv.block<3, 3>(0, 0));
	const Eigen::Vector3f    bboxT1(c);


	//变换到原点的点云主方向
	pcl::PointXYZ op;
	op.x = 0.0;
	op.y = 0.0;
	op.z = 0.0;
	Eigen::Vector3f px, py, pz;
	Eigen::Affine3f tm_aff(tm);
	//pcl::transformVector(eigenVectorsPCA.col(0), px, tm_aff);
	//pcl::transformVector(eigenVectorsPCA.col(1), py, tm_aff);
	//pcl::transformVector(eigenVectorsPCA.col(2), pz, tm_aff);
	pcl::PointXYZ pcaX;
	pcaX.x = sc * px(0);
	pcaX.y = sc * px(1);
	pcaX.z = sc * px(2);
	pcl::PointXYZ pcaY;
	pcaY.x = sc * py(0);
	pcaY.y = sc * py(1);
	pcaY.z = sc * py(2);
	pcl::PointXYZ pcaZ;
	pcaZ.x = sc * pz(0);
	pcaZ.y = sc * pz(1);
	pcaZ.z = sc * pz(2);


	//初始点云的主方向
	pcl::PointXYZ cp;
	cp.x = pcaCentroid(0);
	cp.y = pcaCentroid(1);
	cp.z = pcaCentroid(2);
	pcl::PointXYZ pcX;
	pcX.x = sc * eigenVectorsPCA(0, 0) + cp.x;
	pcX.y = sc * eigenVectorsPCA(1, 0) + cp.y;
	pcX.z = sc * eigenVectorsPCA(2, 0) + cp.z;
	pcl::PointXYZ pcY;
	pcY.x = sc * eigenVectorsPCA(0, 1) + cp.x;
	pcY.y = sc * eigenVectorsPCA(1, 1) + cp.y;
	pcY.z = sc * eigenVectorsPCA(2, 1) + cp.z;
	pcl::PointXYZ pcZ;
	pcZ.x = sc * eigenVectorsPCA(0, 2) + cp.x;
	pcZ.y = sc * eigenVectorsPCA(1, 2) + cp.y;
	pcZ.z = sc * eigenVectorsPCA(2, 2) + cp.z;

	viewer->addCube(bboxT, bboxQ, whd(0), whd(1), whd(2), "bouding box");
	viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "bouding box");
	viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "bouding box");

	viewer->addArrow(pcX, cp, 1.0, 0.0, 0.0, false, "arrow_X_v1");
	viewer->addArrow(pcY, cp, 0.0, 1.0, 0.0, false, "arrow_Y_v1");
	viewer->addArrow(pcZ, cp, 0.0, 0.0, 1.0, false, "arrow_Z_v1");

	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}

	return (0);
}