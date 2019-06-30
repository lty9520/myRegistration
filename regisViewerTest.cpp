#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include <pcl/console/parse.h>  //pcl控制台解析
#include <Eigen/src/StlSupport/details.h>
#include <Eigen/Core>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/registration_visualizer.h>

using namespace std;
using namespace pcl;

bool next_iteration = false;

struct myPC
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
	pcl::PointCloud<pcl::PointXYZ>::Ptr transformedCloud;
	Eigen::Quaternionf bboxQ;
	Eigen::Vector3f bboxT;
	pcl::PointXYZ cenpot, pcaX, pcaY, pcaZ;
	Eigen::Vector3f whd;
	float sc;
};

void spatialTF(myPC & pointcloud)
{
	Eigen::Vector4f pcaCentroid;
	//pcl::PointCloud<pcl::PointXYZ>::Ptr ori_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	//pointcloud.cloud = ori_cloud;
	pcl::compute3DCentroid(*pointcloud.cloud, pcaCentroid);
	Eigen::Matrix3f covariance;
	pcl::computeCovarianceMatrixNormalized(*pointcloud.cloud, pcaCentroid, covariance);
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
	pointcloud.transformedCloud = tfCloud;
	pcl::transformPointCloud(*pointcloud.cloud, *pointcloud.transformedCloud, tm);

	pcl::PointXYZ min_p1, max_p1;
	Eigen::Vector3f c1, c;
	pcl::getMinMax3D(*pointcloud.transformedCloud, min_p1, max_p1);
	c1 = 0.5f*(min_p1.getVector3fMap() + max_p1.getVector3fMap());

	std::cout << "型心c1(3x1):\n" << c1 << std::endl;

	Eigen::Affine3f tm_inv_aff(tm_inv);
	pcl::transformPoint(c1, c, tm_inv_aff);

	Eigen::Vector3f whd;
	pointcloud.whd = max_p1.getVector3fMap() - min_p1.getVector3fMap();
	whd = pointcloud.whd;
	pointcloud.sc = (pointcloud.whd(0) + pointcloud.whd(1) + pointcloud.whd(2)) / 3;  //点云平均尺度，用于设置主方向箭头大小

	std::cout << "width1=" << pointcloud.whd(0) << endl;
	std::cout << "heght1=" << pointcloud.whd(1) << endl;
	std::cout << "depth1=" << pointcloud.whd(2) << endl;
	std::cout << "scale1=" << pointcloud.sc << endl;

	pointcloud.bboxQ = Eigen::Quaternionf::Identity();
	pointcloud.bboxT = c1;

	const Eigen::Quaternionf bboxQ(tm_inv.block<3, 3>(0, 0));
	const Eigen::Vector3f    bboxT(c);


	//变换到原点的点云主方向
	//pcl::PointXYZ op;
	pointcloud.cenpot.x = 0.0;
	pointcloud.cenpot.y = 0.0;
	pointcloud.cenpot.z = 0.0;
	Eigen::Vector3f px, py, pz;
	Eigen::Affine3f tm_aff(tm);
	pcl::transformVector(eigenVectorsPCA.col(0), px, tm_aff);
	pcl::transformVector(eigenVectorsPCA.col(1), py, tm_aff);
	pcl::transformVector(eigenVectorsPCA.col(2), pz, tm_aff);
	//pcl::PointXYZ pcaX;
	pointcloud.pcaX.x = pointcloud.sc * px(0);
	pointcloud.pcaX.y = pointcloud.sc * px(1);
	pointcloud.pcaX.z = pointcloud.sc * px(2);
	//pcl::PointXYZ pcaY;
	pointcloud.pcaY.x = pointcloud.sc * py(0);
	pointcloud.pcaY.y = pointcloud.sc * py(1);
	pointcloud.pcaY.z = pointcloud.sc * py(2);
	//pcl::PointXYZ pcaZ;
	pointcloud.pcaZ.x = pointcloud.sc * pz(0);
	pointcloud.pcaZ.y = pointcloud.sc * pz(1);
	pointcloud.pcaZ.z = pointcloud.sc * pz(2);


	//初始点云的主方向
	pcl::PointXYZ cp;
	cp.x = pcaCentroid(0);
	cp.y = pcaCentroid(1);
	cp.z = pcaCentroid(2);
	pcl::PointXYZ pcX;
	pcX.x = pointcloud.sc * eigenVectorsPCA(0, 0) + cp.x;
	pcX.y = pointcloud.sc * eigenVectorsPCA(1, 0) + cp.y;
	pcX.z = pointcloud.sc * eigenVectorsPCA(2, 0) + cp.z;
	pcl::PointXYZ pcY;
	pcY.x = pointcloud.sc * eigenVectorsPCA(0, 1) + cp.x;
	pcY.y = pointcloud.sc * eigenVectorsPCA(1, 1) + cp.y;
	pcY.z = pointcloud.sc * eigenVectorsPCA(2, 1) + cp.z;
	pcl::PointXYZ pcZ;
	pcZ.x = pointcloud.sc * eigenVectorsPCA(0, 2) + cp.x;
	pcZ.y = pointcloud.sc * eigenVectorsPCA(1, 2) + cp.y;
	pcZ.z = pointcloud.sc * eigenVectorsPCA(2, 2) + cp.z;
}


void scaleTF(myPC & ori_cloud, float scale)
{
	Eigen::Affine3f transform_scale = Eigen::Affine3f::Identity();
	transform_scale.scale(scale);

	cout << "变换矩阵：" << endl;
	cout << transform_scale.matrix() << endl;

	pcl::PointCloud<pcl::PointXYZ>::Ptr tfCloud(new pcl::PointCloud<pcl::PointXYZ>);
	ori_cloud.transformedCloud = tfCloud;
	pcl::transformPointCloud(*ori_cloud.cloud, *ori_cloud.transformedCloud, transform_scale);
	ori_cloud.cloud = ori_cloud.transformedCloud;

}

//设置键盘交互函数
void keyboardEvent(const pcl::visualization::KeyboardEvent &event, void *nothing)
{
	if (event.getKeySym() == "space" && event.keyDown())
		next_iteration = true;
}
/*... 上述函数表示当键盘空格键按下时，才可执行ICP计算 ... */

int main()
{

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_sources(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr final(new pcl::PointCloud<pcl::PointXYZ>);

	pcl::io::loadPCDFile("proj-final-fbx-dian.pcd", *cloud_target);
	pcl::io::loadPCDFile("huagong-final-1.5.pcd", *cloud_sources);



	cout << "Reference Point cloud data: " << cloud_target->points.size() << " points" << endl;
	cout << "Aligned Point cloud data: " << cloud_sources->points.size() << " points" << endl;

	myPC refer, align;
	refer.cloud = cloud_target;
	align.cloud = cloud_sources;
	cout << "********************spatial transform********************" << endl;
	spatialTF(refer);
	spatialTF(align);

	cout << "********************scale transform********************" << endl;
	float sca = refer.sc / align.sc;
	scaleTF(align, sca);
	spatialTF(align);



	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;  //创建ICP的实例类
	icp.setInputSource(align.cloud);
	icp.setInputTarget(refer.cloud);
	icp.setMaxCorrespondenceDistance(100);
	icp.setTransformationEpsilon(1e-10);
	icp.setEuclideanFitnessEpsilon(0.001);
	icp.setMaximumIterations(100);
	icp.align(*final);



	//pcl::RegistrationVisualizer<pcl::PointXYZ, pcl::PointXYZ> viewer ;
	boost::shared_ptr<pcl::RegistrationVisualizer<pcl::PointXYZ, pcl::PointXYZ>> viewer(new pcl::RegistrationVisualizer<pcl::PointXYZ, pcl::PointXYZ>);
	viewer->setRegistration(icp);
	viewer->setMaximumDisplayedCorrespondences(1000000);
	viewer->startDisplay();

	system("pause");
	return 0;

}