#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include <pcl/console/parse.h>  //pcl����̨����
#include <Eigen/src/StlSupport/details.h>
#include <Eigen/Core>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <pcl/point_cloud.h>

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
	eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1)); //У��������䴹ֱ
	eigenVectorsPCA.col(0) = eigenVectorsPCA.col(1).cross(eigenVectorsPCA.col(2));
	eigenVectorsPCA.col(1) = eigenVectorsPCA.col(2).cross(eigenVectorsPCA.col(0));

	std::cout << "����ֵva(3x1):\n" << eigenValuesPCA << std::endl;
	std::cout << "��������ve(3x3):\n" << eigenVectorsPCA << std::endl;
	std::cout << "���ĵ�(4x1):\n" << pcaCentroid << std::endl;
	/*
	// ��һ�ּ������Э�����������ֵ�����������ķ�ʽ:ͨ��pcl�е�pca�ӿڣ����£���������õ�����������������������
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPCAprojection (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PCA<pcl::PointXYZ> pca;
	pca.setInputCloud(cloudSegmented);
	pca.project(*cloudSegmented, *cloudPCAprojection);
	std::cerr << std::endl << "EigenVectors: " << pca.getEigenVectors() << std::endl;//������������
	std::cerr << std::endl << "EigenValues: " << pca.getEigenValues() << std::endl;//��������ֵ
	*/
	Eigen::Matrix4f tm = Eigen::Matrix4f::Identity();
	Eigen::Matrix4f tm_inv = Eigen::Matrix4f::Identity();
	tm.block<3, 3>(0, 0) = eigenVectorsPCA.transpose();   //R.
	tm.block<3, 1>(0, 3) = -1.0f * (eigenVectorsPCA.transpose()) *(pcaCentroid.head<3>());//  -R*t
	tm_inv = tm.inverse();

	std::cout << "�任����tm(4x4):\n" << tm << std::endl;
	std::cout << "������tm'(4x4):\n" << tm_inv << std::endl;

	pcl::PointCloud<pcl::PointXYZ>::Ptr tfCloud(new pcl::PointCloud<pcl::PointXYZ>);
	pointcloud.transformedCloud = tfCloud;
	pcl::transformPointCloud(*pointcloud.cloud, *pointcloud.transformedCloud, tm);

	pcl::PointXYZ min_p1, max_p1;
	Eigen::Vector3f c1, c;
	pcl::getMinMax3D(*pointcloud.transformedCloud, min_p1, max_p1);
	c1 = 0.5f*(min_p1.getVector3fMap() + max_p1.getVector3fMap());

	std::cout << "����c1(3x1):\n" << c1 << std::endl;

	Eigen::Affine3f tm_inv_aff(tm_inv);
	pcl::transformPoint(c1, c, tm_inv_aff);

	Eigen::Vector3f whd;
	pointcloud.whd = max_p1.getVector3fMap() - min_p1.getVector3fMap();
	whd = pointcloud.whd;
	pointcloud.sc = (pointcloud.whd(0) + pointcloud.whd(1) + pointcloud.whd(2)) / 3;  //����ƽ���߶ȣ����������������ͷ��С

	std::cout << "width1=" << pointcloud.whd(0) << endl;
	std::cout << "heght1=" << pointcloud.whd(1) << endl;
	std::cout << "depth1=" << pointcloud.whd(2) << endl;
	std::cout << "scale1=" << pointcloud.sc << endl;

	pointcloud.bboxQ = Eigen::Quaternionf::Identity();
	pointcloud.bboxT = c1;

	const Eigen::Quaternionf bboxQ(tm_inv.block<3, 3>(0, 0));
	const Eigen::Vector3f    bboxT(c);


	//�任��ԭ��ĵ���������
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


	//��ʼ���Ƶ�������
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

	cout << "�任����" << endl;
	cout << transform_scale.matrix() << endl;

	pcl::PointCloud<pcl::PointXYZ>::Ptr tfCloud(new pcl::PointCloud<pcl::PointXYZ>);
	ori_cloud.transformedCloud = tfCloud;
	pcl::transformPointCloud(*ori_cloud.cloud, *ori_cloud.transformedCloud, transform_scale);
	ori_cloud.cloud = ori_cloud.transformedCloud;

}

//���ü��̽�������
void keyboardEvent(const pcl::visualization::KeyboardEvent &event, void *nothing)
{
	if (event.getKeySym() == "space" && event.keyDown())
		next_iteration = true;
}
/*... ����������ʾ�����̿ո������ʱ���ſ�ִ��ICP���� ... */

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



	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;  //����ICP��ʵ����
	icp.setInputSource(align.cloud);
	icp.setInputTarget(refer.cloud);
	icp.setMaxCorrespondenceDistance(100);
	icp.setTransformationEpsilon(1e-10);
	icp.setEuclideanFitnessEpsilon(0.001);
	icp.setMaximumIterations(100);
	icp.align(*final);

	
	

	boost::shared_ptr<pcl::visualization::PCLVisualizer> view(new pcl::visualization::PCLVisualizer("icp test"));  //���崰�ڹ���ָ��
	int v1; //������������v1��v2������v1������ʾ��ʼλ�ã�v2������ʾ��׼����
	int v2;
	view->createViewPort(0.0, 0.0, 0.5, 1.0, v1);  //�ĸ����ڲ����ֱ��Ӧx_min,y_min,x_max.y_max.
	view->createViewPort(0.5, 0.0, 1.0, 1.0, v2);

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> sources_cloud_color(align.cloud, 250, 0, 0); //����Դ���Ƶ���ɫΪ��ɫ
	view->addPointCloud(align.cloud, sources_cloud_color, "sources_cloud_v1", v1);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> target_cloud_color(refer.cloud, 0, 250, 0);  //Ŀ�����Ϊ��ɫ
	view->addPointCloud(refer.cloud, target_cloud_color, "target_cloud_v1", v1); //��������ӵ�v1����

	view->setBackgroundColor(0.0, 0.05, 0.05, v1); //�����������ڵı���ɫ
	view->setBackgroundColor(0.05, 0.05, 0.05, v2);

	view->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "sources_cloud_v1");  //������ʾ��Ĵ�С
	view->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "target_cloud_v1");

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>aligend_cloud_color(final, 255, 255, 255);  //������׼���Ϊ��ɫ
	view->addPointCloud(final, aligend_cloud_color, "aligend_cloud_v2", v2);
	view->addPointCloud(refer.cloud, target_cloud_color, "target_cloud_v2", v2);

	view->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "aligend_cloud_v2");
	view->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "target_cloud_v2");

	view->registerKeyboardCallback(&keyboardEvent, (void*)NULL);  //���ü��̻ص�����
	int iterations = 0; //��������
	while (!view->wasStopped())
	{
		view->spinOnce();  //������ͼ
		if (next_iteration)
		{
			icp.align(*final);  //icp����
			cout << "has conveged:" << icp.hasConverged() << "score:" << icp.getFitnessScore() << endl;
			cout << "matrix:\n" << icp.getFinalTransformation() << endl;
			cout << "iteration = " << ++iterations;
			/*... ���icp.hasConverged=1,��˵��������׼�ɹ���icp.getFinalTransformation()������任����   ...*/
			if (iterations == 1000)  //��������������
				return 0;
			view->updatePointCloud(final, aligend_cloud_color, "aligend_cloud_v2");

		}
		next_iteration = false;  //���ε����������ȴ�����

	}


}