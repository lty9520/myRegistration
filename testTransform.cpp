#include <iostream>
#include <string>

#include <vtkAutoInit.h>        

VTK_MODULE_INIT(vtkRenderingOpenGL);  //��������ο���http://tieba.baidu.com/p/4551950404#93116920200l��

//VTK_MODULE_INIT(vtkRenderingOpenGL2);  //�Լ���PCL1.8.1��װ���������OpenGL,������һ����Ҫ�ĳ�OpenGL.

VTK_MODULE_INIT(vtkInteractionStyle); //�ⲿ���������opengl32.lib�����Լ��Ļ�Ҫ���vfw32.lib��Ĳ�֪��Ҫ��Ҫ��

VTK_MODULE_INIT(vtkRenderingFreeType);

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Core>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/vtk_lib_io.h>//loadPolygonFileOBJ����ͷ�ļ���
#include <Eigen/src/StlSupport/details.h>
#include <pcl/registration/icp.h>
#include <pcl/common/time.h>

using namespace std;
using namespace pcl;

/*
	myPointCloud(myPC)���·�װ���Ƹ�ʽ
		/
		|	cloud �� pcl::PointXYZ��ʽ��������
		|	transformedCloud �� pcl::PointXYZ��ʽ��������
	   /	bboxQ �� ��Ԫ��(rotation of bouding box) 
	   \	bboxT �� ��ά����(translation of bouding box)
		|	cenpot �� ���ĵ�
		|	pcaX��pcaY��pcaZ �� �������
		|	whd(width&height&depth) �� ��ά����
		\
*/
struct myPC
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
	pcl::PointCloud<pcl::PointXYZ>::Ptr transformedCloud;
	//��ת����
	Eigen::Quaternionf bboxQ;
	//ƽ�ƾ���
	Eigen::Vector3f bboxT;
	pcl::PointXYZ cenpot, pcaX, pcaY, pcaZ;
	Eigen::Vector3f whd;
	float sc;
};

//************************************
// Method:    spatialTF
// FullName:  spatialTransForm( Transform a point cloud to (0,0,0))
// Access:    public 
// Returns:   void
// Qualifier:
// Parameter: myPC & pointcloud �� the point cloud need to be transformed to (0,0,0)
//************************************
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


//************************************
// Method:    scaleTF
// FullName:  scaleTransForm( Zooming a point cloud by parameter "scale")
// Access:    public 
// Returns:   void
// Qualifier:
// Parameter: myPC & ori_cloud �� the point cloud need to be zoomed
// Parameter: float scale �� the zooming scale
//************************************
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

int main()
{

	pcl::StopWatch time;
	//�ο�����
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_refer(new pcl::PointCloud<pcl::PointXYZ>);
	//�������
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_align(new pcl::PointCloud<pcl::PointXYZ>);
	//visualization
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("myRegistration"));  //���崰�ڹ���ָ��
	int v1, v2; //���������Ӵ�, v1��ʾ�ռ�任���Ч��, v2��ʾICP��׼����Ч��.
	viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);  //�ĸ����ڲ����ֱ��Ӧx_min,y_min,x_max.y_max.
	viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);


	////����mesh����
	//pcl::PolygonMesh mesh;
	////��ȡpolygon�ļ���obj��ʽ��ȡΪmesh
	//pcl::io::loadPolygonFile("zhengti-dianyun.obj", mesh);
	//
	//
	//
	////��mesh��ʽת��ΪPointCloud��ʽ �����ȡ
	//pcl::fromPCLPointCloud2(mesh.cloud, *cloud_align);
	////ת��Ϊ�ɶ�ȡ��PCD�ļ���ʽ
	//pcl::io::savePCDFileASCII("zhengti-dianyun.pcd", *cloud_align);


	cout << "********************LOAD FILES PHASE********************" << endl;
	//refer-scale 1.6999	align-scale  128.799
	pcl::io::loadPCDFile("zhengti-abaqus-zhongxin.pcd", *cloud_refer);
	pcl::io::loadPCDFile("zhengti-dianyun-zhongxin.pcd", *cloud_align);

	cout << "Reference Point cloud data: " << cloud_refer->points.size() << " points" << endl;
	cout << "Aligned Point cloud data: " << cloud_align->points.size() << " points" << endl;

	//refer �� �ο����ƽṹ�� �� align �� ������ƽṹ��
	myPC refer, align;
	refer.cloud = cloud_refer;
	align.cloud = cloud_align;

	cout << "********************SPATIAL TRANSFORM PHASE********************" << endl;
	//��refer��align���ƽ��пռ�任��ԭ��λ��
	//spatialTF(refer);
	//spatialTF(align);

	cout << "********************SCALE TRANSFORM PHASE********************" << endl;
	//�Զ���������ղο����Ƶĳߴ��С�������ŵı���

	
	//spatialTF(refer);
	//spatialTF(align);

	

	float sca = refer.sc / align.sc;
	//����������������ű����������Ų��任������ԭ��
	//scaleTF(align, sca);
	//spatialTF(align);

	//-------------------------spatial visualization-----------------------
	//refer cloud phase
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> refer_handler_v1(refer.cloud, 0, 255, 0); //ת����ԭ��ĵ������
	viewer->addPointCloud(refer.cloud, refer_handler_v1, "transformCloud_v1", v1);
	//viewer->addCube(refer.bboxT, refer.bboxQ, refer.whd(0), refer.whd(1), refer.whd(2), "bbox1_v1", v1);
	//viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "bbox1_v1");
	//viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, "bbox1_v1");
	//
	//viewer->addArrow(refer.pcaX, refer.cenpot, 1.0, 0.0, 0.0, false, "arrow_X_v1", v1);
	//viewer->addArrow(refer.pcaY, refer.cenpot, 0.0, 1.0, 0.0, false, "arrow_Y_v1", v1);
	//viewer->addArrow(refer.pcaZ, refer.cenpot, 0.0, 0.0, 1.0, false, "arrow_Z_v1", v1);

	//align cloud phase
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> align_handler_v1(align.cloud, 255, 0, 0);  //����ĳ�ʼ�������
	viewer->addPointCloud(align.cloud, align_handler_v1, "cloud_v1", v1);
	//viewer->addCube(align.bboxT, align.bboxQ, align.whd(0), align.whd(1), align.whd(2), "bbox_v1", v1);
	//viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "bbox_v1");
	//viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "bbox_v1");
	//
	//viewer->addArrow(align.pcaX, align.cenpot, 1.0, 0.0, 0.0, false, "arrow_x_v1", v1);
	//viewer->addArrow(align.pcaY, align.cenpot, 0.0, 1.0, 0.0, false, "arrow_y_v1", v1);
	//viewer->addArrow(align.pcaZ, align.cenpot, 0.0, 0.0, 1.0, false, "arrow_z_v1", v1);

	//fundamental setting
	viewer->addCoordinateSystem(0.5f*refer.sc);
	viewer->setBackgroundColor(1.0, 1.0, 1.0, v1);

	
	

	cout << "********************ICP REGISTRATION PHASE********************" << endl;


	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;  //����ICP��ʵ����
	icp.setInputSource(align.cloud);
	icp.setInputTarget(refer.cloud);
	//���ö�Ӧ���֮��������루��ֵ����׼���Ӱ��ϴ󣩡�  
	icp.setMaxCorrespondenceDistance(1);
	// �������α仯����֮��Ĳ�ֵ��һ������Ϊ1e-10���ɣ�
	icp.setTransformationEpsilon(1e-10);
	//�������������Ǿ�������С����ֵ�� ֹͣ������
	icp.setEuclideanFitnessEpsilon(0.0001);
	//��������������
	icp.setMaximumIterations(100000000);
	pcl::PointCloud<pcl::PointXYZ>::Ptr tempFinal(new pcl::PointCloud<pcl::PointXYZ>);
	icp.align(*tempFinal);
	
	align.cloud = tempFinal;
	cout << "*********----------SUB SPATIAL TRANSFORM PHASE----------*********" << endl;
	//spatialTF(align);
	Eigen::Affine3f transform_rota = Eigen::Affine3f::Identity();
	transform_rota.rotate(Eigen::AngleAxisf(M_PI, Eigen::Vector3f::UnitY()));
	Eigen::Affine3f transform_rota1 = Eigen::Affine3f::Identity();
	transform_rota1.rotate(Eigen::AngleAxisf( - M_PI / 6, Eigen::Vector3f::UnitZ()));
	//pcl::transformPointCloud(*align.transformedCloud, *align.transformedCloud, transform_rota);
	//pcl::transformPointCloud(*align.transformedCloud, *align.transformedCloud, transform_rota1);

	//Eigen::Matrix4f transformation = icp.getFinalTransformation();
	cout << "*********----------SUB ICP RESULT PHASE----------*********" << endl;
	cout << "has converged: " << icp.hasConverged() << endl;

	//Obtain the Euclidean fitness score (e.g.,the average of sum of squared distances from the source to the target) 
	cout << "RMS : " << sqrt(icp.getFitnessScore()) << endl;
	
	

	//Get the final transformation matrix estimated by the registration method. 
	cout << icp.getFinalTransformation() << endl;



	cout << "running time : " << time.getTime() << endl;
	
	//-------------------------ICP visualization-----------------------
	//refer cloud phase
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> refer_handler_v2(refer.cloud, 0, 255, 0); //ת����ԭ��ĵ������
	viewer->addPointCloud(refer.cloud, refer_handler_v2, "transformCloud_v2", v2);
	//viewer->addCube(refer.bboxT, refer.bboxQ, refer.whd(0), refer.whd(1), refer.whd(2), "bbox1_v2", v2);
	//viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "bbox1_v2");
	//viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, "bbox1_v2");
	//
	//viewer->addArrow(refer.pcaX, refer.cenpot, 1.0, 0.0, 0.0, false, "arrow_X_v2", v2);
	//viewer->addArrow(refer.pcaY, refer.cenpot, 0.0, 1.0, 0.0, false, "arrow_Y_v2", v2);
	//viewer->addArrow(refer.pcaZ, refer.cenpot, 0.0, 0.0, 1.0, false, "arrow_Z_v2", v2);
	
	//align cloud phase
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> align_handler_v2(align.cloud, 255, 0, 0);  //����ĳ�ʼ�������
	viewer->addPointCloud(align.cloud, align_handler_v2, "cloud_v2", v2);
	//viewer->addCube(align.bboxT, align.bboxQ, align.whd(0), align.whd(1), align.whd(2), "bbox_v2", v2);
	//viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "bbox_v2");
	//viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "bbox_v2");
	//
	//viewer->addArrow(align.pcaX, align.cenpot, 1.0, 0.0, 0.0, false, "arrow_x_v2", v2);
	//viewer->addArrow(align.pcaY, align.cenpot, 0.0, 1.0, 0.0, false, "arrow_y_v2", v2);
	//viewer->addArrow(align.pcaZ, align.cenpot, 0.0, 0.0, 1.0, false, "arrow_z_v2", v2);
	
	//fundamental setting
	viewer->addCoordinateSystem(0.5f*refer.sc);
	viewer->setBackgroundColor(1.0, 1.0, 1.0, v2);

	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
	}

	return 0;
}