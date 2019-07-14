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
#include <Eigen/Core>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/vtk_lib_io.h>//loadPolygonFileOBJ所属头文件；
#include <Eigen/src/StlSupport/details.h>
#include <pcl/registration/icp.h>
#include <pcl/common/time.h>

using namespace std;
using namespace pcl;

/*
	myPointCloud(myPC)重新封装点云格式
		/
		|	cloud → pcl::PointXYZ格式点云数据
		|	transformedCloud → pcl::PointXYZ格式点云数据
	   /	bboxQ → 四元数(rotation of bouding box) 
	   \	bboxT → 三维向量(translation of bouding box)
		|	cenpot → 形心点
		|	pcaX、pcaY、pcaZ → 主方向点
		|	whd(width&height&depth) → 三维向量
		\
*/
struct myPC
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
	pcl::PointCloud<pcl::PointXYZ>::Ptr transformedCloud;
	//旋转矩阵
	Eigen::Quaternionf bboxQ;
	//平移矩阵
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
// Parameter: myPC & pointcloud → the point cloud need to be transformed to (0,0,0)
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


//************************************
// Method:    scaleTF
// FullName:  scaleTransForm( Zooming a point cloud by parameter "scale")
// Access:    public 
// Returns:   void
// Qualifier:
// Parameter: myPC & ori_cloud → the point cloud need to be zoomed
// Parameter: float scale → the zooming scale
//************************************
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

int main()
{

	pcl::StopWatch time;
	//参考点云
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_refer(new pcl::PointCloud<pcl::PointXYZ>);
	//对齐点云
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_align(new pcl::PointCloud<pcl::PointXYZ>);
	//visualization
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("myRegistration"));  //定义窗口共享指针
	int v1, v2; //创建两个视窗, v1显示空间变换后的效果, v2显示ICP配准最后的效果.
	viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);  //四个窗口参数分别对应x_min,y_min,x_max.y_max.
	viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);


	////创建mesh对象
	//pcl::PolygonMesh mesh;
	////读取polygon文件，obj格式读取为mesh
	//pcl::io::loadPolygonFile("zhengti-dianyun.obj", mesh);
	//
	//
	//
	////将mesh格式转换为PointCloud格式 方便读取
	//pcl::fromPCLPointCloud2(mesh.cloud, *cloud_align);
	////转存为可读取的PCD文件格式
	//pcl::io::savePCDFileASCII("zhengti-dianyun.pcd", *cloud_align);


	cout << "********************LOAD FILES PHASE********************" << endl;
	//refer-scale 1.6999	align-scale  128.799
	pcl::io::loadPCDFile("zhengti-abaqus-zhongxin.pcd", *cloud_refer);
	pcl::io::loadPCDFile("zhengti-dianyun-zhongxin.pcd", *cloud_align);

	cout << "Reference Point cloud data: " << cloud_refer->points.size() << " points" << endl;
	cout << "Aligned Point cloud data: " << cloud_align->points.size() << " points" << endl;

	//refer → 参考点云结构体 ； align → 对齐点云结构体
	myPC refer, align;
	refer.cloud = cloud_refer;
	align.cloud = cloud_align;

	cout << "********************SPATIAL TRANSFORM PHASE********************" << endl;
	//对refer和align点云进行空间变换到原点位置
	//spatialTF(refer);
	//spatialTF(align);

	cout << "********************SCALE TRANSFORM PHASE********************" << endl;
	//对对齐点云依照参考点云的尺寸大小进行缩放的倍数

	
	//spatialTF(refer);
	//spatialTF(align);

	

	float sca = refer.sc / align.sc;
	//将对齐点云依据缩放倍数进行缩放并变换到坐标原点
	//scaleTF(align, sca);
	//spatialTF(align);

	//-------------------------spatial visualization-----------------------
	//refer cloud phase
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> refer_handler_v1(refer.cloud, 0, 255, 0); //转换到原点的点云相关
	viewer->addPointCloud(refer.cloud, refer_handler_v1, "transformCloud_v1", v1);
	//viewer->addCube(refer.bboxT, refer.bboxQ, refer.whd(0), refer.whd(1), refer.whd(2), "bbox1_v1", v1);
	//viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "bbox1_v1");
	//viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, "bbox1_v1");
	//
	//viewer->addArrow(refer.pcaX, refer.cenpot, 1.0, 0.0, 0.0, false, "arrow_X_v1", v1);
	//viewer->addArrow(refer.pcaY, refer.cenpot, 0.0, 1.0, 0.0, false, "arrow_Y_v1", v1);
	//viewer->addArrow(refer.pcaZ, refer.cenpot, 0.0, 0.0, 1.0, false, "arrow_Z_v1", v1);

	//align cloud phase
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> align_handler_v1(align.cloud, 255, 0, 0);  //输入的初始点云相关
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


	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;  //创建ICP的实例类
	icp.setInputSource(align.cloud);
	icp.setInputTarget(refer.cloud);
	//设置对应点对之间的最大距离（此值对配准结果影响较大）。  
	icp.setMaxCorrespondenceDistance(1);
	// 设置两次变化矩阵之间的差值（一般设置为1e-10即可）
	icp.setTransformationEpsilon(1e-10);
	//设置收敛条件是均方误差和小于阈值， 停止迭代；
	icp.setEuclideanFitnessEpsilon(0.0001);
	//设置最大迭代次数
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
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> refer_handler_v2(refer.cloud, 0, 255, 0); //转换到原点的点云相关
	viewer->addPointCloud(refer.cloud, refer_handler_v2, "transformCloud_v2", v2);
	//viewer->addCube(refer.bboxT, refer.bboxQ, refer.whd(0), refer.whd(1), refer.whd(2), "bbox1_v2", v2);
	//viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "bbox1_v2");
	//viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, "bbox1_v2");
	//
	//viewer->addArrow(refer.pcaX, refer.cenpot, 1.0, 0.0, 0.0, false, "arrow_X_v2", v2);
	//viewer->addArrow(refer.pcaY, refer.cenpot, 0.0, 1.0, 0.0, false, "arrow_Y_v2", v2);
	//viewer->addArrow(refer.pcaZ, refer.cenpot, 0.0, 0.0, 1.0, false, "arrow_Z_v2", v2);
	
	//align cloud phase
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> align_handler_v2(align.cloud, 255, 0, 0);  //输入的初始点云相关
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