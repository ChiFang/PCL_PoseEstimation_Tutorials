

#define PCL_NO_PRECOMPILE // by ancre

#define PCL_SUB_FILENAME_LENGTH 4 // by ancre

#define GEN_MATRIX_DEBUG 0	// by ancre

#include <Eigen/Core>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/time.h>
#include <pcl/console/print.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>


#include <iostream>
#include <pcl/io/io.h>
#include <pcl/visualization/cloud_viewer.h>

#include <pcl/io/ply_io.h>

#include <pcl/common/transforms.h>


#include <pcl/console/parse.h>
#include <string>         // std::string



// Types
enum FORMAT { PLY = 0, PCD, UNKNOWN };
enum ROTATE { X = 0, Y, Z, UNKNOW };

typedef pcl::PointXYZRGB PointColor; // by ancre

typedef pcl::PointNormal PointNT;
typedef pcl::PointCloud<PointNT> PointCloudT;
typedef pcl::FPFHSignature33 FeatureT;
typedef pcl::FPFHEstimationOMP<PointNT,PointNT,FeatureT> FeatureEstimationT;
typedef pcl::PointCloud<FeatureT> FeatureCloudT;
typedef pcl::visualization::PointCloudColorHandlerCustom<PointNT> ColorHandlerT;


void viewerOneOff(pcl::visualization::PCLVisualizer& viewer)
{
	// set background to black (R = 0, G = 0, B = 0)
	viewer.setBackgroundColor(0, 0, 0);
}

void viewerPsycho(pcl::visualization::PCLVisualizer& viewer)
{
	// you can add something here, ex:  add text in viewer
}

int ViewPCD(const std::string a_sFileName)
{
	pcl::PointCloud<PointColor>::Ptr cloud(new pcl::PointCloud<PointColor>);

	// Load .pcd file from argv[1]
	int ret = pcl::io::loadPCDFile(a_sFileName, *cloud);
	if (ret < 0) {
		PCL_ERROR("Couldn't read file %s\n", a_sFileName);
		return -1;
	}

	pcl::visualization::CloudViewer viewer("Cloud Viewer");

	// blocks until the cloud is actually rendered
	viewer.showCloud(cloud);

	// use the following functions to get access to the underlying more advanced/powerful
	// PCLVisualizer

	// This will only get called once
	viewer.runOnVisualizationThreadOnce(viewerOneOff);

	// This will get called once per visualization iteration
	viewer.runOnVisualizationThread(viewerPsycho);
	while (!viewer.wasStopped()) {
		// you can also do cool processing here
		// FIXME: Note that this is running in a separate thread from viewerPsycho
		// and you should guard against race conditions yourself...
	}

	return 0;
}

int ExtractFileFormat(const std::string a_sFileName)
{
	// Fetch point cloud filename in arguments | Works with PCD and PLY files
	bool file_is_pcd = false;
	int lFormat = FORMAT::UNKNOWN;
	int lStrLen = a_sFileName.length();

	std::size_t found = a_sFileName.find(".ply");
	if (found == lStrLen- PCL_SUB_FILENAME_LENGTH)
	{
		std::cout << "PLY files" << '\n';
		lFormat = FORMAT::PLY;
		return lFormat;
	}

	found = a_sFileName.find(".pcd");
	if (found == lStrLen - PCL_SUB_FILENAME_LENGTH)
	{
		std::cout << "PCD files" << '\n';
		lFormat = FORMAT::PCD;
		return lFormat;
	}

	std::cout << "Fail! Must be .ply or .pcd file" << '\n';
	lFormat = FORMAT::UNKNOWN;

	return lFormat;
}

bool LoadPointCloudFromFile(const std::string a_sFileName, PointCloudT::Ptr a_ptPointCloudData)
{
	// Fetch point cloud filename in arguments | Works with PCD and PLY files
	int lFormat = ExtractFileFormat(a_sFileName);

	switch (lFormat) {
		case FORMAT::PCD:
			if (pcl::io::loadPCDFile<PointNT>(a_sFileName, *a_ptPointCloudData) < 0)
			{
				pcl::console::print_error("Error loading Point Cloud file!\n");
				return false;
			}
			return true;
			break;
		case FORMAT::PLY:
			if (pcl::io::loadPLYFile(a_sFileName, *a_ptPointCloudData) < 0)
			{
				pcl::console::print_error("Error loading Point Cloud file!\n");
				return false;
			}
			return true;
			break;
		default:
			return false;
	}
	return 0;
}

bool LoadPointCloudFromFile(const std::string a_sFileName, pcl::PointCloud<pcl::PointXYZ>::Ptr a_ptPointCloudData)
{
	// Fetch point cloud filename in arguments | Works with PCD and PLY files
	int lFormat = ExtractFileFormat(a_sFileName);

	switch (lFormat) {
	case FORMAT::PCD:
		if (pcl::io::loadPCDFile(a_sFileName, *a_ptPointCloudData) < 0)
		{
			pcl::console::print_error("Error loading Point Cloud file!\n");
			return false;
		}
		return true;
		break;
	case FORMAT::PLY:
		if (pcl::io::loadPLYFile(a_sFileName, *a_ptPointCloudData) < 0)
		{
			pcl::console::print_error("Error loading Point Cloud file!\n");
			return false;
		}
		return true;
		break;
	default:
		return false;
	}
	return 0;
}

Eigen::Matrix4f GetRotateMatrix(const float a_fTheta, int a_Mode)
{
	Eigen::Matrix4f tMatrix_4x4 = Eigen::Matrix4f::Identity();
	float fThetaTmp = a_fTheta * (M_PI / 180); // The angle of rotation from degree to radians
#if GEN_MATRIX_DEBUG > 0
	std::cout << tMatrix_4x4 << std::endl;
#endif

	switch (a_Mode) {
	case ROTATE::X:
		tMatrix_4x4(1, 1) = cos(fThetaTmp);
		tMatrix_4x4(1, 2) = -sin(fThetaTmp);
		tMatrix_4x4(2, 1) = sin(fThetaTmp);
		tMatrix_4x4(2, 2) = cos(fThetaTmp);
		break;
	case ROTATE::Y:
		tMatrix_4x4(0, 0) = cos(fThetaTmp);
		tMatrix_4x4(0, 2) = sin(fThetaTmp);
		tMatrix_4x4(2, 0) = -sin(fThetaTmp);
		tMatrix_4x4(2, 2) = cos(fThetaTmp);
		break;
	case ROTATE::Z:
		tMatrix_4x4(0, 0) = cos(fThetaTmp);
		tMatrix_4x4(0, 1) = -sin(fThetaTmp);
		tMatrix_4x4(1, 0) = sin(fThetaTmp);
		tMatrix_4x4(1, 1) = cos(fThetaTmp);
		break;
	default:
		std::cout << "error rotate mode" << std::endl;
		return tMatrix_4x4;
	}
#if GEN_MATRIX_DEBUG > 0
	std::cout << tMatrix_4x4 << std::endl;
#endif
	return tMatrix_4x4;
}

Eigen::Matrix4f GetTranslateMatrix(const float a_X, const float a_Y, const float a_Z)
{
	Eigen::Matrix4f tMatrix_4x4 = Eigen::Matrix4f::Identity();
#if GEN_MATRIX_DEBUG > 0
	std::cout << tMatrix_4x4 << std::endl;
#endif
	tMatrix_4x4(0, 3) = a_X;
	tMatrix_4x4(1, 3) = a_Y;
	tMatrix_4x4(2, 3) = a_Z;
#if GEN_MATRIX_DEBUG > 0
	std::cout << tMatrix_4x4 << std::endl;
#endif
	return tMatrix_4x4;
}


// This is the main function
int transform_test_func(const std::string a_sFileName)
{

	// Fetch point cloud filename in arguments | Works with PCD and PLY files
	// Load file | Works with PCD and PLY files
	pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZ>());
	bool bOpen = LoadPointCloudFromFile(a_sFileName, source_cloud);
	if(!bOpen)
	{
		std::cout << "Error loading point cloud " << a_sFileName << std::endl << std::endl;
		return -1;
	}

	/* Reminder: how transformation matrices work :

	|-------> This column is the translation
	| 1 0 0 x |  \
	| 0 1 0 y |   }-> The identity 3x3 matrix (no rotation) on the left
	| 0 0 1 z |  /
	| 0 0 0 1 |    -> We do not use this line (and it has to stay 0,0,0,1)

	METHOD #1: Using a Matrix4f
	This is the "manual" method, perfect to understand but error prone !
	*/
	Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();
	Eigen::Matrix4f tMatrixTmp = Eigen::Matrix4f::Identity();

	// Define a rotation matrix (see https://en.wikipedia.org/wiki/Rotation_matrix)
	tMatrixTmp = GetRotateMatrix(45, ROTATE::Z);
	transform_1 = tMatrixTmp*transform_1;

	// Define a translation of 2.5 meters on the x axis.
	tMatrixTmp = GetTranslateMatrix(2.5, 0, 0);
	transform_1 = tMatrixTmp*transform_1;

	// Print the transformation
	printf("Method #1: using a Matrix4f\n");
	std::cout << transform_1 << std::endl;

	// Executing the transformation
	pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>());
	// You can either apply transform_1 or transform_2; they are the same
	pcl::transformPointCloud(*source_cloud, *transformed_cloud, transform_1);

	// Visualization
	printf("\nPoint cloud colors :  white  = original point cloud\n"
		"                        red  = transformed point cloud\n");
	pcl::visualization::PCLVisualizer viewer("Matrix transformation example");

	// Define R,G,B colors for the point cloud
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler(source_cloud, 255, 255, 255);
	// We add the point cloud to the viewer and pass the color handler
	viewer.addPointCloud(source_cloud, source_cloud_color_handler, "original_cloud");
	// viewer.addPointCloud(source_cloud, ColorHandlerT(source_cloud, 0.0, 255.0, 0.0), "original_cloud"); // test

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> transformed_cloud_color_handler(transformed_cloud, 230, 20, 20); // Red
	viewer.addPointCloud(transformed_cloud, transformed_cloud_color_handler, "transformed_cloud");

#if 1 // save transformed_cloud
	pcl::io::savePLYFileASCII("output_transformed.ply", *transformed_cloud);
#endif

	viewer.addCoordinateSystem(1.0, "cloud", 0);
	viewer.setBackgroundColor(0.05, 0.05, 0.05, 0); // Setting background to a dark grey
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "original_cloud");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "transformed_cloud");
	//viewer.setPosition(800, 400); // Setting visualiser window position

	while (!viewer.wasStopped()) { // Display the visualiser until 'q' key is pressed
		viewer.spinOnce();
	}

	return 0;
}



// This is the main function
int transform_test(const std::string a_sFileName)
{

	// Fetch point cloud filename in arguments | Works with PCD and PLY files
	std::vector<int> filenames;
	bool file_is_pcd = false;

	// Load file | Works with PCD and PLY files
	pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZ>());
	// PointCloudT::Ptr source_cloud(new PointCloudT);

	if (file_is_pcd) {
		if (pcl::io::loadPCDFile(a_sFileName, *source_cloud) < 0) {
			std::cout << "Error loading point cloud " << a_sFileName << std::endl << std::endl;
			return -1;
		}
	}
	else {
		if (pcl::io::loadPLYFile(a_sFileName, *source_cloud) < 0) {
			std::cout << "Error loading point cloud " << a_sFileName << std::endl << std::endl;
			return -1;
		}
	}

	/* Reminder: how transformation matrices work :

	|-------> This column is the translation
	| 1 0 0 x |  \
	| 0 1 0 y |   }-> The identity 3x3 matrix (no rotation) on the left
	| 0 0 1 z |  /
	| 0 0 0 1 |    -> We do not use this line (and it has to stay 0,0,0,1)

	METHOD #1: Using a Matrix4f
	This is the "manual" method, perfect to understand but error prone !
	*/
	Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();

	// Define a rotation matrix (see https://en.wikipedia.org/wiki/Rotation_matrix)
	float theta = M_PI / 4; // The angle of rotation in radians
	transform_1(0, 0) = cos(theta);
	transform_1(0, 1) = -sin(theta);
	transform_1(1, 0) = sin(theta);
	transform_1(1, 1) = cos(theta);
	//    (row, column)

	// Define a translation of 2.5 meters on the x axis.
	transform_1 (0,3) = 2.5;

	// Print the transformation
	printf("Method #1: using a Matrix4f\n");
	std::cout << transform_1 << std::endl;

	/*  METHOD #2: Using a Affine3f
	This method is easier and less error prone
	*/
	Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();

	// Define a translation of 2.5 meters on the x axis.
	transform_2.translation() << 2.5, 0.0, 0.0;

	// The same rotation matrix as before; theta radians arround Z axis
	transform_2.rotate(Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitZ()));

	// Print the transformation
	printf("\nMethod #2: using an Affine3f\n");
	std::cout << transform_2.matrix() << std::endl;

	// Executing the transformation
	pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>());
	// You can either apply transform_1 or transform_2; they are the same
	pcl::transformPointCloud(*source_cloud, *transformed_cloud, transform_1);

	// Visualization
	printf("\nPoint cloud colors :  white  = original point cloud\n"
		"                        red  = transformed point cloud\n");
	pcl::visualization::PCLVisualizer viewer("Matrix transformation example");

	// Define R,G,B colors for the point cloud
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler(source_cloud, 255, 255, 255);
	// We add the point cloud to the viewer and pass the color handler
	viewer.addPointCloud(source_cloud, source_cloud_color_handler, "original_cloud");
	// viewer.addPointCloud(source_cloud, ColorHandlerT(source_cloud, 0.0, 255.0, 0.0), "original_cloud"); // test

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> transformed_cloud_color_handler(transformed_cloud, 230, 20, 20); // Red
	viewer.addPointCloud(transformed_cloud, transformed_cloud_color_handler, "transformed_cloud");

#if 1 // save transformed_cloud
	pcl::io::savePLYFileASCII("output_transformed.ply", *transformed_cloud);
#endif

	viewer.addCoordinateSystem(1.0, "cloud", 0);
	viewer.setBackgroundColor(0.05, 0.05, 0.05, 0); // Setting background to a dark grey
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "original_cloud");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "transformed_cloud");
	//viewer.setPosition(800, 400); // Setting visualiser window position

	while (!viewer.wasStopped()) { // Display the visualiser until 'q' key is pressed
		viewer.spinOnce();
	}

	return 0;
}

// Align a rigid object to a scene with clutter and occlusions
int main (int argc, char **argv)
{
	// Point clouds
	PointCloudT::Ptr object (new PointCloudT);
	PointCloudT::Ptr object_aligned (new PointCloudT);
	PointCloudT::Ptr scene (new PointCloudT);
	FeatureCloudT::Ptr object_features (new FeatureCloudT);
	FeatureCloudT::Ptr scene_features (new FeatureCloudT);
 
#if 0 // test by ancre
	PointCloudT::Ptr data(new PointCloudT);

	bool bFlag = LoadPointCloudFromFile("Test5.ply", data);
	pcl::visualization::PCLVisualizer visu_test("test");
	visu_test.addPointCloud(data, ColorHandlerT(data, 0.0, 255.0, 255.0), "data");
	visu_test.spin();
#endif

	// transform_test_func("output.ply");
	// GetRotateMatrix(45, ROTATE::Z);
	// ViewPCD("scene.pcd"); // test by ancre
	// transform_test("output.ply"); // test by ancre
  
	// Load object and scene
	pcl::console::print_highlight("Loading point clouds...\n");

	bool bObject = LoadPointCloudFromFile("toy.ply", object);
	bool bScene = LoadPointCloudFromFile("toy_transformed.ply", scene);

	if (!bObject || !bScene)
	{	// loading fail
		return -1;
	}

	pcl::io::savePCDFileASCII("test_log.pcd", *object); // test log by ancre
	pcl::io::savePLYFileASCII("test_log.ply", *object); // test log by ancre

	// Downsample
	pcl::console::print_highlight ("Downsampling...\n");
	pcl::VoxelGrid<PointNT> grid;
	const float leaf = 0.005f;
	grid.setLeafSize (leaf, leaf, leaf);
	grid.setInputCloud (object);
	grid.filter (*object);
	grid.setInputCloud (scene);
	grid.filter (*scene);

#if 1 // test by ancre
	pcl::visualization::PCLVisualizer visu_t1("diff");
	visu_t1.addPointCloud(scene, ColorHandlerT(scene, 0.0, 255.0, 0.0), "scene");
	visu_t1.addPointCloud(object, ColorHandlerT(object, 255.0, 0.0, 0.0), "object");
	visu_t1.spin();
#endif

	// Estimate normals for scene
	pcl::console::print_highlight ("Estimating scene normals...\n");
	pcl::NormalEstimationOMP<PointNT,PointNT> nest;
	// nest.setRadiusSearch (0.01);
	nest.setRadiusSearch(0.005);
	nest.setInputCloud (scene);
	nest.compute (*scene);
  
	// Estimate features
	pcl::console::print_highlight ("Estimating features...\n");
	FeatureEstimationT fest;
	// fest.setRadiusSearch (0.025);
	fest.setRadiusSearch(0.015); // ancre test
	fest.setInputCloud (object);
	fest.setInputNormals (object);
	fest.compute (*object_features);
	fest.setInputCloud (scene);
	fest.setInputNormals (scene);
	fest.compute (*scene_features);
  
	// Perform alignment
	pcl::console::print_highlight ("Starting alignment...\n");
	pcl::SampleConsensusPrerejective<PointNT,PointNT,FeatureT> align;
	align.setInputSource (object);
	align.setSourceFeatures (object_features);
	align.setInputTarget (scene);
	align.setTargetFeatures (scene_features);
	// align.setMaximumIterations (50000); // Number of RANSAC iterations
	align.setMaximumIterations(2000); // Number of RANSAC iterations >>>>> ancre test
	align.setNumberOfSamples (3); // Number of points to sample for generating/prerejecting a pose
	align.setCorrespondenceRandomness (5); // Number of nearest features to use
	align.setSimilarityThreshold (0.9f); // Polygonal edge length similarity threshold
	align.setSimilarityThreshold(0.9f); // Polygonal edge length similarity threshold >>>>> ancre test
	align.setMaxCorrespondenceDistance (2.5f * leaf); // Inlier threshold
	align.setInlierFraction (0.25f); // Required inlier fraction for accepting a pose hypothesis
	{
		pcl::ScopeTime t("Alignment");
		align.align (*object_aligned);
	}
  
	if (align.hasConverged ())
	{
		// Print results
		printf ("\n");
		Eigen::Matrix4f transformation = align.getFinalTransformation ();
		pcl::console::print_info ("    | %6.3f %6.3f %6.3f | \n", transformation (0,0), transformation (0,1), transformation (0,2));
		pcl::console::print_info ("R = | %6.3f %6.3f %6.3f | \n", transformation (1,0), transformation (1,1), transformation (1,2));
		pcl::console::print_info ("    | %6.3f %6.3f %6.3f | \n", transformation (2,0), transformation (2,1), transformation (2,2));
		pcl::console::print_info ("\n");
		pcl::console::print_info ("t = < %0.3f, %0.3f, %0.3f >\n", transformation (0,3), transformation (1,3), transformation (2,3));
		pcl::console::print_info ("\n");
		pcl::console::print_info ("Inliers: %i/%i\n", align.getInliers().size(), object->size());
    
		// Show alignment
		pcl::visualization::PCLVisualizer visu("Alignment");
		visu.addPointCloud (scene, ColorHandlerT (scene, 0.0, 255.0, 0.0), "scene");
		visu.addPointCloud (object_aligned, ColorHandlerT (object_aligned, 0.0, 0.0, 255.0), "object_aligned");
		visu.spin ();
	}
	else
	{
		pcl::console::print_error ("Alignment failed!\n");
		return (1);
	}
#if 0 // ancre disable test 
#endif
	return (0);
}
