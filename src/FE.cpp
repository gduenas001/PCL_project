// General 
#include <iostream>
#include <iomanip>
#include <sstream>
#include <fstream>
#include <map>
#include <Eigen/Dense>
#include <vector>
#include <string>

// PCL specific
#include <pcl/point_types.h>
#include <pcl/search/organized.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/common.h>
#include <pcl/common/distances.h>
#include <pcl/common/common_headers.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/segmentation/sac_segmentation.h>


// User specific
#include "classes_and_structs.h"
#include "cylinder_segmentation.h"
#include "read_transformations.h"
#include "to_string.h"
#include "compute_cloud_parameters.h"
#include "create_clusters.h"
#include "initialize_parameters.h"
#include "read_pcd_file.h"
#include "configure_viewer.h"

using namespace std;
using namespace pcl;




// *********************  MAIN  ********************* //
int main (int argc, char *argv[])
{


// Declare variables
int num_frames, initial_frame;
double meanX, meanY, meanZ, varX, varY, varZ;
string cloud_cylinder_id;
vector <Frame> frames;
vector <Landmark> landmarks;
vector <Cylinder> cylinders;
vector <Eigen::Matrix4d> T= read_transformations();
map<string, double> cloud_parameters;
parameters P;

// Pass command line arguments
istringstream (argv[1]) >> initial_frame;
istringstream (argv[2]) >> num_frames;

// Set the parameters and thresholds
initialize_parameters(P);

pcl::visualization::PCLVisualizer viewer= configure_viewer();


for (int epoch= initial_frame; epoch <= num_frames; ++epoch)
{

  	// Create vaiables
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

	cout<< "Start reading PCD file..."<< endl;
	read_pcd_file(cloud, epoch);
	cout<< "End reading PCD file."<< endl;

	// Visualize point cloud - White cloud
	viewer.removeAllPointClouds();
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> 
					white_color (cloud, 255, 255, 255);
	viewer.addPointCloud <pcl::PointXYZ> (cloud, white_color, "cloud_original");

	// Extraction of clusters
	vector<pcl::PointIndices> clusters_indices;
	create_clusters(cloud, clusters_indices, P);
	

	vector <pcl::PointCloud<pcl::PointXYZ> ::Ptr> clusters; // Vector of pointclouds pointers
	
	// Extract sub cluster from indeces
	pcl::ExtractIndices<pcl::PointXYZ> extract;
	extract.setInputCloud (cloud);
	extract.setNegative (false);

	int numClusters= 0;
	for (int cluster_index = 0; cluster_index < clusters_indices.size(); ++cluster_index)
	{
		// Extract the inliers
	    pcl::PointIndices::Ptr cluster_indeces_pointer
	    					(new pcl::PointIndices ( clusters_indices[cluster_index] ));
    	extract.setIndices (cluster_indeces_pointer);

    	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster 
    						(new pcl::PointCloud<pcl::PointXYZ>);
    	extract.filter (*cloud_cluster);

	    compute_cloud_parameters(cloud_cluster, cloud_parameters);

	    // Check if cluster is valid
	    if ( cloud_parameters["density"] > P.densityThreshold  		   &&  
	    	 cloud_parameters["sdX"] < P.sdXYThreshold				   &&
	    	 cloud_parameters["sdY"] < P.sdXYThreshold				   &&
	    	 cloud_parameters["slendernessX"] > P.slindernessThreshold &&  
	    	 cloud_parameters["slendernessY"] > P.slindernessThreshold )
	    {
	    	// Add cluster
		    ++numClusters; 
		    clusters.push_back(cloud_cluster);

		    //Save cluster
		    
		    cout << "Cluster " << numClusters << " ----> " 
		    					<< clusters[numClusters-1]->points.size() << " points." << endl;
		    cout << "density = " << cloud_parameters["density"] << endl;
		    cout << "SD in X = " << cloud_parameters["sdX"] << endl;
		    cout << "SD in Y = " << cloud_parameters["sdY"] << endl;
		    cout << "SD in Z = " << cloud_parameters["sdZ"] << endl;
		    cout << "slendeness in X = " << cloud_parameters["slendernessX"] << endl;
		    cout << "slendeness in Y = " << cloud_parameters["slendernessY"] << endl << endl;

	    }
	} // End of loop over clusters

	cylinders.clear();
	cylinder_segmentation( cylinders, clusters, T[epoch]);

	// Update the cylinders' visualization
	int counter= 0;
	for (vector <Cylinder>::iterator it= cylinders.begin(),
		it_end= cylinders.end();
		it != it_end; ++it, ++counter)
	{
		cloud_cylinder_id= "cloud_cylinder_" + patch::to_string( counter );
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red_color ( it->cloud, 255, 0, 0);
		viewer.addPointCloud<pcl::PointXYZ> (it->cloud, red_color, cloud_cylinder_id);
		viewer.setPointCloudRenderingProperties 
								(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, cloud_cylinder_id);
		cout<< "Cylinder "<< counter<< " ----> "<< it->cloud->points.size()<<  endl;
	}

	cout<< "-----------------------------------"<< endl;
	cout << "# epoch = " << epoch << endl;
	cout << "# clusters = " << numClusters << endl;
	cout << "# cylinders = " << cylinders.size() << endl;
	cout<< "-----------------------------------"<< endl;

	//Vierwer
	viewer.spinOnce ();
}

} // End of Main