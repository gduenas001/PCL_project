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
#include <thread>


// User specific
#include "classes_and_structs.h"
#include "cylinder_segmentation.h"
#include "read_transformations.h"
#include "to_string.h"
#include "compute_cloud_parameters.h"
#include "create_clusters.h"
#include "read_pcd_file.h"
#include "configure_viewer.h"
#include "read_inputs.h"

#define VERBOSE false


using namespace std;
using namespace pcl;



void read_pcd_file_callback(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int epoch)
{
	// Read current file
	pcl::PCDReader reader;
	string filename2;
	stringstream sa;
	sa << setw(6) << setfill('0') << epoch;
	filename2= sa.str();
	reader.read ("../Data/pcd-files/KITTI/" + filename2 + ".pcd", *cloud);
	if (VERBOSE)
		cout<< "Read next cloud: "<< epoch<< endl;
}




// *********************  MAIN  ********************* //
int main (int argc, char *argv[])
{

map<string, float> PARAMS;
if (!read_inputs(PARAMS))
	cout<< "Error reading paramter files"<< endl;


// Declare variables
double meanX, meanY, meanZ, varX, varY, varZ;
string cloud_cylinder_id;
vector <Frame> frames;
vector <Landmark> landmarks;
vector <Cylinder> cylinders;
vector <Eigen::Matrix4d> T= read_transformations();
map<string, double> cloud_parameters;

int initial_frame= static_cast<int>(PARAMS["initial_frame"]);
int num_frames= static_cast<int>(PARAMS["final_frame"]);


pcl::visualization::PCLVisualizer viewer= configure_viewer();

// create vector of threads
vector<thread> threadVector;

// Read first cloud
pcl::PointCloud<pcl::PointXYZ>::Ptr next_cloud (new pcl::PointCloud<pcl::PointXYZ>);
threadVector.push_back(thread( read_pcd_file_callback, next_cloud, initial_frame ));	



for (int epoch= initial_frame; epoch <= num_frames; ++epoch)
{

	//------------------ READING PCD FILE ------------------//
  	// Create vaiables
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

	// join previous thread
	threadVector.at( threadVector.size()-1 ).join();
	cloud= next_cloud;

	// Start next thread
	next_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
	threadVector.push_back(thread (read_pcd_file_callback, next_cloud, epoch+1));

	// Reduce size of vector of threads
	if (threadVector.size() > 2)
		threadVector.erase( threadVector.begin() + 1 );
	//------------------------------------------------------//
	


	// Visualize point cloud - White cloud
	viewer.removeAllPointClouds();
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> 
					white_color (cloud, 255, 255, 255);
	viewer.addPointCloud <pcl::PointXYZ> (cloud, white_color, "cloud_original");

	// Extraction of clusters
	vector<pcl::PointIndices> clusters_indices;
	create_clusters(cloud, clusters_indices, PARAMS);
	 

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
	    if ( cloud_parameters["density"] > PARAMS["densityThreshold"]  		   &&  
	    	 cloud_parameters["sdX"] < PARAMS["sdXYThreshold"]				   &&
	    	 cloud_parameters["sdY"] < PARAMS["sdXYThreshold"]				   &&
	    	 cloud_parameters["slendernessX"] > PARAMS["slindernessThreshold"] &&  
	    	 cloud_parameters["slendernessY"] > PARAMS["slindernessThreshold"] )
	    {
	    	// Add cluster
		    ++numClusters; 
		    clusters.push_back(cloud_cluster);

		    //Save cluster
		    if (VERBOSE)
			{
			    cout << "Cluster " << numClusters << " ----> " 
			    					<< clusters[numClusters-1]->points.size() << " points." << endl;
			    cout << "density = " << cloud_parameters["density"] << endl;
			    cout << "SD in X = " << cloud_parameters["sdX"] << endl;
			    cout << "SD in Y = " << cloud_parameters["sdY"] << endl;
			    cout << "SD in Z = " << cloud_parameters["sdZ"] << endl;
			    cout << "slendeness in X = " << cloud_parameters["slendernessX"] << endl;
			    cout << "slendeness in Y = " << cloud_parameters["slendernessY"] << endl << endl;
		    }
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
		if (VERBOSE)
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