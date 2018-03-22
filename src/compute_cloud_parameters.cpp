#include <pcl/point_types.h>
#include <pcl/common/common.h>

#include <map>
#include <cmath>        // std::pow
#include "compute_cloud_parameters.h"

#include <pcl/filters/statistical_outlier_removal.h>

using namespace std;

void compute_cloud_parameters(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, 
							  map<string, double>& cloud_parameters)
{

    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud (cloud);
    sor.setMeanK (50);
    sor.setStddevMulThresh (1);
    sor.filter (*cloud);

    // Compute parameter for the cluster
    double meanX= 0, meanY= 0, meanZ= 0;
    for (int i=0, end= cloud->points.size(); i < end; i++) // find type of object and find inbuilt function!!
    {
		meanX= meanX + double(cloud->points[i].x);
		meanY= meanY + double(cloud->points[i].y);
		meanZ= meanZ + double(cloud->points[i].z);
    }
    meanX= meanX / double(cloud->points.size());
    meanY= meanY / double(cloud->points.size());
    meanZ= meanZ / double(cloud->points.size());
    double reject_cluster;
    double horizontal_distance=sqrt(pow(meanX,2)+pow(meanY,2));
    if (horizontal_distance < 3)
        reject_cluster=1;
    else
        reject_cluster=0;


    double varX= 0, varY= 0, varZ=0;
    for (int i=0, end= cloud->points.size(); i < end; i++)
    {
		varX= varX + pow( double(cloud->points[i].x) - meanX, 2 );
		varY= varY + pow( double(cloud->points[i].y) - meanY, 2 );
		varZ= varZ + pow( double(cloud->points[i].z) - meanZ, 2 );
  	}
    varX= varX / double( cloud->points.size() - 1 ); cloud_parameters["sdX"]= sqrt(varX);
    varY= varY / double( cloud->points.size() - 1 ); cloud_parameters["sdY"]= sqrt(varY);
    varZ= varZ / double( cloud->points.size() - 1 ); cloud_parameters["sdZ"]= sqrt(varZ);
    cloud_parameters["sdXYZ"]= cloud_parameters["sdX"] * cloud_parameters["sdY"] * cloud_parameters["sdZ"];

    // Parameters of the cluster
    cloud_parameters["slendernessX"]= cloud_parameters["sdZ"] / cloud_parameters["sdX"];
    cloud_parameters["slendernessY"]= cloud_parameters["sdZ"] / cloud_parameters["sdY"];
    cloud_parameters["density"]= double( cloud->points.size() ) / (27*cloud_parameters["sdXYZ"]);
    cloud_parameters["very_close"]= reject_cluster;

}