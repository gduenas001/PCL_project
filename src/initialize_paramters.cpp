
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/common/common.h>


#include "classes_and_structs.h"
#include "initialize_parameters.h"

void initialize_parameters(parameters &P)
{
	P.leafSize=0.05,  // leafSize
  	P.minClusterSize= 10,   // minClusterSize
  	P.clusterTolerance= 0.5,  // clusterTolerance
  	P.limitsXY= 25,   // limitsXY
  	P.limitZlow= -0.7, // limitZlow
  	P.limitZhigh= 1,    // limitZhigh

	P.densityThreshold= 200,  // densityThreshold
	P.slindernessThreshold= 1.0,  // slendernessThreshold
	P.sdXYThreshold= 0.3;   // sdXYThreshold
}
