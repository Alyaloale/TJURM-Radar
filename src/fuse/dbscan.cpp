#include "data_manager/param.h"
#include "data_manager/base.h"
#include "fuse/fuse.h"
#include "fuse/dbscan.h"
#include <openrm/cudatools.h>
#include <unistd.h>
#include <algorithm>


cv::Point3f dbscan(std::vector<point>dataset)
{

	int len = dataset.size();

	cv::Point3f result;

	for(int i = 0; i < len; i++){

		result.x += dataset[i].pos.x;
		result.y += dataset[i].pos.y;
		result.z += dataset[i].pos.z;

	}
	result.x /= len;
	result.y /= len;
	result.z /= len;

	return result;
}

