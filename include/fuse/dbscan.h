#ifndef RM2024_FUSE_DBSCAN_H_
#define RM2024_FUSE_DBSCAN_H_

#include "data_manager/param.h"
#include "data_manager/base.h"
#include <openrm/cudatools.h>
#include <unistd.h>



class point {
public:
	double depth;
    cv::Point3f pos;
	int pointtype;//1 noise 2 border 3 core
	int visited;
	int cluster;
	std::vector<point*>N; //point指针的向量
	point(){
        pointtype = 1;
		visited = 0;
		cluster = 0;
    }
};

cv::Point3f dbscan(std::vector<point>dataset, double eps, int minpts);


#endif