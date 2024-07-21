#include "data_manager/param.h"
#include "data_manager/base.h"
#include "fuse/fuse.h"
#include "fuse/dbscan.h"
#include <openrm/cudatools.h>
#include <unistd.h>
#include <algorithm>


cv::Point3f dbscan(std::vector<point>dataset, double eps, int minpts)
{

	int count = 0;
	int len = dataset.size();
	for (int i = 0; i < len; i++)
	{
		for (int j = 0; j < len; j++)
		{
			if (i == j)continue;

			else if (abs(dataset[i].depth-dataset[j].depth) < eps)
			{
				dataset[i].N.push_back(&dataset[j]);
			}
		}
	}
	for (int i = 0; i < len; i++)
	{
		if (dataset[i].visited == 1) continue;
		dataset[i].visited = 1;
		if (dataset[i].N.size() >= minpts)
		{
			count++;
			dataset[i].pointtype = 3;
			dataset[i].cluster = count;
			for (int j = 0; j < dataset[i].N.size(); j++)
			{
				if (dataset[i].N[j]->visited == 1) continue;//访问dataset[j]的属性
				dataset[i].N[j]->visited = 1;   //dataset[i].N[j]->  就是dataset[j]
				if (dataset[i].N[j]->N.size() > minpts)
				{
					for (int k = 0; k < dataset[i].N[j]->N.size(); k++)
						dataset[i].N.push_back(&(*(dataset[i].N[j]->N[k]))); //传递另一个dataset向量里面的元素内的成员N里面存储其他dataset的地址值  注意得到的是地址
					dataset[i].N[j]->pointtype = 3;
				}
				else dataset[i].N[j]->pointtype = 2;
				if (dataset[i].N[j]->cluster == 0)
					dataset[i].N[j]->cluster = count;
			}
		}
	}
	std::vector<double> cluster_depth(count+1, 0);
	std::vector<int> cluster_count(count+1, 0);
	for(int i = 0; i < len; i++){
		if(dataset[i].pointtype == 3){
			cluster_depth[dataset[i].cluster] += dataset[i].depth;
			cluster_count[dataset[i].cluster]++;
		}
	}
	
	for(int i = 1; i <= count; i++){
		cluster_depth[i] /= cluster_count[i];
	}
	double min_depth;
	int min_cluster;
	for(int i = 1; i <= count; i++){
		if(i == 1 || cluster_depth[i] < min_depth){
			min_depth = cluster_depth[i];
			min_cluster = i;
		}
	}

	cv::Point3f result;
	
	for(int i = 0; i < len; i++){
		if(dataset[i].cluster == min_cluster){
			result.x += dataset[i].pos.x;
			result.y += dataset[i].pos.y;
			result.z += dataset[i].pos.z;
		}
	}
	result.x /= cluster_count[min_cluster];
	result.y /= cluster_count[min_cluster];
	result.z /= cluster_count[min_cluster];

	return result;
}