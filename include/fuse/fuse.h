#ifndef RM2024_FUSE_FUSE_H_
#define RM2024_FUSE_FUSE_H_

#include "data_manager/param.h"
#include "data_manager/base.h"
#include <openrm/cudatools.h>
#include <unistd.h>


cv::Mat PointCloud2Depth(rm::Radar* radar, rm::Camera* camera);

bool extrinsic_calib();


#endif