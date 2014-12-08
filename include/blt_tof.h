#ifndef BLT_TOF_H_INCLUDED
#define BLT_TOF_H_INCLUDED

#include <pcl_ros/point_cloud.h>
#include <bta.h>


#ifdef __cplusplus
extern "C"
{
#endif

typedef pcl::PointCloud<pcl::PointXYZ>		FrameXYZ;
typedef pcl::PointCloud<pcl::PointXYZI>		FrameXYZAmp;

static void BTA_CALLCONV infoEvent(BTA_EventId eventId, int8_t *msg);
void blt_tof_connect(BTA_Handle *btaHandle);
void blt_tof_get_frame(BTA_Handle btaHandle, FrameXYZ::Ptr msg);

#ifdef __cplusplus
}
#endif

#endif
