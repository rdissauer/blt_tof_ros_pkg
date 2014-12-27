#include <blt_tof.h>


static void BTA_CALLCONV infoEvent(BTA_EventId eventId, int8_t *msg)
{
    ROS_INFO("   BTA infoEvent (%d) %s\n", eventId, msg);
}

void blt_tof_connect(BTA_Handle *btaHandle)
{
    BTA_Config config;
    BTA_Status status;
    
    BTAinitConfig(&config);
    config.frameMode = BTA_FrameModeXYZAmp;
    config.infoEvent = &infoEvent;
    config.verbosity = 5;
    
    status = BTAopen(&config, btaHandle);
    if (status != BTA_StatusOk)
    {
    	ROS_ERROR("Couldn't open Sensor!");
        return;
    }
    
    BTA_DeviceInfo *deviceInfo;
    ROS_INFO("BTAgetDeviceInfo()");
    status = BTAgetDeviceInfo(*btaHandle, &deviceInfo);
    if (status != BTA_StatusOk)
    {
    	ROS_ERROR("Couldn't get device info!");
        return;
    }
    ROS_INFO("Device type: 0x%x", deviceInfo->deviceType);
    ROS_INFO("BTAfreeDeviceInfo()");
    BTAfreeDeviceInfo(deviceInfo);
    
    ROS_INFO("Service running: %d", BTAisRunning(*btaHandle));
    ROS_INFO("Connection up: %d", BTAisConnected(*btaHandle));
}


void blt_tof_get_frame(BTA_Handle btaHandle, FrameXYZAmp::Ptr msg)
{
    bool cart = false;
    bool amp = false;

    float *xCoordinates, *yCoordinates, *zCoordinates;
    float *amplitudes;
    BTA_DataFormat dataFormat;
    BTA_Unit unit;
    uint16_t xRes, yRes;
    BTA_Frame *frame;
    BTA_Status status;
    
    status = BTAgetFrame(btaHandle, &frame, 3000);
    if (status != BTA_StatusOk) {
        ROS_ERROR("FAILED TO GET FRAME");
    }
    else
    {
        status = BTAgetXYZcoordinates(frame, (void**)&xCoordinates, (void**)&yCoordinates, (void**)&zCoordinates, &dataFormat, &unit, &xRes, &yRes);
        if (status == BTA_StatusOk)
        {
            if (dataFormat == BTA_DataFormatFloat32)
            {
                if (unit == BTA_UnitMeter)
                {
                    cart = true;
                }
            }
        }
        
        status = BTAgetAmplitudes(frame, (void**)&amplitudes, &dataFormat, &unit, &xRes, &yRes);
        if (status == BTA_StatusOk)
        {
            //ROS_INFO("Got Amplitudes");
            //ROS_INFO_STREAM("dataFormat: " << dataFormat << "; unit: " << unit);
            if (dataFormat == BTA_DataFormatFloat32)
            {
                //ROS_INFO("dataFormat right");
                if (unit == BTA_UnitUnitLess)
                {
                    //ROS_INFO("unit right");
                    amp = true;
                }
            }
        }
        
        
        if (cart & amp)
        {
            ROS_INFO("Create Cloud...");
            for (int i = 0; i < xRes*yRes; i++)
            {
                pcl::PointXYZI tmp_point;
                tmp_point.x = (float)xCoordinates[i];
                tmp_point.y = (float)yCoordinates[i];
                tmp_point.z = (float)zCoordinates[i];
                tmp_point.intensity = (float)amplitudes[i];
        
                msg->points.push_back(tmp_point);
            }
    
            msg->header.frame_id = "blt_tof";
            msg->height = 1;
            msg->width = xRes * yRes;
    
            cart = false;
            amp = false;
        }
    
    }
    
    BTAfreeFrame(&frame);
}


