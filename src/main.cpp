/******************************************************************************
 * Copyright (c) 2014 Roman Dissauer
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 ******************************************************************************/

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <bta.h>


typedef pcl::PointCloud<pcl::PointXYZ>		FrameXYZ;
typedef pcl::PointCloud<pcl::PointXYZI>		FrameXYZAmp;

static void BTA_CALLCONV infoEvent(BTA_EventId eventId, int8_t *msg) {
    ROS_INFO("   BTA infoEvent (%d) %s\n", eventId, msg);
}


/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
    /**
     * You must call one of the versions of ros::init() before using any other
     * part of the ROS system.
     */
    ros::init(argc, argv, "blt_tof");

    /**
     * NodeHandle is the main access point to communications with the ROS system.
     * The first NodeHandle constructed will fully initialize this node, and the last
     * NodeHandle destructed will close down the node.
     */
    ros::NodeHandle n;
    
    BTA_Config config;
    BTAinitConfig(&config);
    config.frameMode = BTA_FrameModeXYZ;
    config.infoEvent = &infoEvent;
    config.verbosity = 5;

    
    BTA_Status status;
    BTA_Handle btaHandle;
    status = BTAopen(&config, &btaHandle);
    if (status != BTA_StatusOk) {
    	ROS_ERROR("Couldn't open Sensor!");
        return -1;
    }
    
    BTA_DeviceInfo *deviceInfo;
    ROS_INFO("BTAgetDeviceInfo()");
    status = BTAgetDeviceInfo(btaHandle, &deviceInfo);
    if (status != BTA_StatusOk) {
    	ROS_ERROR("Couldn't get device info!");
        return -1;
    }
    ROS_INFO("Device type: 0x%x", deviceInfo->deviceType);
    ROS_INFO("BTAfreeDeviceInfo()");
    BTAfreeDeviceInfo(deviceInfo);
    
    ROS_INFO("Service running: %d", BTAisRunning(btaHandle));
    ROS_INFO("Connection up: %d", BTAisConnected(btaHandle));

    /**
     * The advertise() function is how you tell ROS that you want to
     * publish on a given topic name. This invokes a call to the ROS
     * master node, which keeps a registry of who is publishing and who
     * is subscribing. After this advertise() call is made, the master
     * node will notify anyone who is trying to subscribe to this topic name,
     * and they will in turn negotiate a peer-to-peer connection with this
     * node.    advertise() returns a Publisher object which allows you to
     * publish messages on that topic through a call to publish().  Once
     * all copies of the returned Publisher object are destroyed, the topic
     * will be automatically unadvertised.
     *
     * The second parameter to advertise() is the size of the message queue
     * used for publishing messages.    If messages are published more quickly
     * than we can send them, the number here specifies how many messages to
     * buffer up before throwing some away.
     */
    ros::Publisher pub_point_cloud;
    pub_point_cloud = n.advertise<FrameXYZ>("point_cloud", 1);

    ros::Rate loop_rate(10);

    /**
     * A count of how many messages we have sent. This is used to create
     * a unique string for each message.
     */
    int count = 0;
    while (ros::ok())
    {
    	bool cart = false;
    	bool amp = false;
    
		float *xCoordinates, *yCoordinates, *zCoordinates;
		//uint16_t *amplitudes;
		BTA_DataFormat dataFormat;
		BTA_Unit unit;
		uint16_t xRes, yRes;
		
		BTA_Frame *frame;
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
			/*
			ROS_INFO("BTAgetAmplitudes");
			status = BTAgetAmplitudes(frame, (void**)&amplitudes, &dataFormat, &unit, &xRes, &yRes);
			ROS_INFO("now check...");
			ROS_INFO_STREAM("status: " << status);
			if (status == BTA_StatusOk)
			{
				ROS_INFO("BTAgetAmplitudes ok");
				if (dataFormat == BTA_DataFormatUInt16)
				{
					ROS_INFO("data format ok");
					if (unit == BTA_UnitUnitLess)
					{
						ROS_INFO("unit ok");
						ROS_INFO("got amplitudes...");
						amp = true;
					}
				}
			}
			*/
			if (cart)
			{
				FrameXYZ::Ptr msg (new FrameXYZ);
				msg->header.frame_id = "blt_tof";
		
				for (int i = 0; i < xRes*yRes; i++)
				{
					pcl::PointXYZ tmp_point;
					tmp_point.x = (float)xCoordinates[i];
					tmp_point.y = (float)yCoordinates[i];
					tmp_point.z = (float)zCoordinates[i];
					//tmp_point.intensity = (float)amplitudes[i];
			
					msg->points.push_back(tmp_point);
				}
		
				msg->height = 1;
				msg->width = xRes * yRes;
		
				/**
				 * The publish() function is how you send messages. The parameter
				 * is the message object. The type of this object must agree with the type
				 * given as a template parameter to the advertise<>() call, as was done
				 * in the constructor above.
				 */
				pub_point_cloud.publish(msg);
				cart = amp = false;
			}
		
    	}
    	
    	BTAfreeFrame(&frame);

        ros::spinOnce();

        loop_rate.sleep();
        ++count;
    }
    
    BTAclose(&btaHandle);
    
    return 0;
}
