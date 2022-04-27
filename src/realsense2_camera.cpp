/*=========================================================================
Filename : src/realsense_camera.cpp
Project  : Person 3D Locator
Purpose  : Abstraction class for realsense camera
---------------------------------------------------------------------------
---------------------------------------------------------------------------
Version Number : 1.0
Last Updated   : September 22, 2021
Updated By     : Manodhayan K
---------------------------------------------------------------------------
Copyright (c) 2012 Mobiveil. All rights reserved.
---------------------------------------------------------------------------
This file contains trade secrets of Mobiveil. 
No part may be reproduced or transmitted in any form by any means or for 
any purpose without the express written permission of Mobiveil.
---------------------------------------------------------------------------
Revision History
---------------------------------------------------------------------------


---------------------------------------------------------------------------
Known Issues
---------------------------------------------------------------------------

---------------------------------------------------------------------------
To Do List
---------------------------------------------------------------------------
*/

#include "../include/realsense_camera.h"
#include <numeric>
#include<stdio.h>
#include <ros/ros.h>

#define CAMERA_TO_COLOR_FRAME_X         (-0.00010158783697988838)
#define CAMERA_TO_COLOR_FRAME_Y         (0.014841210097074509)
#define CAMERA_TO_COLOR_FRAME_Z         (-0.00022671300393994898)
#define CAMERA_TO_COLOR_FRAME_QX        (-0.0008337442995980382)
#define CAMERA_TO_COLOR_FRAME_QY        (0.0010442184284329414)
#define CAMERA_TO_COLOR_FRAME_QZ        (0.0009920650627464056)
#define CAMERA_TO_COLOR_FRAME_QW        (0.9999986290931702)
#define DEPTH_IMAGE_FRAME_ID            ("depth_image")     

#define CAMERA_TO_DEPTH_FRAME_X         (0)
#define CAMERA_TO_DEPTH_FRAME_Y         (0)
#define CAMERA_TO_DEPTH_FRAME_Z         (0)
#define CAMERA_TO_DEPTH_FRAME_QX        (0)
#define CAMERA_TO_DEPTH_FRAME_QY        (0)
#define CAMERA_TO_DEPTH_FRAME_QZ        (0)
#define CAMERA_TO_DEPTH_FRAME_QW        (1)


#define COLOR_IMAGE_TOPIC               ("color_image")

#define PERSON_FOLLOWER_STATUS_PARAM            ("/person_3d_locator/status")
#define PERSON_FOLLOWER_STOPPED                 ("STOPPED")


float get_depth_scale(rs2::device dev);

/*============================================================================
Name    :   RealsenseCamera()
------------------------------------------------------------------------------
Purpose :   Constructor for realsense camera to init the camera
Input   :   n/a
Output  :   n/a
Notes   :   n/a
============================================================================*/

RealsenseCamera::RealsenseCamera()
{
    pNodeHandlerCamera = new ros::NodeHandle("~"); 

    config.enable_stream(RS2_STREAM_DEPTH);
    config.enable_stream(RS2_STREAM_COLOR);

    try
    {
    	profile = pipeline.start(config);
    }
    catch(...)
    {
	    std::cout << "Unable to start the pipline" << std::endl;
	    ros::param::set(PERSON_FOLLOWER_STATUS_PARAM, PERSON_FOLLOWER_STOPPED);
    }

    // Set the device to High Accuracy preset
    auto sensor = profile.get_device().first<rs2::depth_sensor>();
    sensor.set_option(RS2_OPTION_VISUAL_PRESET, RS2_RS400_VISUAL_PRESET_HIGH_ACCURACY);
    dec.set_option(RS2_OPTION_FILTER_MAGNITUDE, 2);
    spat.set_option(RS2_OPTION_HOLES_FILL, 5);
    depthScale = get_depth_scale(profile.get_device());

    align_to_color = new rs2::align(RS2_STREAM_COLOR);

    image_transport::ImageTransport imageTransporter(*pNodeHandlerCamera);
    rbgImagePublisher = imageTransporter.advertise(COLOR_IMAGE_TOPIC, 10);
    ROS_INFO("Topic advertised");


    
}

void RealsenseCamera::CaptureFrames()
{

    rs2::disparity_transform disparity2depth(false);

    // Block program until frames arrive
    frameset = pipeline.wait_for_frames();
    frameset = frameset.apply_filter(*align_to_color);
    // frameset = frameset.apply_filter(dec);
    frameset = frameset.apply_filter(depth2disparity);
    frameset = frameset.apply_filter(spat);
    frameset = frameset.apply_filter(temp);
    frameset = frameset.apply_filter(disparity2depth);

    postprocessed_frames.enqueue(frameset);
    bIsFrameChanged = true;
    rgbImage.release();
    rgbImage = frame_to_mat(frameset.get_color_frame());
    
    

    if (rgbImage.empty())
    {
        ROS_INFO("Empty Image");
    }
    else
    {
        imageMessage = cv_bridge::CvImage(std_msgs::Header(), PUBLISH_IMAGE_ENCODING, rgbImage).toImageMsg();
        imageMessage->header.frame_id = COLOR_IMAGE_FRAME_ID;
    }
}

float RealsenseCamera::GetDistance(int iPixelX, int iPixelY)
{
    float fWidth, fHeight, fDepth;
    float averageDepth = false;
    int iNumDepths = false;
    
    rs2::frameset poll_frame;
    postprocessed_frames.poll_for_frame(&poll_frame);

    if (!poll_frame)
    {
        ROS_INFO("post processed frame queue is empty. Using previous frame to fetch distance");
    }
    else
    {
        current_frameset = poll_frame;
    }

    rs2::depth_frame depthFrame = current_frameset.get_depth_frame();

    

    fWidth = depthFrame.get_width();
    fHeight = depthFrame.get_height();


    if (!depthFrame)
    {
        ROS_INFO("Depth frame not available");
        return averageDepth;
    }

    

    
    if ((iPixelX - CENTER_FAR_OFFSET_DEPTH) > 0)
    {

        fDepth = depthFrame.get_distance(iPixelX - CENTER_FAR_OFFSET_DEPTH , iPixelY);
        if (fDepth < CLIPPING_DISTANCE)
        {
            averageDepth += fDepth;
            iNumDepths++;
        }

    }
    if ((iPixelY - CENTER_FAR_OFFSET_DEPTH) > 0)
    {
        fDepth = depthFrame.get_distance(iPixelX, iPixelY - CENTER_FAR_OFFSET_DEPTH);
        if (fDepth < CLIPPING_DISTANCE)
        {
            averageDepth += fDepth;
            iNumDepths++;
        }
    }
    if ((iPixelX + CENTER_FAR_OFFSET_DEPTH) < fWidth)
    {
        fDepth = depthFrame.get_distance(iPixelX + CENTER_FAR_OFFSET_DEPTH , iPixelY);
        if (fDepth < CLIPPING_DISTANCE)
        {
            averageDepth += fDepth;
            iNumDepths++;
        }
    }
    if ((iPixelY + CENTER_FAR_OFFSET_DEPTH) < fHeight)
    {
        fDepth = depthFrame.get_distance(iPixelX, iPixelY + CENTER_FAR_OFFSET_DEPTH);
        if (fDepth < CLIPPING_DISTANCE)
        {
            averageDepth += fDepth;
            iNumDepths++;
        }
    }

    fDepth = depthFrame.get_distance(iPixelX, iPixelY);
    if (fDepth < CLIPPING_DISTANCE)
    {
        averageDepth += fDepth;
        iNumDepths++;
    }
    
    
    averageDepth =  (iNumDepths) ? averageDepth / iNumDepths : iNumDepths;

    return averageDepth;
}

void RealsenseCamera::DepthTo3D(float fPoint[], int iPixelX, int iPixelY, float fDepth)
{
    float fPixel[2] = {(float) iPixelX, (float) iPixelY}; // pixel
    auto intrinsics = pipeline.get_active_profile().get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>().get_intrinsics();

    rs2_deproject_pixel_to_point(fPoint, &intrinsics, fPixel, fDepth);
}

void RealsenseCamera::PublishTransforms(tf2_ros::StaticTransformBroadcaster &tranformBroadcaster)
{
    geometry_msgs::TransformStamped static_transformStamped;

    // Static Transform
    static_transformStamped.header.stamp = ros::Time::now();
    static_transformStamped.header.frame_id = CAMERA_FRAME_ID;

    static_transformStamped.child_frame_id = COLOR_IMAGE_FRAME_ID;
    static_transformStamped.transform.translation.x = CAMERA_TO_COLOR_FRAME_X;
    static_transformStamped.transform.translation.y = CAMERA_TO_COLOR_FRAME_Y;
    static_transformStamped.transform.translation.z = CAMERA_TO_COLOR_FRAME_Z;

    static_transformStamped.transform.rotation.x = CAMERA_TO_COLOR_FRAME_QX;
    static_transformStamped.transform.rotation.y = CAMERA_TO_COLOR_FRAME_QY;
    static_transformStamped.transform.rotation.z = CAMERA_TO_COLOR_FRAME_QZ;
    static_transformStamped.transform.rotation.w = CAMERA_TO_COLOR_FRAME_QW;

    tranformBroadcaster.sendTransform(static_transformStamped);

    static_transformStamped.child_frame_id = DEPTH_IMAGE_FRAME_ID;

    static_transformStamped.transform.translation.x = CAMERA_TO_DEPTH_FRAME_X;
    static_transformStamped.transform.translation.y = CAMERA_TO_DEPTH_FRAME_Y;
    static_transformStamped.transform.translation.z = CAMERA_TO_DEPTH_FRAME_Z;
    
    static_transformStamped.transform.rotation.x = CAMERA_TO_DEPTH_FRAME_QX;
    static_transformStamped.transform.rotation.y = CAMERA_TO_DEPTH_FRAME_QY;
    static_transformStamped.transform.rotation.z = CAMERA_TO_DEPTH_FRAME_QZ;
    static_transformStamped.transform.rotation.w = CAMERA_TO_DEPTH_FRAME_QW;

    tranformBroadcaster.sendTransform(static_transformStamped);
}

void RealsenseCamera::PublishImage()
{
    rbgImagePublisher.publish(imageMessage);

}

/*============================================================================
Name    :  ~RealsenseCamera()
------------------------------------------------------------------------------
Purpose :   Destructor for realsense camera to de-init the camera
Input   :   n/a
Output  :   n/a
Notes   :   n/a
============================================================================*/

RealsenseCamera::~RealsenseCamera()
{
    pipeline.stop();
}


float get_depth_scale(rs2::device dev)
{
    // Go over the device's sensors
    for (rs2::sensor& sensor : dev.query_sensors())
    {
        // Check if the sensor if a depth sensor
        if (rs2::depth_sensor dpt = sensor.as<rs2::depth_sensor>())
        {
            return dpt.get_depth_scale();
        }
    }
    throw std::runtime_error("Device does not have a depth sensor");
}

 
