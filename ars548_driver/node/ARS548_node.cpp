#include <pcl/PCLPointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <cmath>
#include <cstdint>

#include "ARS548.hpp"
#include "ros/publisher.h"
#include "ros/ros.h"
#include "ros/time.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/PointField.h"

// #include "sensor_msgs/PointCloud.h"
Driver::ARS548 ars548{};
bool timeInit = false;
ros::Time firstReceiveTime;
ros::Time firstTimeStamp;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ARS548_Driver");
    ros::NodeHandle nodeHandle("~");
    ros::Publisher pointCloudPub =
        nodeHandle.advertise<sensor_msgs::PointCloud2>("/radar_raw_message",
                                                       10);
    sensor_msgs::PointField fieldAzimuth;
    fieldAzimuth.name = "azimuth";
    fieldAzimuth.count = 1;
    fieldAzimuth.offset = 0;
    fieldAzimuth.datatype = sensor_msgs::PointField::FLOAT32;
    sensor_msgs::PointField fieldAzimuthSTD;
    fieldAzimuthSTD.name = "azimuthSTD";
    fieldAzimuthSTD.count = 1;
    fieldAzimuthSTD.offset = 4;
    fieldAzimuthSTD.datatype = sensor_msgs::PointField::FLOAT32;
    sensor_msgs::PointField fieldElevation;
    fieldElevation.name = "elevation";
    fieldElevation.count = 1;
    fieldElevation.offset = 8;
    fieldElevation.datatype = sensor_msgs::PointField::FLOAT32;
    sensor_msgs::PointField fieldElevationSTD;
    fieldElevationSTD.name = "elevationSTD";
    fieldElevationSTD.count = 1;
    fieldElevationSTD.offset = 12;
    fieldElevationSTD.datatype = sensor_msgs::PointField::FLOAT32;
    sensor_msgs::PointField fieldRange;
    fieldRange.name = "range";
    fieldRange.count = 1;
    fieldRange.offset = 16;
    fieldRange.datatype = sensor_msgs::PointField::FLOAT32;
    sensor_msgs::PointField fieldRangeSTD;
    fieldRangeSTD.name = "rangeSTD";
    fieldRangeSTD.count = 1;
    fieldRangeSTD.offset = 20;
    fieldRangeSTD.datatype = sensor_msgs::PointField::FLOAT32;
    sensor_msgs::PointField fieldVelocity;
    fieldVelocity.name = "velocity";
    fieldVelocity.count = 1;
    fieldVelocity.offset = 24;
    fieldVelocity.datatype = sensor_msgs::PointField::FLOAT32;
    sensor_msgs::PointField fieldVelocitySTD;
    fieldVelocitySTD.name = "velocitySTD";
    fieldVelocitySTD.count = 1;
    fieldVelocitySTD.offset = 28;
    fieldVelocitySTD.datatype = sensor_msgs::PointField::FLOAT32;
    sensor_msgs::PointField fieldRCS;
    fieldRCS.name = "rcs";
    fieldRCS.count = 1;
    fieldRCS.offset = 32;
    fieldRCS.datatype = sensor_msgs::PointField::INT8;
    sensor_msgs::PointCloud2::_fields_type fields;
    fields.emplace_back(fieldAzimuth);
    fields.emplace_back(fieldAzimuthSTD);
    fields.emplace_back(fieldElevation);
    fields.emplace_back(fieldElevationSTD);
    fields.emplace_back(fieldRange);
    fields.emplace_back(fieldRangeSTD);
    fields.emplace_back(fieldVelocity);
    fields.emplace_back(fieldVelocitySTD);
    fields.emplace_back(fieldRCS);

    ars548.init("eno1", "224.0.2.2", 42102, "10.13.1.113", 42401, 42101);
    while (ros::ok())
    {
        ros::Time ars548TimeStamp;
        ars548.receiveMsg();
        if (ars548.getDetectionListReady())
        {
            auto &detectionsArray = ars548.getDetectionList();
            if (!timeInit)
            {
                firstReceiveTime = ros::Time::now();
                firstTimeStamp.sec = detectionsArray.timestampS;
                firstTimeStamp.nsec = detectionsArray.timestampNs;
                timeInit = true;
            }
            ars548TimeStamp.sec = detectionsArray.timestampS;
            ars548TimeStamp.nsec = detectionsArray.timestampNs;

            sensor_msgs::PointCloud2 pubFrame;
            pubFrame.header.frame_id = "world";
            pubFrame.header.stamp =
                firstReceiveTime + (ars548TimeStamp - firstTimeStamp);
            pubFrame.fields = fields;
            pubFrame.is_bigendian =
                sensor_msgs::PointCloud2::_is_bigendian_type(false);
            pubFrame.is_dense = sensor_msgs::PointCloud2::_is_dense_type(true);
            pubFrame.point_step = 33;

            for (int index = 0; index < detectionsArray.listNumOfDetections;
                 index++)
            {
                auto &detection = detectionsArray.detectionArray[index];
                if (detection.invalidFlags == 0)
                {
                    uint32_t byteArray = 0;
                    byteArray =
                        *reinterpret_cast<uint32_t *>(&detection.azimuthAngle);
                    pubFrame.data.emplace_back(
                        static_cast<uint8_t>(byteArray & 0xFF));
                    pubFrame.data.emplace_back(
                        static_cast<uint8_t>((byteArray >> 8) & 0xFF));
                    pubFrame.data.emplace_back(
                        static_cast<uint8_t>((byteArray >> 16) & 0xFF));
                    pubFrame.data.emplace_back(
                        static_cast<uint8_t>((byteArray >> 24) & 0xFF));
                    byteArray =
                        *reinterpret_cast<uint32_t *>(&detection.azimuthSTD);
                    pubFrame.data.emplace_back(
                        static_cast<uint8_t>(byteArray & 0xFF));
                    pubFrame.data.emplace_back(
                        static_cast<uint8_t>((byteArray >> 8) & 0xFF));
                    pubFrame.data.emplace_back(
                        static_cast<uint8_t>((byteArray >> 16) & 0xFF));
                    pubFrame.data.emplace_back(
                        static_cast<uint8_t>((byteArray >> 24) & 0xFF));
                    byteArray = *reinterpret_cast<uint32_t *>(
                        &detection.elevationAngle);
                    pubFrame.data.emplace_back(
                        static_cast<uint8_t>(byteArray & 0xFF));
                    pubFrame.data.emplace_back(
                        static_cast<uint8_t>((byteArray >> 8) & 0xFF));
                    pubFrame.data.emplace_back(
                        static_cast<uint8_t>((byteArray >> 16) & 0xFF));
                    pubFrame.data.emplace_back(
                        static_cast<uint8_t>((byteArray >> 24) & 0xFF));
                    byteArray =
                        *reinterpret_cast<uint32_t *>(&detection.elevationSTD);
                    pubFrame.data.emplace_back(
                        static_cast<uint8_t>(byteArray & 0xFF));
                    pubFrame.data.emplace_back(
                        static_cast<uint8_t>((byteArray >> 8) & 0xFF));
                    pubFrame.data.emplace_back(
                        static_cast<uint8_t>((byteArray >> 16) & 0xFF));
                    pubFrame.data.emplace_back(
                        static_cast<uint8_t>((byteArray >> 24) & 0xFF));
                    byteArray = *reinterpret_cast<uint32_t *>(&detection.range);
                    pubFrame.data.emplace_back(
                        static_cast<uint8_t>(byteArray & 0xFF));
                    pubFrame.data.emplace_back(
                        static_cast<uint8_t>((byteArray >> 8) & 0xFF));
                    pubFrame.data.emplace_back(
                        static_cast<uint8_t>((byteArray >> 16) & 0xFF));
                    pubFrame.data.emplace_back(
                        static_cast<uint8_t>((byteArray >> 24) & 0xFF));
                    byteArray =
                        *reinterpret_cast<uint32_t *>(&detection.rangeSTD);
                    pubFrame.data.emplace_back(
                        static_cast<uint8_t>(byteArray & 0xFF));
                    pubFrame.data.emplace_back(
                        static_cast<uint8_t>((byteArray >> 8) & 0xFF));
                    pubFrame.data.emplace_back(
                        static_cast<uint8_t>((byteArray >> 16) & 0xFF));
                    pubFrame.data.emplace_back(
                        static_cast<uint8_t>((byteArray >> 24) & 0xFF));
                    byteArray =
                        *reinterpret_cast<uint32_t *>(&detection.rangeRate);
                    pubFrame.data.emplace_back(
                        static_cast<uint8_t>(byteArray & 0xFF));
                    pubFrame.data.emplace_back(
                        static_cast<uint8_t>((byteArray >> 8) & 0xFF));
                    pubFrame.data.emplace_back(
                        static_cast<uint8_t>((byteArray >> 16) & 0xFF));
                    pubFrame.data.emplace_back(
                        static_cast<uint8_t>((byteArray >> 24) & 0xFF));
                    byteArray =
                        *reinterpret_cast<uint32_t *>(&detection.rangeRateSTD);
                    pubFrame.data.emplace_back(
                        static_cast<uint8_t>(byteArray & 0xFF));
                    pubFrame.data.emplace_back(
                        static_cast<uint8_t>((byteArray >> 8) & 0xFF));
                    pubFrame.data.emplace_back(
                        static_cast<uint8_t>((byteArray >> 16) & 0xFF));
                    pubFrame.data.emplace_back(
                        static_cast<uint8_t>((byteArray >> 24) & 0xFF));
                    pubFrame.data.emplace_back(detection.rcs);
                    pubFrame.row_step++;
                }
            }
            pointCloudPub.publish(pubFrame);
        }
        // if (ars548.getObjectListReady())
        // {
        //     //
        // }
        // if (ars548.getRadarStatusReady())
        // {
        //     //
        // }
        ars548.clearReadyFlag();
    }
    return 0;
}