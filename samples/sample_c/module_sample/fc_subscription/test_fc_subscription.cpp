/**
 ********************************************************************
 * @file    test_fc_subscription.cpp
 * @brief   ROS2-enabled FC subscription with GPS/RTK publishers
 *
 * @copyright (c) 2021 DJI. All rights reserved.
 *
 * All information contained herein is, and remains, the property of DJI.
 * The intellectual and technical concepts contained herein are proprietary
 * to DJI and may be covered by U.S. and foreign patents, patents in process,
 * and protected by trade secret or copyright law.  Dissemination of this
 * information, including but not limited to data and other proprietary
 * material(s) incorporated within the information, in any form, is strictly
 * prohibited without the express written consent of DJI.
 *
 * If you receive this source code without DJI's authorization, you may not
 * further disseminate the information, and you must immediately remove the
 * source code and notify DJI of its removal. DJI reserves the right to pursue
 * legal actions against you for any loss(es) or damage(s) caused by your
 * failure to do so.
 *
 *********************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include <utils/util_misc.h>
#include <math.h>
#include "test_fc_subscription.h"
#include "dji_logger.h"
#include "dji_platform.h"
#include "widget_interaction_test/test_widget_interaction.h"

// ROS2 includes
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_msgs/msg/u_int8.hpp>

/* Private constants ---------------------------------------------------------*/
#define FC_SUBSCRIPTION_TASK_FREQ         (1)
#define FC_SUBSCRIPTION_TASK_STACK_SIZE   (2048)

/* Private types -------------------------------------------------------------*/

/* Private functions declaration ---------------------------------------------*/
static void *UserFcSubscription_Task(void *arg);
static T_DjiReturnCode DjiTest_FcSubscriptionReceiveQuaternionCallback(const uint8_t *data, uint16_t dataSize,
                                                                       const T_DjiDataTimestamp *timestamp);

/* Private variables ---------------------------------------------------------*/
static T_DjiTaskHandle s_userFcSubscriptionThread;
static bool s_userFcSubscriptionDataShow = true;
static uint8_t s_totalSatelliteNumberUsed = 0;
static uint32_t s_userFcSubscriptionDataCnt = 0;
static uint8_t s_rtkFixStatus = 0;

// ROS2 global variables
static rclcpp::Node::SharedPtr g_ros_node = nullptr;
static rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr g_gps_pub = nullptr;
static rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr g_rtk_pub = nullptr;
// static rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr g_rtk_status_pub = nullptr;

/* Exported functions definition ---------------------------------------------*/

extern "C" T_DjiReturnCode DjiTest_FcSubscriptionStartService(void)
{
    T_DjiReturnCode djiStat;
    T_DjiOsalHandler *osalHandler = NULL;

    // Initialize ROS2 node if not already initialized
    if (!rclcpp::ok()) {
        USER_LOG_ERROR("ROS2 not initialized! Call rclcpp::init() before starting FC subscription.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    // Create ROS2 node
    if (g_ros_node == nullptr) {
        g_ros_node = rclcpp::Node::make_shared("dji_gps_rtk_node");

        // Create publishers
        g_gps_pub = g_ros_node->create_publisher<sensor_msgs::msg::NavSatFix>("/m300/gps/fix", 10);
        g_rtk_pub = g_ros_node->create_publisher<sensor_msgs::msg::NavSatFix>("/m300/rtk/fix", 10);
       // g_rtk_status_pub = g_ros_node->create_publisher<std_msgs::msg::UInt8>("/m300/rtk/status", 10);

        USER_LOG_INFO("ROS2 node created with GPS/RTK publishers:");
        USER_LOG_INFO("  - /m300/gps/fix (sensor_msgs/NavSatFix)");
        USER_LOG_INFO("  - /m300/rtk/fix (sensor_msgs/NavSatFix)");
        // USER_LOG_INFO("  - /m300/rtk/status (std_msgs/UInt8)");
    }

    osalHandler = DjiPlatform_GetOsalHandler();
    djiStat = DjiFcSubscription_Init();
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("init data subscription module error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    djiStat = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_QUATERNION, DJI_DATA_SUBSCRIPTION_TOPIC_50_HZ,
                                               DjiTest_FcSubscriptionReceiveQuaternionCallback);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe topic quaternion error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    } else {
        USER_LOG_DEBUG("Subscribe topic quaternion success.");
    }

    djiStat = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_VELOCITY, DJI_DATA_SUBSCRIPTION_TOPIC_1_HZ,
                                               NULL);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe topic velocity error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    } else {
        USER_LOG_DEBUG("Subscribe topic velocity success.");
    }

    djiStat = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_GPS_POSITION, DJI_DATA_SUBSCRIPTION_TOPIC_1_HZ,
                                               NULL);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe topic gps position error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    } else {
        USER_LOG_DEBUG("Subscribe topic gps position success.");
    }

    djiStat = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_GPS_DETAILS, DJI_DATA_SUBSCRIPTION_TOPIC_1_HZ,
                                               NULL);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe topic gps details error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    } else {
        USER_LOG_DEBUG("Subscribe topic gps details success.");
    }

    // Subscribe to RTK Position for high-precision positioning
    djiStat = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_RTK_POSITION, DJI_DATA_SUBSCRIPTION_TOPIC_1_HZ,
                                               NULL);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe topic rtk position error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    } else {
        USER_LOG_DEBUG("Subscribe topic rtk position success.");
    }

    // Subscribe to RTK Position Info for fix status
    djiStat = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_RTK_POSITION_INFO, DJI_DATA_SUBSCRIPTION_TOPIC_1_HZ,
                                               NULL);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe topic rtk position info error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    } else {
        USER_LOG_DEBUG("Subscribe topic rtk position info success.");
    }

    // Subscribe to RTK Velocity for high-precision velocity
    djiStat = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_RTK_VELOCITY, DJI_DATA_SUBSCRIPTION_TOPIC_1_HZ,
                                               NULL);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe topic rtk velocity error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    } else {
        USER_LOG_DEBUG("Subscribe topic rtk velocity success.");
    }

    if (osalHandler->TaskCreate("user_subscription_task", UserFcSubscription_Task,
                                FC_SUBSCRIPTION_TASK_STACK_SIZE, NULL, &s_userFcSubscriptionThread) !=
        DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("user data subscription task create error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

extern "C" T_DjiReturnCode DjiTest_FcSubscriptionRunSample(void)
{
    // This function is not used in our ROS2 implementation
    // All functionality is handled by DjiTest_FcSubscriptionStartService
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

extern "C" T_DjiReturnCode DjiTest_FcSubscriptionDataShowTrigger(void)
{
    s_userFcSubscriptionDataShow = !s_userFcSubscriptionDataShow;
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

extern "C" T_DjiReturnCode DjiTest_FcSubscriptionGetTotalSatelliteNumber(uint8_t *number)
{
    *number = s_totalSatelliteNumberUsed;
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

/* Private functions definition-----------------------------------------------*/
#ifndef __CC_ARM
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmissing-noreturn"
#pragma GCC diagnostic ignored "-Wreturn-type"
#endif

static void *UserFcSubscription_Task(void *arg)
{
    T_DjiReturnCode djiStat;
    T_DjiFcSubscriptionVelocity velocity = {0};
    T_DjiDataTimestamp timestamp = {0};
    T_DjiFcSubscriptionGpsPosition gpsPosition = {0};
    T_DjiFcSubscriptionGpsDetails gpsDetails = {0};
    T_DjiFcSubscriptionRtkPosition rtkPosition = {0};
    T_DjiFcSubscriptionRtkPositionInfo rtkPositionInfo = 0;
    T_DjiFcSubscriptionRtkVelocity rtkVelocity = {0};
    T_DjiOsalHandler *osalHandler = NULL;
    const char *rtkStatusStr;

    USER_UTIL_UNUSED(arg);
    osalHandler = DjiPlatform_GetOsalHandler();

    USER_LOG_INFO("========================================");
    USER_LOG_INFO("  GPS/RTK ROS2 PUBLISHER STARTED!");
    USER_LOG_INFO("  Publishing to ROS2 topics at 1 Hz");
    USER_LOG_INFO("========================================");

    while (1) {
        osalHandler->TaskSleepMs(1000 / FC_SUBSCRIPTION_TASK_FREQ);

        // Spin ROS2 node to process callbacks
        if (g_ros_node != nullptr) {
            rclcpp::spin_some(g_ros_node);
        }

        // Get RTK Position (high precision)
        djiStat = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_RTK_POSITION,
                                                          (uint8_t *) &rtkPosition,
                                                          sizeof(T_DjiFcSubscriptionRtkPosition),
                                                          &timestamp);
        bool rtkValid = (djiStat == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);

        // Get RTK Position Info (fix status)
        djiStat = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_RTK_POSITION_INFO,
                                                          (uint8_t *) &rtkPositionInfo,
                                                          sizeof(T_DjiFcSubscriptionRtkPositionInfo),
                                                          &timestamp);
        if (djiStat == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            s_rtkFixStatus = rtkPositionInfo;
        }

        // Get RTK Velocity (high precision)
        djiStat = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_RTK_VELOCITY,
                                                          (uint8_t *) &rtkVelocity,
                                                          sizeof(T_DjiFcSubscriptionRtkVelocity),
                                                          &timestamp);

        // Get standard GPS Position
        djiStat = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_GPS_POSITION,
                                                          (uint8_t *) &gpsPosition,
                                                          sizeof(T_DjiFcSubscriptionGpsPosition),
                                                          &timestamp);
        bool gpsValid = (djiStat == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);

        // Get GPS Details
        djiStat = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_GPS_DETAILS,
                                                          (uint8_t *) &gpsDetails,
                                                          sizeof(T_DjiFcSubscriptionGpsDetails),
                                                          &timestamp);
        if (djiStat == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            s_totalSatelliteNumberUsed = gpsDetails.totalSatelliteNumberUsed;
        }

        // Get standard Velocity
        djiStat = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_VELOCITY,
                                                          (uint8_t *) &velocity,
                                                          sizeof(T_DjiFcSubscriptionVelocity),
                                                          &timestamp);

        // Publish GPS data to ROS2
        if (gpsValid && g_gps_pub != nullptr) {
            sensor_msgs::msg::NavSatFix gps_msg;
            gps_msg.header.stamp = g_ros_node->now();
            gps_msg.header.frame_id = "gps";

            // Convert GPS position (DJI format: deg * 10^-7)
            gps_msg.latitude = gpsPosition.y / 10000000.0;
            gps_msg.longitude = gpsPosition.x / 10000000.0;
            gps_msg.altitude = gpsPosition.z / 1000.0;  // mm to meters

            // Set status based on fix
            gps_msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
            gps_msg.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;

            // Set covariance from GPS details (convert mm to m)
            double h_cov = (gpsDetails.hacc / 1000.0) * (gpsDetails.hacc / 1000.0);
            double v_cov = (gpsDetails.vacc / 1000.0) * (gpsDetails.vacc / 1000.0);
            gps_msg.position_covariance[0] = h_cov;  // East
            gps_msg.position_covariance[4] = h_cov;  // North
            gps_msg.position_covariance[8] = v_cov;  // Up
            gps_msg.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;

            g_gps_pub->publish(gps_msg);
        }

        // Publish RTK data to ROS2
        if (rtkValid && g_rtk_pub != nullptr) {
            sensor_msgs::msg::NavSatFix rtk_msg;
            rtk_msg.header.stamp = g_ros_node->now();
            rtk_msg.header.frame_id = "rtk";

            // RTK position is already in degrees
            rtk_msg.latitude = rtkPosition.latitude;
            rtk_msg.longitude = rtkPosition.longitude;
            rtk_msg.altitude = rtkPosition.hfsl;

            // Set status based on RTK fix type
            if (rtkPositionInfo == DJI_FC_SUBSCRIPTION_POSITION_SOLUTION_PROPERTY_NARROW_INT) {
                rtk_msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX;  // RTK fixed
            } else if (rtkPositionInfo == DJI_FC_SUBSCRIPTION_POSITION_SOLUTION_PROPERTY_FLOAT_SOLUTION) {
                rtk_msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_SBAS_FIX;  // RTK float
            } else {
                rtk_msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
            }
            rtk_msg.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;

            // RTK has much better accuracy - set conservative estimates
            double rtk_cov = 0.02 * 0.02;  // 2cm standard RTK accuracy
            if (rtkPositionInfo == DJI_FC_SUBSCRIPTION_POSITION_SOLUTION_PROPERTY_FLOAT_SOLUTION) {
                rtk_cov = 0.10 * 0.10;  // 10cm for float solution
            }
            rtk_msg.position_covariance[0] = rtk_cov;
            rtk_msg.position_covariance[4] = rtk_cov;
            rtk_msg.position_covariance[8] = rtk_cov * 2;  // Vertical usually worse
            rtk_msg.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;

            g_rtk_pub->publish(rtk_msg);
        }

        // Publish RTK status
        // if (g_rtk_status_pub != nullptr) {
        //     std_msgs::msg::UInt8 status_msg;
        //     status_msg.data = s_rtkFixStatus;
        //     g_rtk_status_pub->publish(status_msg);
        // }

        // Console logging
        if (s_userFcSubscriptionDataShow == true) {
            // Display RTK fix status
            switch (rtkPositionInfo) {
                case DJI_FC_SUBSCRIPTION_POSITION_SOLUTION_PROPERTY_NOT_AVAILABLE:
                    rtkStatusStr = "NOT AVAILABLE";
                    break;
                case DJI_FC_SUBSCRIPTION_POSITION_SOLUTION_PROPERTY_FLOAT_SOLUTION:
                    rtkStatusStr = "FLOAT SOLUTION (accuracy ~10cm)";
                    break;
                case DJI_FC_SUBSCRIPTION_POSITION_SOLUTION_PROPERTY_NARROW_INT:
                    rtkStatusStr = "FIXED SOLUTION (accuracy ~2cm)";
                    break;
                default:
                    rtkStatusStr = "UNKNOWN";
                    break;
            }

            // USER_LOG_INFO("\n========== POSITION & NAVIGATION DATA ==========");
            // USER_LOG_INFO("\n>>> RTK STATUS: %s", rtkStatusStr);
            // USER_LOG_INFO("\n=== RTK POSITION (HIGH PRECISION) ===");
            // USER_LOG_INFO("Latitude:  %.8f°", rtkPosition.latitude);
            // USER_LOG_INFO("Longitude: %.8f°", rtkPosition.longitude);
            // USER_LOG_INFO("Height MSL: %.3f m", rtkPosition.hfsl);

            // USER_LOG_INFO("\n=== GPS POSITION (Standard) ===");
            // USER_LOG_INFO("Latitude:  %.7f°", gpsPosition.y / 10000000.0);
            // USER_LOG_INFO("Longitude: %.7f°", gpsPosition.x / 10000000.0);
            // USER_LOG_INFO("Altitude:  %.3f m", gpsPosition.z / 1000.0);

            // USER_LOG_INFO("\n=== SATELLITE & ACCURACY INFO ===");
            // USER_LOG_INFO("Satellites: GPS=%d, GLONASS=%d, Total=%d",
            //               gpsDetails.gpsSatelliteNumberUsed,
            //               gpsDetails.glonassSatelliteNumberUsed,
            //               gpsDetails.totalSatelliteNumberUsed);
            // USER_LOG_INFO("Horizontal Accuracy: %.1f mm", gpsDetails.hacc);
            // USER_LOG_INFO("Vertical Accuracy:   %.1f mm", gpsDetails.vacc);

            // USER_LOG_INFO("\n=== ROS2 PUBLISHING ===");
            // USER_LOG_INFO("GPS topic:    /m300/gps/fix");
            // USER_LOG_INFO("RTK topic:    /m300/rtk/fix");
            // USER_LOG_INFO("Status topic: /m300/rtk/status");
            // USER_LOG_INFO("\n================================================\n");
        }
    }
}

#ifndef __CC_ARM
#pragma GCC diagnostic pop
#endif

static T_DjiReturnCode DjiTest_FcSubscriptionReceiveQuaternionCallback(const uint8_t *data, uint16_t dataSize,
                                                                       const T_DjiDataTimestamp *timestamp)
{
    T_DjiFcSubscriptionQuaternion *quaternion = (T_DjiFcSubscriptionQuaternion *) data;
    dji_f64_t pitch, yaw, roll;

    USER_UTIL_UNUSED(dataSize);

    pitch = (dji_f64_t) asinf(-2 * quaternion->q1 * quaternion->q3 + 2 * quaternion->q0 * quaternion->q2) * 57.3;
    roll = (dji_f64_t) atan2f(2 * quaternion->q2 * quaternion->q3 + 2 * quaternion->q0 * quaternion->q1,
                             -2 * quaternion->q1 * quaternion->q1 - 2 * quaternion->q2 * quaternion->q2 + 1) * 57.3;
    yaw = (dji_f64_t) atan2f(2 * quaternion->q1 * quaternion->q2 + 2 * quaternion->q0 * quaternion->q3,
                             -2 * quaternion->q2 * quaternion->q2 - 2 * quaternion->q3 * quaternion->q3 + 1) *
          57.3;

    if (s_userFcSubscriptionDataShow == true) {
        if (s_userFcSubscriptionDataCnt++ % DJI_DATA_SUBSCRIPTION_TOPIC_50_HZ == 0) {
            USER_LOG_DEBUG("euler angles: pitch = %.2f roll = %.2f yaw = %.2f.", pitch, roll, yaw);
            DjiTest_WidgetLogAppend("pitch = %.2f roll = %.2f yaw = %.2f.", pitch, roll, yaw);
        }
    }

    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

/****************** (C) COPYRIGHT DJI Innovations *****END OF FILE****/
