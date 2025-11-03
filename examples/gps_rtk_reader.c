/**
 * Simple GPS/RTK Data Reader for DJI Matrice 300 RTK with E-Port Kit
 * 
 * This program subscribes to GPS and RTK data topics and displays the information
 * in real-time. Perfect for getting position data from the M300 RTK.
 */

#include <stdio.h>
#include <unistd.h>
#include "dji_fc_subscription.h"
#include "dji_logger.h"
#include "dji_platform.h"

// Callback function for RTK position data
static T_DjiReturnCode RtkPositionCallback(const uint8_t *data, uint16_t dataSize,
                                           const T_DjiDataTimestamp *timestamp)
{
    T_DjiFcSubscriptionRtkPosition *rtkPosition = (T_DjiFcSubscriptionRtkPosition *) data;
    
    printf("\n=== RTK Position Data ===\n");
    printf("Timestamp: %u ms, %u us\n", timestamp->millisecond, timestamp->microsecond);
    printf("Latitude:  %.8f deg\n", rtkPosition->latitude);
    printf("Longitude: %.8f deg\n", rtkPosition->longitude);
    printf("Height (MSL): %.3f m\n", rtkPosition->hfsl);
    printf("========================\n");
    
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

// Callback function for GPS position data
static T_DjiReturnCode GpsPositionCallback(const uint8_t *data, uint16_t dataSize,
                                           const T_DjiDataTimestamp *timestamp)
{
    T_DjiFcSubscriptionGpsPosition *gpsPosition = (T_DjiFcSubscriptionGpsPosition *) data;
    
    printf("\n=== GPS Position Data ===\n");
    printf("Timestamp: %u ms, %u us\n", timestamp->millisecond, timestamp->microsecond);
    printf("Longitude: %.7f deg\n", gpsPosition->x / 10000000.0);  // Convert from deg*10^-7
    printf("Latitude:  %.7f deg\n", gpsPosition->y / 10000000.0);  // Convert from deg*10^-7
    printf("Altitude:  %.3f m\n", gpsPosition->z / 1000.0);        // Convert from mm to m
    printf("========================\n");
    
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

// Callback function for RTK position info (fix status)
static T_DjiReturnCode RtkPositionInfoCallback(const uint8_t *data, uint16_t dataSize,
                                               const T_DjiDataTimestamp *timestamp)
{
    T_DjiFcSubscriptionRtkPositionInfo *rtkInfo = (T_DjiFcSubscriptionRtkPositionInfo *) data;
    
    printf("\n=== RTK Status ===\n");
    printf("Solution Type: ");
    switch (*rtkInfo) {
        case DJI_FC_SUBSCRIPTION_POSITION_SOLUTION_PROPERTY_NOT_AVAILABLE:
            printf("Not Available\n");
            break;
        case DJI_FC_SUBSCRIPTION_POSITION_SOLUTION_PROPERTY_FLOAT_SOLUTION:
            printf("Float Solution\n");
            break;
        case DJI_FC_SUBSCRIPTION_POSITION_SOLUTION_PROPERTY_NARROW_INT:
            printf("Fixed Solution (BEST)\n");
            break;
        default:
            printf("Other (%d)\n", *rtkInfo);
    }
    printf("==================\n");
    
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

int main(int argc, char **argv)
{
    T_DjiReturnCode returnCode;
    
    printf("========================================\n");
    printf("  GPS/RTK Data Reader for M300 RTK\n");
    printf("========================================\n\n");
    
    // Initialize FC subscription module
    printf("Initializing FC Subscription module...\n");
    returnCode = DjiFcSubscription_Init();
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        printf("ERROR: Failed to initialize FC subscription module (code: 0x%08X)\n", returnCode);
        return -1;
    }
    printf("✓ FC Subscription initialized\n\n");
    
    // Subscribe to GPS Position at 5Hz
    printf("Subscribing to GPS Position...\n");
    returnCode = DjiFcSubscription_SubscribeTopic(
        DJI_FC_SUBSCRIPTION_TOPIC_GPS_POSITION,
        DJI_DATA_SUBSCRIPTION_TOPIC_5_HZ,
        GpsPositionCallback
    );
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        printf("ERROR: Failed to subscribe to GPS Position (code: 0x%08X)\n", returnCode);
        goto cleanup;
    }
    printf("✓ GPS Position subscribed\n");
    
    // Subscribe to GPS Details at 5Hz
    printf("Subscribing to GPS Details...\n");
    returnCode = DjiFcSubscription_SubscribeTopic(
        DJI_FC_SUBSCRIPTION_TOPIC_GPS_DETAILS,
        DJI_DATA_SUBSCRIPTION_TOPIC_5_HZ,
        NULL  // We'll poll this data
    );
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        printf("ERROR: Failed to subscribe to GPS Details (code: 0x%08X)\n", returnCode);
        goto cleanup;
    }
    printf("✓ GPS Details subscribed\n");
    
    // Subscribe to RTK Position at 5Hz
    printf("Subscribing to RTK Position...\n");
    returnCode = DjiFcSubscription_SubscribeTopic(
        DJI_FC_SUBSCRIPTION_TOPIC_RTK_POSITION,
        DJI_DATA_SUBSCRIPTION_TOPIC_5_HZ,
        RtkPositionCallback
    );
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        printf("ERROR: Failed to subscribe to RTK Position (code: 0x%08X)\n", returnCode);
        goto cleanup;
    }
    printf("✓ RTK Position subscribed\n");
    
    // Subscribe to RTK Position Info at 5Hz
    printf("Subscribing to RTK Position Info...\n");
    returnCode = DjiFcSubscription_SubscribeTopic(
        DJI_FC_SUBSCRIPTION_TOPIC_RTK_POSITION_INFO,
        DJI_DATA_SUBSCRIPTION_TOPIC_5_HZ,
        RtkPositionInfoCallback
    );
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        printf("ERROR: Failed to subscribe to RTK Position Info (code: 0x%08X)\n", returnCode);
        goto cleanup;
    }
    printf("✓ RTK Position Info subscribed\n");
    
    // Subscribe to RTK Velocity at 5Hz
    printf("Subscribing to RTK Velocity...\n");
    returnCode = DjiFcSubscription_SubscribeTopic(
        DJI_FC_SUBSCRIPTION_TOPIC_RTK_VELOCITY,
        DJI_DATA_SUBSCRIPTION_TOPIC_5_HZ,
        NULL  // We'll poll this data
    );
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        printf("ERROR: Failed to subscribe to RTK Velocity (code: 0x%08X)\n", returnCode);
        goto cleanup;
    }
    printf("✓ RTK Velocity subscribed\n\n");
    
    printf("========================================\n");
    printf("  Data streaming started!\n");
    printf("  Press Ctrl+C to stop\n");
    printf("========================================\n");
    
    // Main loop - poll GPS details and RTK velocity
    while (1) {
        T_DjiFcSubscriptionGpsDetails gpsDetails = {0};
        T_DjiFcSubscriptionRtkVelocity rtkVelocity = {0};
        T_DjiDataTimestamp timestamp = {0};
        
        // Get GPS Details
        returnCode = DjiFcSubscription_GetLatestValueOfTopic(
            DJI_FC_SUBSCRIPTION_TOPIC_GPS_DETAILS,
            (uint8_t *) &gpsDetails,
            sizeof(T_DjiFcSubscriptionGpsDetails),
            &timestamp
        );
        
        if (returnCode == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            printf("\n--- GPS Details ---\n");
            printf("Satellites Used: GPS=%u, GLONASS=%u, Total=%u\n",
                   gpsDetails.gpsSatelliteNumberUsed,
                   gpsDetails.glonassSatelliteNumberUsed,
                   gpsDetails.totalSatelliteNumberUsed);
            printf("HDOP: %.2f, PDOP: %.2f\n", gpsDetails.hdop, gpsDetails.pdop);
            printf("Position Accuracy: H=%.1fmm, V=%.1fmm\n", 
                   gpsDetails.hacc, gpsDetails.vacc);
        }
        
        // Get RTK Velocity
        returnCode = DjiFcSubscription_GetLatestValueOfTopic(
            DJI_FC_SUBSCRIPTION_TOPIC_RTK_VELOCITY,
            (uint8_t *) &rtkVelocity,
            sizeof(T_DjiFcSubscriptionRtkVelocity),
            &timestamp
        );
        
        if (returnCode == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            printf("\n--- RTK Velocity ---\n");
            printf("X: %.3f cm/s, Y: %.3f cm/s, Z: %.3f cm/s\n",
                   rtkVelocity.x, rtkVelocity.y, rtkVelocity.z);
        }
        
        sleep(2);  // Poll every 2 seconds
    }
    
cleanup:
    // Cleanup and unsubscribe
    printf("\n\nCleaning up...\n");
    
    DjiFcSubscription_UnSubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_RTK_VELOCITY);
    DjiFcSubscription_UnSubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_RTK_POSITION_INFO);
    DjiFcSubscription_UnSubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_RTK_POSITION);
    DjiFcSubscription_UnSubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_GPS_DETAILS);
    DjiFcSubscription_UnSubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_GPS_POSITION);
    
    DjiFcSubscription_DeInit();
    
    printf("Done!\n");
    return 0;
}
