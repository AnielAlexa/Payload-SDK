/* 
 * Quick Reference: GPS/RTK Data Structures for M300 RTK
 * ======================================================
 */

// ============================================================================
// GPS TOPICS
// ============================================================================

// GPS Position (DJI_FC_SUBSCRIPTION_TOPIC_GPS_POSITION)
typedef struct {
    int32_t x;  // Longitude in deg*10^-7  (divide by 10000000 to get degrees)
    int32_t y;  // Latitude in deg*10^-7   (divide by 10000000 to get degrees)
    int32_t z;  // Altitude in mm          (divide by 1000 to get meters)
} T_DjiFcSubscriptionGpsPosition;

// Example usage:
// double lon = gpsPosition.x / 10000000.0;
// double lat = gpsPosition.y / 10000000.0;
// double alt = gpsPosition.z / 1000.0;


// GPS Details (DJI_FC_SUBSCRIPTION_TOPIC_GPS_DETAILS)
typedef struct {
    float hdop;                          // Horizontal dilution of precision (0.01 unit)
    float pdop;                          // Position dilution of precision (0.01 unit)
    float fixState;                      // GPS fix state (see E_DjiFcSubscriptionGpsFixState)
    float vacc;                          // Vertical accuracy (mm)
    float hacc;                          // Horizontal accuracy (mm)
    float sacc;                          // Speed accuracy (cm/s)
    uint32_t gpsSatelliteNumberUsed;     // GPS satellites used
    uint32_t glonassSatelliteNumberUsed; // GLONASS satellites used
    uint16_t totalSatelliteNumberUsed;   // Total satellites
    uint16_t gpsCounter;                 // GPS data counter
} T_DjiFcSubscriptionGpsDetails;

// HDOP/PDOP interpretation:
// <1    : Ideal
// 1-2   : Excellent
// 2-5   : Good
// 5-10  : Moderate
// 10-20 : Fair
// >20   : Poor


// GPS Velocity (DJI_FC_SUBSCRIPTION_TOPIC_GPS_VELOCITY)
typedef struct {
    float x;  // Velocity in cm/s (North-ish direction, see note)
    float y;  // Velocity in cm/s (East-ish direction)
    float z;  // Velocity in cm/s (Down, sign flipped from standard NED)
} T_DjiFcSubscriptionGpsVelocity;

// Note: Z-axis is flipped. For true NED, negate the z value.


// ============================================================================
// RTK TOPICS (HIGH PRECISION)
// ============================================================================

// RTK Position (DJI_FC_SUBSCRIPTION_TOPIC_RTK_POSITION)
// Accuracy: ~2cm horizontal, ~3cm vertical when fixed
typedef struct {
    double longitude;  // Longitude in degrees
    double latitude;   // Latitude in degrees
    float hfsl;        // Height above mean sea level in meters
} T_DjiFcSubscriptionRtkPosition;

// This is ready to use! No conversion needed.


// RTK Velocity (DJI_FC_SUBSCRIPTION_TOPIC_RTK_VELOCITY)
typedef struct {
    float x;  // Velocity in cm/s
    float y;  // Velocity in cm/s
    float z;  // Velocity in cm/s
} T_DjiFcSubscriptionRtkVelocity;


// RTK Yaw (DJI_FC_SUBSCRIPTION_TOPIC_RTK_YAW)
typedef int16_t T_DjiFcSubscriptionRtkYaw;  // Yaw in degrees
// Note: This is the vector from ANT1 to ANT2, 90° offset from aircraft yaw


// RTK Position Info (DJI_FC_SUBSCRIPTION_TOPIC_RTK_POSITION_INFO)
typedef uint8_t T_DjiFcSubscriptionRtkPositionInfo;

// Important values:
// 0  = Not available
// 16 = Single point solution (standard GPS)
// 34 = Float solution (RTK working, ~10cm accuracy)
// 50 = Fixed solution (RTK best, ~2cm accuracy) ⭐ TARGET THIS


// RTK Yaw Info (DJI_FC_SUBSCRIPTION_TOPIC_RTK_YAW_INFO)
typedef uint8_t T_DjiFcSubscriptionRtkYawInfo;
// Same status codes as RTK Position Info


// ============================================================================
// SUBSCRIPTION FREQUENCIES
// ============================================================================

typedef enum {
    DJI_DATA_SUBSCRIPTION_TOPIC_1_HZ   = 1,
    DJI_DATA_SUBSCRIPTION_TOPIC_5_HZ   = 5,
    DJI_DATA_SUBSCRIPTION_TOPIC_10_HZ  = 10,
    DJI_DATA_SUBSCRIPTION_TOPIC_50_HZ  = 50,
    DJI_DATA_SUBSCRIPTION_TOPIC_100_HZ = 100,
    DJI_DATA_SUBSCRIPTION_TOPIC_200_HZ = 200,
    DJI_DATA_SUBSCRIPTION_TOPIC_400_HZ = 400,
} E_DjiDataSubscriptionTopicFreq;

// GPS topics support up to 5Hz
// RTK topics support up to 5Hz


// ============================================================================
// TIMESTAMP
// ============================================================================

typedef struct {
    uint32_t millisecond;   // Timestamp in milliseconds
    uint32_t microsecond;   // Additional microsecond precision
} T_DjiDataTimestamp;

// On M300 RTK: Uses flight controller power-on timestamp
// On M30/M30T: Uses payload local timestamp


// ============================================================================
// EXAMPLE: COMPLETE SUBSCRIPTION CODE
// ============================================================================

/*
void setup_gps_rtk_subscription(void) {
    T_DjiReturnCode rc;
    
    // Initialize
    rc = DjiFcSubscription_Init();
    if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        printf("Init failed: 0x%08X\n", rc);
        return;
    }
    
    // Subscribe to RTK position with callback
    rc = DjiFcSubscription_SubscribeTopic(
        DJI_FC_SUBSCRIPTION_TOPIC_RTK_POSITION,
        DJI_DATA_SUBSCRIPTION_TOPIC_5_HZ,
        MyRtkPositionCallback
    );
    
    // Subscribe to RTK status
    rc = DjiFcSubscription_SubscribeTopic(
        DJI_FC_SUBSCRIPTION_TOPIC_RTK_POSITION_INFO,
        DJI_DATA_SUBSCRIPTION_TOPIC_5_HZ,
        MyRtkStatusCallback
    );
    
    // Subscribe to GPS details (for satellite count)
    rc = DjiFcSubscription_SubscribeTopic(
        DJI_FC_SUBSCRIPTION_TOPIC_GPS_DETAILS,
        DJI_DATA_SUBSCRIPTION_TOPIC_5_HZ,
        NULL  // Poll this instead of callback
    );
}

// Callback for RTK position
T_DjiReturnCode MyRtkPositionCallback(const uint8_t *data, uint16_t dataSize,
                                      const T_DjiDataTimestamp *timestamp) {
    T_DjiFcSubscriptionRtkPosition *rtk = (T_DjiFcSubscriptionRtkPosition *)data;
    
    // Log to file or process
    printf("RTK Position: %.8f, %.8f, %.3f\n",
           rtk->latitude, rtk->longitude, rtk->hfsl);
    
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

// Callback for RTK status
T_DjiReturnCode MyRtkStatusCallback(const uint8_t *data, uint16_t dataSize,
                                    const T_DjiDataTimestamp *timestamp) {
    T_DjiFcSubscriptionRtkPositionInfo *status = (T_DjiFcSubscriptionRtkPositionInfo *)data;
    
    if (*status == 50) {
        printf("✓ RTK FIXED - Best accuracy!\n");
    } else if (*status == 34) {
        printf("⚠ RTK FLOAT - Good but not perfect\n");
    } else {
        printf("✗ RTK not available (status: %d)\n", *status);
    }
    
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

// Poll GPS details in main loop
void poll_gps_details(void) {
    T_DjiFcSubscriptionGpsDetails details;
    T_DjiDataTimestamp timestamp;
    
    T_DjiReturnCode rc = DjiFcSubscription_GetLatestValueOfTopic(
        DJI_FC_SUBSCRIPTION_TOPIC_GPS_DETAILS,
        (uint8_t *)&details,
        sizeof(T_DjiFcSubscriptionGpsDetails),
        &timestamp
    );
    
    if (rc == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        printf("Satellites: GPS=%u, GLONASS=%u, Total=%u\n",
               details.gpsSatelliteNumberUsed,
               details.glonassSatelliteNumberUsed,
               details.totalSatelliteNumberUsed);
        printf("Accuracy: H=%.1fmm, V=%.1fmm\n",
               details.hacc, details.vacc);
    }
}
*/


// ============================================================================
// COORDINATE CONVERSION HELPERS
// ============================================================================

/*
// WGS84 to ECEF (Earth-Centered, Earth-Fixed)
void wgs84_to_ecef(double lat, double lon, double alt, 
                   double *x, double *y, double *z) {
    const double a = 6378137.0;           // WGS84 semi-major axis
    const double e2 = 0.00669437999014;   // WGS84 first eccentricity squared
    
    double lat_rad = lat * M_PI / 180.0;
    double lon_rad = lon * M_PI / 180.0;
    
    double N = a / sqrt(1 - e2 * sin(lat_rad) * sin(lat_rad));
    
    *x = (N + alt) * cos(lat_rad) * cos(lon_rad);
    *y = (N + alt) * cos(lat_rad) * sin(lon_rad);
    *z = (N * (1 - e2) + alt) * sin(lat_rad);
}

// Calculate distance between two WGS84 points (Haversine formula)
double distance_between_wgs84(double lat1, double lon1, 
                              double lat2, double lon2) {
    const double R = 6371000.0;  // Earth radius in meters
    
    double lat1_rad = lat1 * M_PI / 180.0;
    double lat2_rad = lat2 * M_PI / 180.0;
    double dlat = (lat2 - lat1) * M_PI / 180.0;
    double dlon = (lon2 - lon1) * M_PI / 180.0;
    
    double a = sin(dlat/2) * sin(dlat/2) +
               cos(lat1_rad) * cos(lat2_rad) *
               sin(dlon/2) * sin(dlon/2);
    double c = 2 * atan2(sqrt(a), sqrt(1-a));
    
    return R * c;  // Distance in meters
}
*/


// ============================================================================
// IMPORTANT NOTES
// ============================================================================

/*
1. RTK REQUIREMENTS FOR M300:
   - RTK base station must be set up and connected
   - Clear sky view (minimal obstructions)
   - Antennas properly placed on aircraft
   - RTK subscription active in DJI Pilot

2. COORDINATE FRAMES:
   - GPS/RTK: WGS84 geodetic (lat/lon/alt)
   - Velocity: NED (North-East-Down) with Z flipped
   - Attitude: Quaternion to NED frame

3. ACCURACY EXPECTATIONS:
   - GPS only: 3-5m horizontal, 5-10m vertical
   - RTK Float: ~10cm horizontal, ~15cm vertical
   - RTK Fixed: ~2cm horizontal, ~3cm vertical ⭐

4. SUBSCRIPTION LIMITS:
   - Max 4 different frequencies per session
   - Max 242 bytes per frequency group
   - Topics must be unsubscribed in reverse order

5. E-PORT CONNECTION:
   - USB recommended for development
   - UART for production (more reliable)
   - Check baud rate matches (921600 default)
   - Ensure proper power supply
*/
