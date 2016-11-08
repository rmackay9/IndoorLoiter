#include <Pozyx.h>
#include <Pozyx_definitions.h>

#define _SS_MAX_RX_BUFF 32

#include <SoftwareSerial.h>
#include <Wire.h>

////////////////// MAVLINK Prams //////////////////////////////
#include <Time.h>
#include <TimeLib.h>
#include "C:\Users\rmackay9\Documents\GitHub\ardupilot\Build.ArduCopter\libraries\GCS_MAVLink\include\mavlink\v2.0\common\mavlink.h"
#define LATITUDE_BASE (36.324187 * 1.0e7)
#define LONGITUDE_BASE (138.639212 * 1.0e7)
#define LOCATION_SCALING_FACTOR_INV_MM 0.08983204953368922f
#define DEG_TO_RAD      (M_PI / 180.0f)
uint8_t buf[MAVLINK_MSG_ID_GPS_INPUT_LEN + 13];
uint8_t buf_len;
int32_t latitude = 0;
int32_t longitude = 0;

////////////////// Pozyx Prams //////////////////////////////
#define CONFIG_TX_GAIN 33.5f

#define NUM_ANCHORS 4
// the network id of the anchors: change these to the network ids of your anchors.
uint16_t anchor_id[4] = { 0x601C, // (0,0)
                          0x6020, // x-axis
                          0x6057, // y-axis
                          0x605E};

// only required for manual anchor calibration. 
// Please change this to the coordinates measured for the anchors
int32_t anchors_x[NUM_ANCHORS];    // anchor x-coorindates in mm (horizontal)
int32_t anchors_y[NUM_ANCHORS];    // anchor y-coordinates in mm (vertical)
int32_t heights[NUM_ANCHORS];        // anchor z-coordinates in mm

// RX TX serial for flight controller ex) Pixhawk
// https://github.com/PaulStoffregen/AltSoftSerial
SoftwareSerial fcboardSerial(10, 11); // rx, tx

////////////////////////////////////////////////

void setup()
{
    Serial.begin(115200);
    fcboardSerial.begin(115200);

    if (Pozyx.begin() == POZYX_FAILURE) {
        Serial.println(("ERR: shield"));
        delay(100);
        abort();
    }

    // clear all previous devices in the device list
    Pozyx.clearDevices();

    // configure beacons
    while (!configure_beacons()) {
        delay(1000);
    }

    // if the automatic anchor calibration is unsuccessful, try manually setting the anchor coordinates.
    // fot this, you must update the arrays anchors_x, anchors_y and heights above
    // comment out the doAnchorCalibration block and the if-statement above if you are using manual mode
    SetAnchorsManual();

    print_anchor_coordinates();
    delay(5000);
}

void loop()
{
    // send GPS_INPUT to ardupilot
    get_and_send_position();
}

void print_tab()
{  
    Serial.print("\t");
}

// set a tag or anchor's gain
//   set tag_id to zero to set local device's gain
//   returns true on success
bool set_device_gain(uint16_t dev_id, float gain)
{
    float tx_power = -1;

    // get/set transmit power of tag
    bool gain_ok = false;
    uint8_t retry = 0;
    while (!gain_ok && retry < 5) {
        if (Pozyx.getTxPower(&tx_power, dev_id) == POZYX_SUCCESS) {
            if (tx_power != gain) {
                Pozyx.setTxPower(CONFIG_TX_GAIN, dev_id);
            } else {
                gain_ok = true;
            }
        }
        retry++;
    }

    // display final gain
    Serial.print(dev_id, HEX);
    Serial.print(":");
    if (tx_power > 0) {
        Serial.print(tx_power);
    } else {
        Serial.print("?");
    }
    Serial.println("dB");

    return gain_ok;
}

// performs repeated calls to get reliable distance between devices
bool get_remote_range(uint16_t dev1, uint16_t dev2, int32_t& range)
{
    // set distances between tags
    uint32_t range_tot = 0;
    uint16_t count = 0;
    device_range_t dev_range;
    for (uint8_t i=0; i <= 10; i++) {
        // origin to 1st
        if (Pozyx.doRemoteRanging(dev1, dev2, &dev_range) == POZYX_SUCCESS) {
            range_tot += dev_range.distance;
            count++;
        }
        if (Pozyx.doRemoteRanging(dev2, dev1, &dev_range) == POZYX_SUCCESS) {
            range_tot += dev_range.distance;
            count++;
        }
    }
    // success if at least 5 successful ranges were retrieved
    if (count > 5) {
        range = range_tot / count;
        return true;
    }
    return false;
}

void set_beacon_position(uint8_t index, int32_t x_mm, int32_t y_mm, int32_t z_mm)
{
    anchors_x[index] = x_mm;
    anchors_y[index] = y_mm;
    heights[index] = z_mm;
}

// configure beacons
bool configure_beacons()
{
    bool configured_ok = true;

    // get/set transmit power of tag
    if (!set_device_gain(0, CONFIG_TX_GAIN)) {
        configured_ok = false;
    }

    // set transmit power of beacons    
    for (uint8_t i=0; i < NUM_ANCHORS; i++) {
        if (!set_device_gain(anchor_id[i], CONFIG_TX_GAIN)) {
            configured_ok = false;
        }
    }

    // set distances between tags
    int32_t x_range = 0, y_range = 0;
    // origin to x-axis (i.e. bottom right)
    if (get_remote_range(anchor_id[0], anchor_id[1], x_range)) {
        set_beacon_position(1, x_range, 0, 0);
    } else {
        configured_ok = false;
    }
    // origin to y-axis (i.e. top left)
    if (get_remote_range(anchor_id[0], anchor_id[2], y_range)) {
        set_beacon_position(2, 0, y_range, 0);
    } else {
        configured_ok = false;
    }
    // top right
    if (x_range != 0 && y_range != 0) {
        set_beacon_position(3, x_range, y_range, 0);
    } else {
        configured_ok = false;
    }
    
    Serial.print("Config ");
    if (configured_ok) {
        Serial.println("ok");
    } else {
        Serial.println("fail!");
    }

    return configured_ok;
}

// function to manually set the anchor coordinates
void SetAnchorsManual()
{
    for (uint8_t i=0; i<NUM_ANCHORS; i++) {
        device_coordinates_t anchor;
        anchor.network_id = anchor_id[i];
        anchor.flag = 0x1; 
        anchor.pos.x = anchors_x[i];
        anchor.pos.y = anchors_y[i];
        anchor.pos.z = heights[i];
        Pozyx.addDevice(anchor);
    }
}

// print coordinates to the serial monitor
void print_coordinates(coordinates_t coor, pos_error_t pos_error)
{  
    Serial.print("x:");
    Serial.print(coor.x);
    print_tab();
    Serial.print("y:");
    Serial.print(coor.y);
    Serial.print(" "); 
}

// print out the anchor coordinates
void print_anchor_coordinates()
{
  uint8_t list_size;
  int status;

  status = Pozyx.getDeviceListSize(&list_size);

  // print error if no anchors are setup
  if (list_size == 0) {
    Serial.print("No Anchors: ");
    Serial.println(Pozyx.getSystemError());
    return;
  }

  // retrieve anchor information
  uint16_t device_ids[list_size];
  status &= Pozyx.getDeviceIds(device_ids,list_size);

  Serial.print("Anchors: ");
  Serial.println(list_size);

  coordinates_t anchor_coor;
  
  for (int i=0; i<list_size; i++) {
    Serial.print(device_ids[i], HEX);
    print_tab();
    status = Pozyx.getDeviceCoordinates(device_ids[i], &anchor_coor);
    Serial.print(anchor_coor.x);
    print_tab();
    Serial.print(anchor_coor.y);
    print_tab();
    Serial.println(anchor_coor.z);
  }    
}

/////////// MAVLINK GPS_INPUT related functions ///////////////

float longitude_scale(uint32_t lat)
{
    static int32_t last_lat = 0;
    static float scale = 1.0;
    if (labs(last_lat - lat) < 100000) {
        // we are within 0.01 degrees (about 1km) of the
        // previous latitude. We can avoid the cos() and return
        // the same scale factor.
        return scale;
    }
    scale = cosf(lat * 1.0e-7f * DEG_TO_RAD);
    if (scale < 0.01f) {
      scale = 0.01f;
    }
    if (scale > 1.0f) {
      scale = 1.0f;
    }
    last_lat = lat;
    return scale;
}

void location_offset(int32_t &lat, int32_t &lng, int32_t offset_north_mm, int32_t offset_east_mm)
{
    int32_t dlat = offset_north_mm * LOCATION_SCALING_FACTOR_INV_MM;
    int32_t dlng = (offset_east_mm * LOCATION_SCALING_FACTOR_INV_MM) / longitude_scale(lat);
    lat += dlat;
    lng += dlng;
}

// calculate GPS time
// based ardupilot/libraries/AP_GPS/GPS_Backend.cpp
void make_gps_time(uint32_t &time_week_ms, uint16_t &time_week)
{
    uint8_t year, mon, day, hour, min, sec;
    uint16_t msec;
    time_t now_time = now();

    year = ::year(now_time);
    mon  = ::month(now_time);
    day  = ::day(now_time);

    uint32_t v = millis();
    msec = v % 1000; v /= 1000;
    sec  = v % 100; v /= 100;
    min  = v % 100; v /= 100;
    hour = v % 100; v /= 100;

    int8_t rmon = mon - 2;
    if (0 >= rmon) {    
        rmon += 12;
        year -= 1;
    }

    // get time in seconds since unix epoch
    uint32_t ret = (year/4) - 15 + 367*rmon/12 + day;
    ret += year*365 + 10501;
    ret = ret*24 + hour;
    ret = ret*60 + min;
    ret = ret*60 + sec;

    // convert to time since GPS epoch
    ret -= 272764785UL;

    // get GPS week and time
    time_week = ret / (7*86400UL);
    time_week_ms = (ret % (7*86400UL)) * 1000;
    time_week_ms += msec;
}

// get position and send in MAVLink GPS_INPUT message
void get_and_send_position()
{
    coordinates_t position;

    // get position
    if (get_position(position)) {
        create_gps_input_mavlink_message(position);
        send_gps_input_mavlink_message();
    }
}

bool get_position(coordinates_t& position)
{
    pos_error_t pos_error;

    // get vehicle position
    //if (Pozyx.doPositioning(&position, POZYX_2_5D, 0) == POZYX_SUCCESS) {
    if (Pozyx.doPositioning(&position, POZYX_3D, 0, 0x00) == POZYX_SUCCESS) {
        if (Pozyx.getPositionError(&pos_error) == POZYX_SUCCESS) {
            // display position
            print_coordinates(position, pos_error);
            return true;
        } else {
            Serial.println("fail");
        }
    }

    return false;
}

// send GPS_INPUT mavlink message to ardupilot
void create_gps_input_mavlink_message(coordinates_t& position)
{
    // Initialize the required buffers
    mavlink_message_t msg;

  /**
 * @brief Pack a gps_input message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_usec Timestamp (micros since boot or Unix epoch)
 * @param gps_id ID of the GPS for multiple GPS inputs
 * @param ignore_flags Flags indicating which fields to ignore (see GPS_INPUT_IGNORE_FLAGS enum).  All other fields must be provided.
 * @param time_week_ms GPS time (milliseconds from start of GPS week)
 * @param time_week GPS week number
 * @param fix_type 0-1: no fix, 2: 2D fix, 3: 3D fix. 4: 3D with DGPS. 5: 3D with RTK
 * @param lat Latitude (WGS84), in degrees * 1E7
 * @param lon Longitude (WGS84), in degrees * 1E7
 * @param alt Altitude (AMSL, not WGS84), in m (positive for up)
 * @param hdop GPS HDOP horizontal dilution of position in m
 * @param vdop GPS VDOP vertical dilution of position in m
 * @param vn GPS velocity in m/s in NORTH direction in earth-fixed NED frame
 * @param ve GPS velocity in m/s in EAST direction in earth-fixed NED frame
 * @param vd GPS velocity in m/s in DOWN direction in earth-fixed NED frame
 * @param speed_accuracy GPS speed accuracy in m/s
 * @param horiz_accuracy GPS horizontal accuracy in m
 * @param vert_accuracy GPS vertical accuracy in m
 * @param satellites_visible Number of satellites visible.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 
    uint16_t ignore_flags = GPS_INPUT_IGNORE_FLAG_VEL_HORIZ|
                            GPS_INPUT_IGNORE_FLAG_VEL_VERT|
                            GPS_INPUT_IGNORE_FLAG_SPEED_ACCURACY|
                            GPS_INPUT_IGNORE_FLAG_HORIZONTAL_ACCURACY|
                            GPS_INPUT_IGNORE_FLAG_VERTICAL_ACCURACY;
    uint32_t time_week_ms = 0;
    uint16_t time_week = 0;
    
    make_gps_time(time_week_ms, time_week);
    
    // adjust position
    latitude = LATITUDE_BASE;
    longitude = LONGITUDE_BASE;
    location_offset(latitude, longitude, position.y, position.x);
    
    uint16_t len = mavlink_msg_gps_input_pack(
                      1,
                      0,
                      &msg,
                      micros(),       // time_usec,
                      0,              // gps_id,
                      ignore_flags,
                      time_week_ms,   // time_week_ms,
                      time_week,      // time_week,
                      3,              // fix_type,
                      latitude,       // latitude,
                      longitude,      // longitude,
                      10,             // altitude,
                      1.0f,           // hdop,
                      1.0f,           // vdop,
                      0.0f,           // vn
                      0.0f,           // ve
                      0.0f,           // vd
                      0.0f,           // speed_accuracy
                      0.0f,           // horiz_accuracy
                      0.0f,           // vert_accuracy,
                      14              // satellites_visible
                      );
    
    // Copy the message to send buffer 
    buf_len = mavlink_msg_to_send_buffer(buf, &msg);
    Serial.print(latitude);
    Serial.print(",");
    Serial.println(longitude);
}

void send_gps_input_mavlink_message()
{
    // Send message
    fcboardSerial.write(buf, buf_len);
}
