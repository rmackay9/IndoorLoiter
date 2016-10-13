#include <Pozyx.h>
#include <Pozyx_definitions.h>

#include <mavlink.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <Time.h> // download from https://github.com/PaulStoffregen/Time

////////////////// Pozyx Prams //////////////////////////////

uint8_t num_anchors = 4;                                    // the number of anchors
// the network id of the anchors: change these to the network ids of your anchors.
uint16_t anchors[4] = { 0x6003, // (0,0)
                        0x6013, // x-axis
                        0x6029, // y-axis
                        0x6045};     
int32_t heights[4] = {0, 0, 0, 0};                          // anchor z-coordinates in mm
boolean bProcessing = false;                                // set this to true to output data for the processing sketch         

// only required for manual anchor calibration. 
// Please change this to the coordinates measured for the anchors
int32_t anchors_x[4] = {0, 10000, 1000, 9000};              // anchor x-coorindates in mm
int32_t anchors_y[4] = {0, 0, 7000, 8000};                  // anchor y-coordinates in mm

////////////////// MAVLINK Prams //////////////////////////////

uint8_t mav_system_id = 100;
uint8_t mav_component_id = 200;

// RX TX serial for flight controller ex) Pixhawk
// https://github.com/PaulStoffregen/AltSoftSerial

uint8_t mavlink_enable = 1; // for debug
uint8_t rxPin = 10;
uint8_t txPin = 11;

SoftwareSerial fcboardSerial(rxPin, txPin); 

////////////////////////////////////////////////

void setup()
{
  Serial.begin(9600);
  fcboardSerial.begin(9600);
  
  if (Pozyx.begin() == POZYX_FAILURE) {
    Serial.println(F("ERROR: Unable to connect to POZYX shield"));
    Serial.println(F("Reset required"));
    delay(100);
    abort();
  }
  
  Serial.println(F("----------POZYX POSITIONING V1.0----------"));
  Serial.println(F("NOTES:"));
  Serial.println(F("- No parameters required."));
  Serial.println();
  Serial.println(F("- System will auto start calibration"));
  Serial.println();
  Serial.println(F("- System will auto start positioning"));
  Serial.println(F("----------POZYX POSITIONING V1.0----------"));
  Serial.println();
  Serial.println(F("Performing auto anchor calibration:"));

  // clear all previous devices in the device list
  Pozyx.clearDevices();
     
  //int status = Pozyx.doAnchorCalibration(POZYX_2_5D, 10, num_anchors, anchors, heights);
  int status = Pozyx.doAnchorCalibration(POZYX_2D, 10, num_anchors, anchors, heights);
  if (status != POZYX_SUCCESS) {
    Serial.println(status);
    Serial.println(F("ERROR: calibration"));
    Serial.println(F("Reset required"));
    delay(100);
    abort();
  }
  
  // if the automatic anchor calibration is unsuccessful, try manually setting the anchor coordinates.
  // fot this, you must update the arrays anchors_x, anchors_y and heights above
  // comment out the doAnchorCalibration block and the if-statement above if you are using manual mode
  //SetAnchorsManual();

  printCalibrationResult();
  delay(3000);

  Serial.println(F("Starting positioning: "));
}

void loop()
{  
  coordinates_t position;
  
  //int status = Pozyx.doPositioning(&position, POZYX_2_5D, 1000);
  int status = Pozyx.doPositioning(&position);
  
  if (status == POZYX_SUCCESS) {
    // print out the result
    if (!bProcessing) {
      printCoordinates(position);
    } else {    
      printCoordinatesProcessing(position);
    }

    // send GPS MAVLINK message
    if (mavlink_enable) {
      SendGPSMAVLinkMessage(position);
    }
  }
}

// function to print the coordinates to the serial monitor
void printCoordinates(coordinates_t coor)
{  
  Serial.print("x_mm: ");
  Serial.print(coor.x);
  Serial.print("\t");
  Serial.print("y_mm: ");
  Serial.print(coor.y);
  Serial.print("\t");
  Serial.print("z_mm: ");
  Serial.print(coor.z);
  Serial.println(); 
}

// function to print out positoining data + ranges for the processing sketch
void printCoordinatesProcessing(coordinates_t coor)
{
  // get the network id and print it
  uint16_t network_id;
  Pozyx.getNetworkId(&network_id);
  
  Serial.print("POS,0x");
  Serial.print(network_id,HEX);
  Serial.print(",");
  Serial.print(coor.x);
  Serial.print(",");
  Serial.print(coor.y);
  Serial.print(",");
  Serial.print(coor.z);
  Serial.print(",");
  
  // get information about the positioning error and print it
  pos_error_t pos_error;
  Pozyx.getPositionError(&pos_error);
    
  Serial.print(pos_error.x);
  Serial.print(",");
  Serial.print(pos_error.y);
  Serial.print(",");
  Serial.print(pos_error.z);
  Serial.print(",");
  Serial.print(pos_error.xy);
  Serial.print(",");
  Serial.print(pos_error.xz);
  Serial.print(",");
  Serial.print(pos_error.yz); 
  
  // read out the ranges to each anchor and print it 
  for (int i=0; i<num_anchors; i++) {
    device_range_t range;
    Pozyx.getDeviceRangeInfo(anchors[i], &range);
    Serial.print(",");
    Serial.print(range.distance);  
    Serial.print(",");
    Serial.print(range.RSS); 
  }
  Serial.println();
}

// print out the anchor coordinates (also required for the processing sketch)
void printCalibrationResult()
{
  uint8_t list_size;
  int status;

  status = Pozyx.getDeviceListSize(&list_size);
  Serial.print("list size: ");
  Serial.println(status*list_size);
  
  if (list_size == 0) {
    Serial.println("Calibration failed.");
    Serial.println(Pozyx.getSystemError());
    return;
  }
  
  uint16_t device_ids[list_size];
  status &= Pozyx.getDeviceIds(device_ids,list_size);
  
  Serial.println(F("Calibration result:"));
  Serial.print(F("Anchors found: "));
  Serial.println(list_size);
  
  coordinates_t anchor_coor;
  
  for (int i=0; i<list_size; i++) {
    Serial.print("ANCHOR,");
    Serial.print("0x");
    Serial.print(device_ids[i], HEX);
    Serial.print(",");    
    status = Pozyx.getDeviceCoordinates(device_ids[i], &anchor_coor);
    Serial.print(anchor_coor.x);
    Serial.print(",");
    Serial.print(anchor_coor.y);
    Serial.print(",");
    Serial.println(anchor_coor.z);
    
  }    
}

// function to manually set the anchor coordinates
void SetAnchorsManual()
{
  int i=0;
  
  for (i=0; i<num_anchors; i++) {
   device_coordinates_t anchor;
   anchor.network_id = anchors[i];
   anchor.flag = 0x1; 
   anchor.pos.x = anchors_x[i];
   anchor.pos.y = anchors_y[i];
   anchor.pos.z = heights[i];
   Pozyx.addDevice(anchor);
 }
}

// GPS MAVLink message using Pozyx potision
void SendGPSMAVLinkMessage(coordinates_t position)
{
  // Initialize the required buffers 
  mavlink_message_t msg; 
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  
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
  uint64_t time_usec = micros();
  uint8_t gps_id = 0;
  uint16_t ignore_flags = GPS_INPUT_IGNORE_FLAG_HDOP|
                          GPS_INPUT_IGNORE_FLAG_VDOP|
                          GPS_INPUT_IGNORE_FLAG_VEL_HORIZ|
                          GPS_INPUT_IGNORE_FLAG_VEL_VERT|
                          GPS_INPUT_IGNORE_FLAG_SPEED_ACCURACY|
                          GPS_INPUT_IGNORE_FLAG_HORIZONTAL_ACCURACY|
                          GPS_INPUT_IGNORE_FLAG_VERTICAL_ACCURACY;
  uint32_t time_week_ms = 0;
  uint16_t time_week = 0;
  uint8_t fix_type = 3;                 // GPS_OK_FIX_3D;
  int32_t latitude = 35.692941*1.0e7;   // sample
  int32_t longitude = 139.776500*1.0e7; // sample
  int32_t altitude = 10;                // unit m
  float hdop = 0.0f;
  float vdop = 0.0f;
  float vn = 0.0f;
  float ve = 0.0f;
  float vd = 0.0f;
  float speed_accuracy = 0.0f;
  float horiz_accuracy = 0.0f;
  float vert_accuracy = 0.0f;
  uint8_t satellites_visible = 10;      // fake

  make_gps_time(time_week_ms, time_week);
  
  uint16_t len = mavlink_msg_gps_input_pack(
                    mav_system_id,
                    mav_component_id,
                    &msg,
                    time_usec,
                    gps_id,
                    ignore_flags,
                    time_week_ms, time_week,
                    fix_type,
                    latitude, longitude, altitude,
                    hdop, vdop,
                    vn, ve, vd,
                    speed_accuracy, horiz_accuracy, vert_accuracy,
                    satellites_visible);
                    
  // Copy the message to send buffer 
  len = mavlink_msg_to_send_buffer(buf, &msg);
  
  // Send message
  fcboardSerial.write(buf, len);

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

