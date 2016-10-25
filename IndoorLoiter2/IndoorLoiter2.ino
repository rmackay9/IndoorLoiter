#include <Pozyx.h>
#include <Pozyx_definitions.h>

#include <SoftwareSerial.h>
#include <Wire.h>

////////////////// Pozyx Prams //////////////////////////////

#define NUM_ANCHORS 4
// the network id of the anchors: change these to the network ids of your anchors.
uint16_t anchor_id[4] = { 0x601C, // (0,0)
                          0x6020, // x-axis
                          0x6057, // y-axis
                          0x605E};     

// only required for manual anchor calibration. 
// Please change this to the coordinates measured for the anchors
int32_t anchors_x[NUM_ANCHORS] = {0,    18600, 0,     18600};    // anchor x-coorindates in mm (horizontal)
int32_t anchors_y[NUM_ANCHORS] = {0,    0,     10000, 10000};    // anchor y-coordinates in mm (vertical)
int32_t heights[NUM_ANCHORS] =   {1420, 0,     0,     1450};     // anchor z-coordinates in mm

// RX TX serial for flight controller ex) Pixhawk
// https://github.com/PaulStoffregen/AltSoftSerial
SoftwareSerial fcboardSerial(10, 11); // rx, tx

#define MSG_HEADER          0x01
#define MSGID_BEACON_CONFIG 0x02
#define MSGID_BEACON_DIST   0x03
#define MSGID_POSITION      0x04

// structure for messages uploaded to ardupilot
union beacon_config_msg {
    struct {
        uint8_t beacon_id;
        uint8_t beacon_count;
        int32_t x;
        int32_t y;
        int32_t z;
    } info;
    uint8_t buf[14];
};
union beacon_distance_msg {
    struct {
        uint8_t beacon_id;
        uint32_t distance;
    } info;
    uint8_t buf[5];
};
union vehicle_position_msg {
    struct {
        int32_t x;
        int32_t y;
        int32_t z;
        int16_t position_error;
    } info;
    uint8_t buf[14];
};
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

    Serial.println(("V1.0"));

    // clear all previous devices in the device list
    Pozyx.clearDevices();

    // if the automatic anchor calibration is unsuccessful, try manually setting the anchor coordinates.
    // fot this, you must update the arrays anchors_x, anchors_y and heights above
    // comment out the doAnchorCalibration block and the if-statement above if you are using manual mode
    SetAnchorsManual();

    print_anchor_coordinates();

    Serial.println(("Waiting.."));
    delay(5000);

    Serial.println(("Starting: "));
}

void loop()
{
    static uint8_t counter = 0;
    counter++;
    if (counter >= 6) {
        counter = 0;
        send_beacon_config();
    }
    //get_ranges();
    get_position();
    //send_beacon_config();
    delay(5000);
}

void print_comma()
{  
    Serial.print(",");
}

void print_tab()
{  
    Serial.print("\t");
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
    print_tab();
    Serial.print("z:");
    Serial.print(coor.z);
    Serial.print(" err x:");
    Serial.print(pos_error.x);
    Serial.print(" y:");
    Serial.print(pos_error.y);
    Serial.print(" z:");
    Serial.print(pos_error.z);
    Serial.print(" covxy:");
    Serial.print(pos_error.xy);
    Serial.print(" covxz:");
    Serial.print(pos_error.xz);
    Serial.print(" covxy:");
    Serial.print(pos_error.xy);
    Serial.println(); 
}

// print out the anchor coordinates
void print_anchor_coordinates()
{
  uint8_t list_size;
  int status;

  status = Pozyx.getDeviceListSize(&list_size);
  Serial.print("list: ");
  Serial.println(status*list_size);

  // print error if no anchors are setup
  if (list_size == 0) {
    Serial.println("No Anchors");
    Serial.println(Pozyx.getSystemError());
    return;
  }

  // retrieve anchor information
  uint16_t device_ids[list_size];
  status &= Pozyx.getDeviceIds(device_ids,list_size);

  Serial.print("Anchors found: ");
  Serial.println(list_size);

  coordinates_t anchor_coor;
  
  for (int i=0; i<list_size; i++) {
    Serial.print("A0x");
    Serial.print(device_ids[i], HEX);
    print_comma();
    status = Pozyx.getDeviceCoordinates(device_ids[i], &anchor_coor);
    Serial.print(anchor_coor.x);
    print_comma();
    Serial.print(anchor_coor.y);
    print_comma();
    Serial.println(anchor_coor.z);
  }    
}

// get ranges for each anchor
void get_ranges()
{
    // get range for each anchor
    device_range_t range;
    bool success = false;
    for (uint8_t i=0; i<NUM_ANCHORS; i++) {
        if (Pozyx.doRanging(anchor_id[i], &range) == POZYX_SUCCESS) {
            Serial.print(range.timestamp);
            Serial.print("ms \t");
            Serial.print(range.distance); 
            Serial.println("mm \t");
            success = true;
            // send info to ardupilot
            send_beacon_distance(i, range.distance);
        }
    }

    // display errors
    if (!success) {
        Serial.println("fail");
    }
}

// get position of tag
void get_position()
{
    coordinates_t position;
    pos_error_t pos_error;

    if (Pozyx.doPositioning(&position, POZYX_2_5D, 1000) == POZYX_SUCCESS) {
        if (Pozyx.getPositionError(&pos_error) == POZYX_SUCCESS) {
            // display position
            print_coordinates(position, pos_error);
            // send to ardupilot
            send_vehicle_position(position, pos_error);
        }
    } else {
        // display errors
        Serial.println("failed to calc position");
    }
}

// send all beacon config to ardupilot
void send_beacon_config()
{
    beacon_config_msg msg;
    msg.info.beacon_count = NUM_ANCHORS;
    for (uint8_t i=0; i<NUM_ANCHORS; i++) {
        msg.info.beacon_id = i;
        msg.info.x = anchors_x[i];
        msg.info.y = anchors_y[i];
        msg.info.z = heights[i];
        send_message(MSGID_BEACON_CONFIG, sizeof(msg.buf), msg.buf);
        Serial.print("Sent anchor info:");
        Serial.println(i);
    }
}

// send a beacon's distance to ardupilot
void send_beacon_distance(uint8_t beacon_id, uint32_t distance_mm)
{
    beacon_distance_msg msg;
    msg.info.beacon_id = beacon_id;
    msg.info.distance = distance_mm;
    send_message(MSGID_BEACON_DIST, sizeof(msg.buf), msg.buf);
}

// send vehicle's position to ardupilot
void send_vehicle_position(coordinates_t& position, pos_error_t& pos_error)
{
    vehicle_position_msg msg;
    
    msg.info.x = position.x;
    msg.info.y = position.y;
    msg.info.z = position.z;
    msg.info.position_error = pos_error.xy;
    send_message(MSGID_POSITION, sizeof(msg.buf), msg.buf);
}

void send_message(uint8_t msg_id, uint8_t data_len, uint8_t data_buf[])
{
    // sanity check
    if (data_len == 0) {
        return;
    }

    // message is buffer length + 1 (for checksum)
    uint8_t msg_len = data_len+1;

    // calculate checksum and place in last element of array
    uint8_t checksum = 0;
    checksum ^= msg_id;
    checksum ^= msg_len;
    for (uint8_t i=0; i<data_len; i++) {
        checksum = checksum ^ data_buf[i];
    }

    // send message
    int16_t num_sent = 0;
    num_sent += fcboardSerial.write(MSG_HEADER);
    num_sent += fcboardSerial.write(msg_id);
    num_sent += fcboardSerial.write(msg_len);
    num_sent += fcboardSerial.write(data_buf, data_len);
    num_sent += fcboardSerial.write(&checksum, 1);
    fcboardSerial.flush();
    Serial.print("Sent:");
    Serial.print(num_sent);
    Serial.print(" data_len:");
    Serial.println(data_len);
}

// GPS MAVLink message using Pozyx potision
void SendMessage(coordinates_t position)
{

  // Copy the message to send buffer 
  uint8_t len = 0;

  // Send message
  //fcboardSerial.write(buf, len);
}

