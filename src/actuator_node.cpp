#include <ros/ros.h>
#include <vector>

#include "std_msgs/String.h"
#include "dynamixel_sdk_examples/BulkGetItem.h"
#include "dynamixel_sdk_examples/BulkSetItem.h"
#include "dynamixel_sdk/dynamixel_sdk.h"

using namespace dynamixel; 

// Control table address
#define ADDR_TORQUE_ENABLE    64
#define ADDR_PRESENT_LED      65
#define ADDR_PRESENT_VELOCITY 128
#define ADDR_GOAL_VELOCITY    104

// Protocol version
#define PROTOCOL_VERSION      2.0             // Default Protocol version of DYNAMIXEL X series.

// Default setting
#define DXL1_ID               41               // DXL1 ID
#define DXL2_ID               42               // DXL2 ID
#define DXL3_ID               43 
#define DXL4_ID               11               // DXL1 ID
#define DXL5_ID               12               // D XL2 ID
#define DXL6_ID               13 
#define BAUDRATE              115200           // Default Baudrate of DYNAMIXEL X series
#define DEVICE_NAME           "/dev/ttyUSB0"  // [Linux] To find assigned port, use "$ ls /dev/ttyUSB*" command
#define NUM_MOTORS            6 
#define NATIVE_GAIN           41.7014178f      // Gain to convert from rad/s to native units for input
#define PULLEY_RADIUS         0.015f              // Radius in terms of metres.

PortHandler * portHandler = PortHandler::getPortHandler(DEVICE_NAME);
PacketHandler * packetHandler = PacketHandler::getPacketHandler(PROTOCOL_VERSION);

GroupBulkRead groupBulkRead(portHandler, packetHandler);
GroupBulkWrite groupBulkWrite(portHandler, packetHandler);

int add_velocity_params(uint8_t* addr_goal_array, uint8_t* len_goal, uint8_t* param_goal_vel, int32_t value, uint8_t id) 

{

    uint8_t dxl_error = 0;
    int dxl_comm_result = COMM_TX_FAIL;
    int dxl_addparam_result = false;

    uint32_t velocity = (unsigned int)value; // Convert int32 -> uint32
    param_goal_vel[0] = DXL_LOBYTE(DXL_LOWORD(velocity));
    param_goal_vel[1] = DXL_HIBYTE(DXL_LOWORD(velocity));
    param_goal_vel[2] = DXL_LOBYTE(DXL_HIWORD(velocity));
    param_goal_vel[3] = DXL_HIBYTE(DXL_HIWORD(velocity));

    dxl_addparam_result = groupBulkWrite.addParam((uint8_t)id, ADDR_GOAL_VELOCITY,
     4, param_goal_vel);


    if (dxl_addparam_result != true) {
        ROS_ERROR("Failed to addparam to groupBulkWrite for Dynamixel ID: %d", id);
    }

    return 0; 

}

int32_t convert_to_native_units(double velocity) 

{

    int32_t native_velocity = int32_t(NATIVE_GAIN*(velocity/PULLEY_RADIUS));
    std::cout << "Native Velocity: " << native_velocity << "\n";
    return native_velocity; 

}

void bulkSetItemCallback(const dynamixel_sdk_examples::BulkSetItem::ConstPtr & msg)
{
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;
  int dxl_addparam_result = false;
  uint8_t param_goal_velocity1[4];
  uint8_t param_goal_velocity2[4];
  uint8_t param_goal_velocity3[4];
  uint8_t param_goal_velocity4[4];
  uint8_t param_goal_velocity5[4];
  uint8_t param_goal_velocity6[4];
  uint8_t addr_goal_item[6];
  uint8_t len_goal_item[6];

  add_velocity_params(addr_goal_item, len_goal_item, param_goal_velocity1, convert_to_native_units(msg->value1), msg->id1);
  add_velocity_params(addr_goal_item, len_goal_item, param_goal_velocity2, convert_to_native_units(msg->value2), msg->id2);
  add_velocity_params(addr_goal_item, len_goal_item, param_goal_velocity3, convert_to_native_units(msg->value3), msg->id3);
  add_velocity_params(addr_goal_item, len_goal_item, param_goal_velocity4, convert_to_native_units(msg->value4), msg->id4);
  add_velocity_params(addr_goal_item, len_goal_item, param_goal_velocity5, convert_to_native_units(msg->value5), msg->id5);
  add_velocity_params(addr_goal_item, len_goal_item, param_goal_velocity6, convert_to_native_units(msg->value6), msg->id6);

  dxl_comm_result = groupBulkWrite.txPacket();
  if (dxl_comm_result != COMM_SUCCESS) {
    ROS_INFO("Failed to set position! Result: %d", dxl_comm_result);
    }

  groupBulkWrite.clearParam();
}


int enable_torque(int DXL_ID) 

{

  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;

  dxl_comm_result = packetHandler->write1ByteTxRx(
    portHandler, DXL_ID, ADDR_TORQUE_ENABLE, 1, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) {
    ROS_ERROR("Failed to enable torque for Dynamixel ID: %d", DXL_ID);
    return -1;
  }

  return 0;  


}

int main(int argc, char ** argv)
{
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;

  std::vector<int> motor_ids = {DXL1_ID, DXL2_ID, DXL3_ID, 
    DXL4_ID, DXL5_ID, DXL6_ID}; 

  if (!portHandler->openPort()) {
    ROS_ERROR("Failed to open the port!");
    return -1;
  }

  if (!portHandler->setBaudRate(BAUDRATE)) {
    ROS_ERROR("Failed to set the baudrate!");
    return -1;
  }

  dxl_comm_result = packetHandler->write1ByteTxRx(
    portHandler, DXL1_ID, ADDR_TORQUE_ENABLE, 1, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) {
    ROS_ERROR("Failed to enable torque for Dynamixel ID: %d", DXL1_ID);
    return -1;
  }

  for (int i:motor_ids) 
  
  {
    
    enable_torque(i);

  } 


  ros::init(argc, argv, "bulk_read_write_node");
  ros::NodeHandle nh;
//   ros::ServiceServer bulk_get_item_srv = nh.advertiseService("/bulk_get_item", bulkGetItemCallback);
  ros::Subscriber bulk_set_item_sub = nh.subscribe("/bulk_set_item", 10, bulkSetItemCallback);
  ros::spin();

  portHandler->closePort();
  return 0;
}