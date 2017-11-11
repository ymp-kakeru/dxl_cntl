#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
//#include <senser_msgs/JointState.h>

#if defined(__linux__) || defined(__APPLE__)
#include <fcntl.h>
#include <termios.h>
#define STDIN_FILENO 0
#elif defined(_WIN32) || defined(_WIN64)
#include <conio.h>
#endif

#include <dynamixel_sdk/dynamixel_sdk.h>                                  // Uses Dynamixel SDK library

#include <stdlib.h>
#include <stdio.h>


// Control table address
#define ADDR_PRO_TORQUE_ENABLE          562                 // Control table address is different in Dynamixel model
#define ADDR_PRO_GOAL_POSITION          596
#define ADDR_PRO_PRESENT_POSITION       611

// Data Byte Length
#define LEN_PRO_GOAL_POSITION           4
#define LEN_PRO_PRESENT_POSITION        4

// Protocol version
#define PROTOCOL_VERSION                2.0                 // See which protocol version is used in the Dynamixel

// Default setting
#define DXL_FR_ID                         1                   // Dynamixel#1 ID: 1
#define DXL_FL_ID                         2                   // Dynamixel#2 ID: 2
#define DXL_RR_ID                         3
#define DXL_RL_ID                         4                   // Dynamixel#4 ID: 4
#define BAUDRATE                        57600
#define DEVICENAME                      "/dev/ttyUSB0"      // Check which port is being used on your controller
                                                            // ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

#define TORQUE_ENABLE                   1                   // Value for enabling the torque
#define TORQUE_DISABLE                  0                   // Value for disabling the torque
#define DXL_MINIMUM_POSITION_VALUE      -150000             // Dynamixel will rotate between this value
#define DXL_MAXIMUM_POSITION_VALUE      150000              // and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
#define DXL_MOVING_STATUS_THRESHOLD     20                  // Dynamixel moving status threshold

#define ESC_ASCII_VALUE                 0x1b

//#define dxl_goal_position                0

int getch()
{
#if defined(__linux__) || defined(__APPLE__)
  struct termios oldt, newt;
  int ch;
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  ch = getchar();
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  return ch;
#elif defined(_WIN32) || defined(_WIN64)
  return _getch();
#endif
}

int kbhit(void)
{
#if defined(__linux__) || defined(__APPLE__)
  struct termios oldt, newt;
  int ch;
  int oldf;

  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
  fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

  ch = getchar();

  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  fcntl(STDIN_FILENO, F_SETFL, oldf);

  if (ch != EOF)
  {
    ungetc(ch, stdin);
    return 1;
  }

  return 0;
#elif defined(_WIN32) || defined(_WIN64)
  return _kbhit();
#endif
}

void dxlCallback(const geometry_msgs::Twist::ConstPtr& cmd_vel_s)
{
 printf("\nDXL Callback Success\n") ;

   int a = cmd_vel_s->angular.z * 1000;
   int dxl_goal_position = 0 ;
   int dxl_goal_position_R = 0 ;
   dxl_goal_position = a; 
   dxl_goal_position_R = a ;
// Initialize PortHandler instance
  // Set the port path
  // Get methods and members of PortHandlerLinux or PortHandlerWindows
  dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);
  // Initialize PacketHandler instance
  // Set the protocol version
  // Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
  dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);
  // Initialize GroupSyncWrite instance
  dynamixel::GroupSyncWrite groupSyncWrite(portHandler, packetHandler, ADDR_PRO_GOAL_POSITION, LEN_PRO_GOAL_POSITION);
  // Initialize Groupsyncread instance for Present Position
  dynamixel::GroupSyncRead groupSyncRead(portHandler, packetHandler, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);
  
  //int dxl_goal_position = 0;
  
  int index = 0;
  bool dxl_addparam_result = false;                 // addParam result
  bool dxl_getdata_result = false;                  // GetParam result
  int dxl_comm_result = COMM_TX_FAIL;             // Communication result
  //int dxl_goal_position[2] = {DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE};         // Goal position
  uint8_t dxl_error = 0;                            // Dynamixel error
  uint8_t param_goal_position[4];
  uint8_t param_goal_position_R[4];
  int32_t dxl_fr_present_position = 0, dxl_fl_present_position = 0;                         // Present position

  // Open port
  portHandler->openPort() ;
  // Set port baudrate
  portHandler->setBaudRate(BAUDRATE) ;

  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_FR_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS)
  {
    printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
  }
  else if (dxl_error != 0)
  {
    printf("%s\n", packetHandler->getRxPacketError(dxl_error));
  }
  else
  {
    printf("Dynamixel has been successfully connected \n");
  }

  //Enabel FL DXL
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_FL_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS)
  {
    printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
  }
  else if (dxl_error != 0)
  {
    printf("%s\n", packetHandler->getRxPacketError(dxl_error));
  }
  else
  {
    printf("Dynamixel has been successfully connected \n");
  }
  //Enabel FR DXL
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_FR_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS)
  {
    printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
  }
  else if (dxl_error != 0)
  {
    printf("%s\n", packetHandler->getRxPacketError(dxl_error));
  }
  else
  {
    printf("Dynamixel has been successfully connected \n");
  }

  //Enabel RL DXL
  /*dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_RL_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS)
  {
    printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
  }
  else if (dxl_error != 0)
  {
    printf("%s\n", packetHandler->getRxPacketError(dxl_error));
  }
  else
  {
    printf("Dynamixel has been successfully connected \n");
  }

  //Enabel RR DXL
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_RR_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS)
  {
    printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
  }
  else if (dxl_error != 0)
  {
    printf("%s\n", packetHandler->getRxPacketError(dxl_error));
  }
  else
  {
    printf("Dynamixel has been successfully connected \n");
  }
*/
  // Add parameter storage for Dynamixel FR present position value
  dxl_addparam_result = groupSyncRead.addParam(DXL_FR_ID);
  if (dxl_addparam_result != true)
  {
    fprintf(stderr, "[ID:%03d] groupSyncRead addparam failed", DXL_FR_ID);
    
    //return 0;
  }

  // Add parameter storage for Dynamixel FL present position value
  dxl_addparam_result = groupSyncRead.addParam(DXL_FL_ID);
  if (dxl_addparam_result != true)
  {
    fprintf(stderr, "[ID:%03d] groupSyncRead addparam failed", DXL_FL_ID);
    
    //return 0;
  }

// Add parameter storage for Dynamixel RR present position value
  /*dxl_addparam_result = groupSyncRead.addParam(DXL_RR_ID);
  if (dxl_addparam_result != true)
  {
    fprintf(stderr, "[ID:%03d] groupSyncRead addparam failed", DXL_RR_ID);
    
    //return 0;
  }

// Add parameter storage for Dynamixel FR present position value
  dxl_addparam_result = groupSyncRead.addParam(DXL_RL_ID);
  if (dxl_addparam_result != true)
  {
    fprintf(stderr, "[ID:%03d] groupSyncRead addparam failed", DXL_RL_ID);
    
    //return 0;
  }
*/
  // Allocate goal position value into byte array
  param_goal_position[0] = DXL_LOBYTE(DXL_LOWORD(dxl_goal_position));
  param_goal_position[1] = DXL_HIBYTE(DXL_LOWORD(dxl_goal_position));
  param_goal_position[2] = DXL_LOBYTE(DXL_HIWORD(dxl_goal_position));
  param_goal_position[3] = DXL_HIBYTE(DXL_HIWORD(dxl_goal_position));

  param_goal_position_R[0] = DXL_LOBYTE(DXL_LOWORD(dxl_goal_position_R));
  param_goal_position_R[1] = DXL_HIBYTE(DXL_LOWORD(dxl_goal_position_R));
  param_goal_position_R[2] = DXL_LOBYTE(DXL_HIWORD(dxl_goal_position_R));
  param_goal_position_R[3] = DXL_HIBYTE(DXL_HIWORD(dxl_goal_position_R));

    dxl_addparam_result = groupSyncWrite.addParam(DXL_FR_ID, param_goal_position);
    if (dxl_addparam_result != true)
    {
      fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", DXL_FR_ID);
      //return 0;
    }

    // Add Dynamixel#2 goal position value to the Syncwrite parameter storage
    dxl_addparam_result = groupSyncWrite.addParam(DXL_FL_ID, param_goal_position);
    if (dxl_addparam_result != true)
    {
      fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", DXL_FL_ID);
      //return 0;
    }
    // Add Dynamixel#2 goal position value to the Syncwrite parameter storage
    /*dxl_addparam_result = groupSyncWrite.addParam(DXL_RR_ID, param_goal_position_R);
    if (dxl_addparam_result != true)
    {
      fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", DXL_RR_ID);
      //return 0;
    }
    // Add Dynamixel#2 goal position value to the Syncwrite parameter storage
    dxl_addparam_result = groupSyncWrite.addParam(DXL_RL_ID, param_goal_position_R);
    if (dxl_addparam_result != true)
    {
      fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", DXL_RL_ID);
      //return 0;
    }
*/
    // Syncwrite goal position
    dxl_comm_result = groupSyncWrite.txPacket();
    if (dxl_comm_result != COMM_SUCCESS) printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));

    // Clear syncwrite parameter storage
    groupSyncWrite.clearParam();


  do
  {
    // Syncread present position
    dxl_comm_result = groupSyncRead.txRxPacket();
    if (dxl_comm_result != COMM_SUCCESS) printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
    // Check if groupsyncread data of Dynamixel#1 is available
    dxl_getdata_result = groupSyncRead.isAvailable(DXL_FR_ID, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);
    if (dxl_getdata_result != true)
    {
      fprintf(stderr, "[ID:%03d] groupSyncRead getdata failed", DXL_FR_ID);
      
      //return 0;
    }
    // Check if groupsyncread data of Dynamixel#2 is available
    dxl_getdata_result = groupSyncRead.isAvailable(DXL_FL_ID, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);
    if (dxl_getdata_result != true)
    {
      fprintf(stderr, "[ID:%03d] groupSyncRead getdata failed", DXL_FL_ID);
      
      //return 0;
    }
// Check if groupsyncread data of Dynamixel#1 is available
    /*dxl_getdata_result = groupSyncRead.isAvailable(DXL_RR_ID, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);
    if (dxl_getdata_result != true)
    {
      fprintf(stderr, "[ID:%03d] groupSyncRead getdata failed", DXL_RR_ID);
      
      //return 0;
    }
  // Check if groupsyncread data of Dynamixel#1 is available
    dxl_getdata_result = groupSyncRead.isAvailable(DXL_RL_ID, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);
    if (dxl_getdata_result != true)
    {
      fprintf(stderr, "[ID:%03d] groupSyncRead getdata failed", DXL_RL_ID);
      
      //return 0;
    }*/


    // Get Dynamixel#1 present position value
    dxl_fr_present_position = groupSyncRead.getData(DXL_FR_ID, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);
    // Get Dynamixel#2 present position value
    dxl_fl_present_position = groupSyncRead.getData(DXL_FL_ID, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);
    // Get Dynamixel#1 present position value
    //dxl_fr_present_position = groupSyncRead.getData(DXL_RR_ID, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);
    // Get Dynamixel#1 present position value
    //dxl_fr_present_position = groupSyncRead.getData(DXL_RL_ID, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);
    printf("[ID:%03d] GoalPos:%03d  PresPos:%03d\t[ID:%03d] GoalPos:%03d  PresPos:%03d\n", DXL_FR_ID, dxl_goal_position, dxl_fr_present_position, DXL_FL_ID, dxl_goal_position, dxl_fl_present_position);

  }while((abs(dxl_goal_position - dxl_fr_present_position) > DXL_MOVING_STATUS_THRESHOLD) || (abs(dxl_goal_position - dxl_fl_present_position) > DXL_MOVING_STATUS_THRESHOLD));


  //if(getch() == ESC_ASCII_VALUE){
   // Disable FR Dynamixel Torque
  
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_FR_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
      printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
      printf("%s\n", packetHandler->getRxPacketError(dxl_error));
     }
   // Disable FL Dynamixel Torque
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_FL_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
      printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
      printf("%s\n", packetHandler->getRxPacketError(dxl_error));
    }
 /*   // Disable FR Dynamixel Torque
  
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_RR_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
      printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
      printf("%s\n", packetHandler->getRxPacketError(dxl_error));
     }
     // Disable FR Dynamixel Torque
  
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_RL_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
      printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
      printf("%s\n", packetHandler->getRxPacketError(dxl_error));
     }*/

    // Close port
    portHandler->closePort();
  //}
 
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "dxl_cntl") ;
  ros::NodeHandle nh_;
  //ros::Publisher joint_state_pub_ = nh_.advertise<sensor_msgs::JointState>(n.param<std::string>("joint_state_topic_name", "/joint_states"), 10);
  ros::Subscriber dxl_sub_ = nh_.subscribe<geometry_msgs::Twist>("cmd_vel_s",1,dxlCallback);
  //dxlCallback(cmd_vel_s) ;
  
  ros::spin();
}
