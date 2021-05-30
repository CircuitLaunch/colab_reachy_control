#include "ros/ros.h"
#include <ros/console.h>
#include <stdio.h>
#include <string.h>
#include <sstream>
#include <fcntl.h>
#include <errno.h>
#include <termio.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <deque>
#include <vector>
#include <colab_reachy_control/Trajectory.h>
#include <sensor_msgs/JointState.h>
#include "BSpline/BSpline.hpp"
#include "BSpline/Parametizer.hpp"

using namespace std;
using namespace sensor_msgs;

enum
{
  INTERPOLATE = 10,
  ABORT,
  TORQUE_ENABLE,
  RECOVER,

  JOINT_STATE = 100,
  TRAJECTORY_COMPLETE,
  DXL_STATUS
};

/*
void subCallback(const colab_reachy_control::Trajectory &iTraj);
int startSerial(const char *iPort);
void processMsg(SerialStream::Message &iMsg);
void reportJointState(SerialStream::Message &iMsg);
*/

class Interpolator
{
  public:
    Interpolator(ros::NodeHandle &iNH, const string &iJointStateTopic, const string &iTrajectoryTopic, vector<string> &iJointNames, uint32_t iQueueSize = 10);

    void launchTrajectory(const colab_reachy_control::Trajectory &iTraj);

    void tick(ros::Time &iTime);

    void reportJointState();

  public:
    ros::NodeHandle &nh;
    ros::Publisher jointStatePub;
    ros::Subscriber trajectorySub;
    vector<string> jointNames;

    float controlPoints[800];
    float knots[104];
    float jointState[8];
    float goalTimeTol;

    BSpline spline;
    Parametizer parametizer;

    int divisions;
    vector<float> parametization;
    bool interpolating;

    int paramIndex;
    ros::Time startTime;
};

Interpolator::Interpolator(ros::NodeHandle &iNH, const string &iJointStateTopic, const string &iTrajectoryTopic, vector<string> &iJointNames, uint32_t iQueueSize)
: nh(iNH),
  jointStatePub(nh.advertise<JointState>(iJointStateTopic, iQueueSize)),
  trajectorySub(nh.subscribe<const colab_reachy_control::Trajectory &, Interpolator>(iTrajectoryTopic, iQueueSize, &Interpolator::launchTrajectory, this)),
  jointNames(iJointNames),
  spline(controlPoints, knots, 100),
  parametizer(spline),
  interpolating(false)
{ }

void Interpolator::launchTrajectory(const colab_reachy_control::Trajectory &iTraj)
{
  ROS_INFO("launchTrajectory");
  goalTimeTol = iTraj.goal_time_tolerance;
  int jointCount = iTraj.dxl_ids.size();
  int controlPointCount = iTraj.control_points.size() / jointCount;
  int floatCount = iTraj.control_points.size();
  if(floatCount > 800) floatCount = 800;

  while(floatCount--) {
    controlPoints[floatCount] = iTraj.control_points[floatCount];
  }

  spline.init(jointCount, controlPointCount);
  parametizer.init();

  divisions = int(goalTimeTol / 0.06666666); // 15 frames a second
  parametization = parametizer.parametizeSigmoidal(divisions);

  ostringstream ss;
  bool first = true;
  for(auto time : parametization) {
    if(!first) ss << ", ";
    ss << time;
    first = false;
  }
  ROS_INFO("Parametization: %s", ss.str().c_str());

  paramIndex = 0;
  startTime = ros::Time::now();
  interpolating = true;
}

void Interpolator::tick(ros::Time &t)
{
  if(interpolating) {
    double elapsed = (t - startTime).toSec();
    int newParamIndex = int(elapsed / 0.06666666);
    if(newParamIndex < divisions) {
      if(newParamIndex > paramIndex) {
        float currentParam = parametization[newParamIndex];
        spline.eval(currentParam, jointState);
        ROS_INFO("%d: %f (%f, %f, %f, %f, %f, %f, %f, %f)", newParamIndex, currentParam, jointState[0], jointState[1], jointState[2], jointState[3], jointState[4], jointState[5], jointState[6], jointState[7]);
        paramIndex = newParamIndex;
      }
    } else {
      interpolating = false;
    }
  }
  reportJointState();
}

void Interpolator::reportJointState()
{
  JointState jntState;
  jntState.header.stamp = ros::Time::now();
  jntState.name = jointNames;
  jntState.position = vector<double>(jointState, jointState + 8);
  jointStatePub.publish(jntState);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "interpolator");

  ros::NodeHandle n;

  vector<string> rightArmJointNames;
  rightArmJointNames.push_back("r_shoulder_pitch");
  rightArmJointNames.push_back("r_shoulder_roll");
  rightArmJointNames.push_back("r_arm_yaw");
  rightArmJointNames.push_back("r_elbow_pitch");
  rightArmJointNames.push_back("r_forearm_yaw");
  rightArmJointNames.push_back("r_wrist_pitch");
  rightArmJointNames.push_back("r_wrist_roll");
  rightArmJointNames.push_back("r_gripper");

  vector<string> leftArmJointNames;
  leftArmJointNames.push_back("l_shoulder_pitch");
  leftArmJointNames.push_back("l_shoulder_roll");
  leftArmJointNames.push_back("l_arm_yaw");
  leftArmJointNames.push_back("l_elbow_pitch");
  leftArmJointNames.push_back("l_forearm_yaw");
  leftArmJointNames.push_back("l_wrist_pitch");
  leftArmJointNames.push_back("l_wrist_roll");
  leftArmJointNames.push_back("l_gripper");

  Interpolator rai(n, string("right_arm_controller/joint_state"), string("action_server/right_arm_trajectory"), rightArmJointNames, 10);
  Interpolator lai(n, string("left_arm_controller/joint_state"), string("action_server/left_arm_trajectory"), leftArmJointNames, 10);

  ros::Time time = ros::Time::now();
  ros::Time controlThrottle = time;
  ros::Time rosThrottle = time;
  while(ros::ok()) {
    time = ros::Time::now();

    if((time - controlThrottle) > ros::Duration(0.0666666)) {
      rai.tick(time);
      lai.tick(time);
      controlThrottle = time;
    }

    if((time - rosThrottle) > ros::Duration(0.03333333)) {
      ros::spinOnce();
      rosThrottle = time;
    }
  }
}

/*
SerialStream ss;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "interpolator");
  ros::NodeHandle n;

  rArmJntStPub = n.advertise<JointState>("right_arm_controller/joint_state", 10);
  lArmJntStPub = n.advertise<JointState>("left_arm_controller/joint_state", 10);

  ros::Subscriber trajSub = n.subscribe("action_server/trajectory", 5, subCallback);

  ros::Time rosThrottle = ros::Time::now();

  ros::Rate rate(30);

  ROS_INFO("Starting serial connection to Arduino");
  int serial_fd = startSerial("/dev/ttyACM0");
  ss.setFD(serial_fd);
  ss.recycleRecvMessage(new SerialStream::Message);

  while(ros::ok()) {
    ss.tick();

    SerialStream::Message *recvMsg;
    if(recvMsg = ss.popRecvMessage()) {
      processMsg(*recvMsg);
      ss.recycleRecvMessage(recvMsg);
    }

    if(ros::Time::now() - rosThrottle > ros::Duration(0.033)) {
      rosThrottle = ros::Time::now();
      ros::spinOnce();
    }
  }

  return 0;
}

void subCallback(const colab_reachy_control::Trajectory &iTraj)
{
  char side = (iTraj.dxl_ids[0]) < 20 ? 'r' : 'l';
  int actuatorCount = iTraj.dxl_ids.size();
  int controlPointCount = iTraj.waypoints.size() / actuatorCount;
  ROS_INFO(
    "Received trajectory for %d actuators with %d waypoints and goal time tolerance %f",
    actuatorCount, controlPointCount, iTraj.goal_time_tolerance);

  float *ptr;
  switch(side) {
    case 'r':
      ptr = rArmControlPoints;
      break;
    case 'l':
      ptr = lArmControlPoints;
      break;
  }

  for(auto waypoint : iTraj.waypoints) {
    for(auto position : waypoint.position) {
      *ptr = position;
      ptr++;
    }
  }

  switch(side) {
    case 'r':
      rArmSpline.initialize(actuatorCount, controlPointCount);
      break;
    case 'l':
      lArmSpline.initialize(actuatorCount, controlPointCount);
      break;
  }

  / *
  vector<WayPoint> waypoints;
  int c = waypointCount;
  while(c--) {
    vector<float> partition(i, i + actuatorCount);
    waypoints.push_back(WayPoint(partition));
    i += actuatorCount;
  }
  * /

  float knotsVector[controlPointCount + 4];
  ParametizingBSpline spline((float *) &iTraj.waypoints[0], knotsVector, actuatorCount, controlPointCount);
  spline.initialize(controlPointCount);

  float duration = 5.0;
  int stepCount = 100;
  float step = duration / float(stepCount - 1);
  vector<float> keyTimes;
  vector<float> parametization = spline.parameterizeSigmoid(stepCount);

  for(int i = 0; i < stepCount; i++) {
    keyTimes.push_back(step * float(i));
  }

  {
    ostringstream s;

    s << "Key times: ";
    for(float f : keyTimes) {
      s << " " << f;
    }

    ROS_INFO("%s", s.str().c_str());
  }

  {
    ostringstream s;

    s << "Parametization: ";
    for(float f : parametization) {
      s << " " << f;
    }

    ROS_INFO("%s", s.str().c_str());
  }

  SerialStream::Message *msg = ss.getSendMessage();
  msg->cmd = INTERPOLATE;

  msg->setPacketCount(4);

  SerialStream::Packet *pkt0 = msg->packets[0];
  pkt0->type = INT32;
  pkt0->byteSize = sizeof(uint32_t) * actuatorCount;
  pkt0->resizeIfNeeded();
  memmove(pkt0->data, (uint8_t *) &iTraj.dxl_ids[0], pkt0->byteSize);

  SerialStream::Packet *pkt1 = msg->packets[1];
  pkt1->type = FLOAT32;
  pkt1->byteSize = sizeof(float) * keyTimes.size();
  pkt1->resizeIfNeeded();
  memmove(pkt1->data, (uint8_t *) &keyTimes[0], pkt1->byteSize);

  SerialStream::Packet *pkt2 = msg->packets[1];
  pkt2->type = FLOAT32;
  pkt2->byteSize = sizeof(float) * parametization.size();
  pkt2->resizeIfNeeded();
  memmove(pkt2->data, (uint8_t *) &parametization[0], pkt2->byteSize);

  SerialStream::Packet *pkt3 = msg->packets[2];
  pkt2->type = FLOAT32;
  pkt2->byteSize = sizeof(float) * iTraj.waypoints.size();
  pkt2->resizeIfNeeded();
  memmove(pkt2->data, (uint8_t *) &iTraj.waypoints[0], pkt2->byteSize);

  ss.queueSendMessage(msg);
}

int startSerial(const char *iPort)
{
  int serial_fd = open(iPort, O_RDWR);
  if(serial_fd >= 0) {
    ROS_INFO("Success!");
  } else {
    ROS_INFO("Failure!");
    exit(-1);
  }

  struct termios portSettings;
  if(tcgetattr(serial_fd, &portSettings) < 0) {
    ROS_INFO("Failed to get port settings");
    close(serial_fd);
    serial_fd = -1;
    exit(-1);
  } else {
    portSettings.c_cflag &= ~PARENB; // Disable parity
    portSettings.c_cflag &= ~CSTOPB; // Use only 1 stop bit
    portSettings.c_cflag &= ~CSIZE;
    portSettings.c_cflag |= CS8; // 8-bit bytes
    portSettings.c_cflag &= ~CRTSCTS; // Disable flow control
    portSettings.c_cflag |= CREAD | CLOCAL;
    portSettings.c_lflag &= ~ICANON; // Non-cononical mode
    portSettings.c_lflag &= ~ECHO;  // Disable echo
    portSettings.c_lflag &= ~ECHOE; // Disable erasure
    portSettings.c_lflag &= ~ECHONL; // Disable new-line echo
    portSettings.c_lflag &= ~ISIG; // Disable ISIG, INTR, QUIT, SUSP
    portSettings.c_iflag &= ~(IXON | IXOFF | IXANY);
    portSettings.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL);
    portSettings.c_oflag &= ~(OPOST | ONLCR);
    portSettings.c_cc[VMIN] = 0;
    portSettings.c_cc[VTIME] = 10; // Block of 1 second
    cfsetispeed(&portSettings, B115200);
    cfsetospeed(&portSettings, B115200);
    if(tcsetattr(serial_fd, TCSANOW, &portSettings) < 0) {
      ROS_INFO("Failed to set baud rate");
      close(serial_fd);
      serial_fd = -1;
      exit(-1);
    }
  }

  sleep(2);
  tcflush(serial_fd, TCIOFLUSH);

  return serial_fd;
}

void processMsg(SerialStream::Message &iMsg)
{
  switch(iMsg.cmd) {
    case JOINT_STATE:
      reportJointState(iMsg);
      break;
  }
}

const char *jointNames[] = {
  "r_shoulder_pitch",
  "r_shoulder_roll",
  "r_arm_yaw",
  "r_elbow_pitch",
  "r_forearm_yaw",
  "r_wrist_pitch",
  "r_wrist_roll",
  "r_gripper",

  "l_shoulder_pitch",
  "l_shoulder_roll",
  "l_arm_yaw",
  "l_elbow_pitch",
  "l_forearm_yaw",
  "l_wrist_pitch",
  "l_wrist_roll",
  "l_gripper"
};

void reportJointState(SerialStream::Message &iMsg)
{
  if(iMsg.cmd == JOINT_STATE) {
    SerialStream::Packet *pkt0 = iMsg.packets[0];
    uint16_t dxlCount = pkt0->size();
    uint32_t *dxlIds = pkt0->getData<uint32_t>();

    SerialStream::Packet *pkt1 = iMsg.packets[1];
    float *positions = pkt1->getData<float>();

    JointState rArmJointState, lArmJointState;
    rArmJointState.header.stamp = ros::Time::now();
    lArmJointState.header.stamp = ros::Time::now();

    bool pubR = false;
    bool pubL = false;
    for(int i = 0; i < dxlCount; i++) {
      if(dxlIds[i] < 20) {
        pubR = true;
        rArmJointState.name.push_back(jointNames[dxlIds[i] - 10]);
        rArmJointState.position.push_back(positions[i]);
      } else {
        pubL = true;
        lArmJointState.name.push_back(jointNames[dxlIds[i] - 20]);
        lArmJointState.position.push_back(positions[i]);
      }
    }

    if(pubR) rArmJntStPub.publish(rArmJointState);
    if(pubL) lArmJntStPub.publish(lArmJointState);
  }
}
*/
