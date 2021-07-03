#include "Interpolator.hpp"
#include <ros/console.h>
#include <stdio.h>
#include <string.h>
#include <sstream>
#include <unordered_map>
/*
#include <fcntl.h>
#include <errno.h>
#include <termio.h>
#include <unistd.h>
#include <arpa/inet.h>
*/
#include <deque>
#include <vector>

#include "BSpline/src/BSpline.hpp"
#include "BSpline/src/Parametizer.hpp"

unordered_map<string, int> kludgyDXLDict;

void initKludgyDXLDict()
{
  kludgyDXLDict["r_shoulder_pitch"] = 10;
  kludgyDXLDict["r_shoulder_roll"] = 11;
  kludgyDXLDict["r_arm_yaw"] = 12;
  kludgyDXLDict["r_elbow_pitch"] = 13;
  kludgyDXLDict["r_forearm_yaw"] = 14;
  kludgyDXLDict["r_wrist_pitch"] = 15;
  kludgyDXLDict["r_wrist_roll"] = 16;
  kludgyDXLDict["r_gripper"] = 17;

  kludgyDXLDict["l_shoulder_pitch"] = 20;
  kludgyDXLDict["l_shoulder_roll"] = 21;
  kludgyDXLDict["l_arm_yaw"] = 22;
  kludgyDXLDict["l_elbow_pitch"] = 23;
  kludgyDXLDict["l_forearm_yaw"] = 24;
  kludgyDXLDict["l_wrist_pitch"] = 25;
  kludgyDXLDict["l_wrist_roll"] = 26;
  kludgyDXLDict["l_gripper"] = 27;
}

Interpolator::Interpolator(DXLPort &iPort, NodeHandle &iNH, const string &iSide, vector<string> &iJointNames, uint32_t iQueueSize)
: port(iPort), nh(iNH), enableJointStateTelemMutex(), enableJointStateTelem(true),
  jointStatePub(nh.advertise<JointState>(iSide + string("_arm_controller/joint_states"), iQueueSize)),
  trajectorySrv(nh.advertiseService<Interpolator, Trajectory::Request, Trajectory::Response>(string("action_server/") + iSide + string("_arm_trajectory"), &Interpolator::trajectorySrvCB, this)),
  enableJointStateTelemSrv(nh.advertiseService<Interpolator, SetBool::Request, SetBool::Response>(iSide + string("_arm_controller/enable_joint_state_telem"), &Interpolator::enableJointStateTelemSrvCB, this)),
  jointNames(iJointNames)
{
  int i = 8;
  while(i--)
    passThroughJointState[i] = 0.0;

  for(auto name : jointNames) {
    int id = kludgyDXLDict[name];
    DXL &actuator = port.getDXL(id, *this);
    if(id < 20) {
      switch(id) {
        case 10:
          actuator.setOffset(TO_RADIANS(-90.0));
          actuator.setPolarity(-1.0);
          break;
        case 11:
          actuator.setOffset(TO_RADIANS(-90.0));
          actuator.setPolarity(-1.0);
          break;
        case 12:
          actuator.setPolarity(-1.0);
          break;
        case 13:
          actuator.setPolarity(-1.0);
          break;
        case 14:
          actuator.setPolarity(-1.0);
          break;
        case 15:
          actuator.setPolarity(-1.0);
          break;
        case 16:
        case 17:
          break;
      }
    } else {
      switch(id) {
        case 20:
          actuator.setOffset(TO_RADIANS(90.0));
          break;
        case 21:
          actuator.setOffset(TO_RADIANS(90.0));
          actuator.setPolarity(-1.0);
          break;
        case 22:
          actuator.setPolarity(-1.0);
          break;
        case 23:
          actuator.setPolarity(-1.0);
          break;
        case 24:
          actuator.setOffset(TO_RADIANS(-15.0));
          actuator.setPolarity(-1.0);
          break;
        case 25:
          actuator.setPolarity(-1.0);
          break;
        case 26:
          actuator.setPolarity(-1.0);
          break;
        case 27:
          actuator.setPolarity(-1.0);
          break;
      }
    }
  }
}

Interpolator::~Interpolator()
{ }

bool Interpolator::trajectorySrvCB(Trajectory::Request &iReq, Trajectory::Response &iRes)
{
  #ifdef DEBUG
  ROS_INFO("trajectoryService");
  #endif

  int jointCount = iReq.dxl_ids.size();
  float goalTimeTol = iReq.goal_time_tolerance.toSec();
  float duration = iReq.time_from_start.toSec();
  int controlPointCount = iReq.control_points.size() / jointCount;
  float knots[controlPointCount + 4];

  // TODO: create additional control points if the provided count less than 4

  #if DEBUG
  {
    ostringstream ss;
    bool first = true;
    for(auto id : iReq.dxl_ids) {
      if(!first) ss << ", ";
      ss << id;
      first = false;
    }
    ROS_INFO("Trajectory ids: %s", ss.str().c_str());
  }

  {
    ROS_INFO("Trajectory:");
    for(int j = 0; j < controlPointCount; j++) {
      ostringstream ss;
      for(int i = 0; i < jointCount; i++) {
        ss << TO_DEGREES(iReq.control_points[j * jointCount + i]) << ", ";
      }
      ROS_INFO(ss.str().c_str());
    }
  }
  #endif

  #if DEBUG
  {
    ROS_INFO("Initializing spline knot vector for %d joints", jointCount);
  }
  #endif

  BSpline spline(&iReq.control_points[0], knots);
  spline.init(jointCount, controlPointCount);
  Parametizer parametizer(spline);
  parametizer.init();

  int divisions = int(duration * CONTROL_HZ);
  vector<float> parametization = parametizer.parametizeSigmoidal(divisions);

  #if DEBUG
  {
    ostringstream ss;
    bool first = true;
    for(auto time : parametization) {
      if(!first) ss << ", ";
      ss << time;
      first = false;
    }
    ROS_INFO("Parametization: %s", ss.str().c_str());
  }
  #endif

  int paramIndex = 0;
  ros::Time startTime = ros::Time::now();
  bool interpolating = true;
  vector<double> times;
  float jointState[8];
  ros::Rate spinRate(int(CONTROL_HZ));
  //ros::Time driveStart;
  //ros::Duration driveLatency;
  while(interpolating) {
    double elapsed = (ros::Time::now() - startTime).toSec();
    int newParamIndex = int(elapsed * CONTROL_HZ);
    if(newParamIndex < divisions) {
      if(newParamIndex > paramIndex) {
        float currentParam = parametization[newParamIndex];
        spline.eval(currentParam, jointState);
        #if PASSTHROUGH
        remapJointState(iReq.dxl_ids, jointState);
        #else
        //driveStart = ros.::Time::now();
        drive(iReq.dxl_ids, jointState);
        //driveLatency = ros::Time::now() - driveStart;
        //times.push_back(driveLatency.toSec());
        #endif
      }
    } else {
      interpolating = false;
    }
    spinRate.sleep();
  }
  /*
  ostringstream ss;
  bool start = true;
  for(auto time : times) {
    if(!start) ss << " ";
    ss << time;
    start = false;
  }
  ROS_INFO("Drive timings: %s", ss.str().c_str());
  */

  iRes.result = string("SUCCESS");

  return true;
}

void Interpolator::enableJointStateTelemetry(bool iEnable)
{
  lock_guard<mutex> lg(enableJointStateTelemMutex);
  enableJointStateTelem = iEnable;
}

bool Interpolator::isJointStateTelemetryEnabled()
{
  lock_guard<mutex> lg(enableJointStateTelemMutex);
  return enableJointStateTelem;
}

bool Interpolator::enableJointStateTelemSrvCB(SetBool::Request &iReq, SetBool::Response &oResp)
{
  enableJointStateTelemetry(iReq.data);
  oResp.success = true;
  oResp.message = iReq.data ? string("joint_state telemetry enabled") : string("joint_state telemetry disabled");
  return true;
}

void Interpolator::drive(vector<int> &dxl_ids, float *iJointState)
{
  // TODO: Use sync write if actually connected to a real controller
  for(int i = 0; i < dxl_ids.size(); i++) {
    int id = dxl_ids[i];
    DXL &actuator = port.getDXL(uint8_t(id), *this);
    /*
    if(actuator.getModel() == MODEL_NUMBER_DUMMY) {
      ROS_INFO("Using dummy actuator for id %d", id);
    } else {
      ROS_INFO("Attempting to set actuator %d goal position to %f", id, iJointState[i]);
    }
    */
    actuator.setGoalPosition(iJointState[i]);
  }
}

vector<double> Interpolator::getJointState() const
{
  vector<double> jointState;
  for(auto name : jointNames) {
    int id = kludgyDXLDict[name];
    DXL &actuator = port.getDXL(uint8_t(id), *this);
    jointState.push_back(double(actuator.getPresentPosition()));
  }
  return jointState;
}

void Interpolator::remapJointState(vector<int> &dxl_ids, float *iJointState)
{
  for(int src = 0; src < dxl_ids.size(); src++) {
    int dst = dxl_ids[src] - ((dxl_ids[src] < 20) ? 10 : 20);
    passThroughJointState[dst] = iJointState[src];
  }
}

void Interpolator::timerTick(const TimerEvent &iEvent)
{
  if(!isJointStateTelemetryEnabled()) return;
  JointState jntState;
  jntState.header.stamp = ros::Time::now();
  jntState.name = jointNames;
  #if PASSTHROUGH
  jntState.position = vector<double>(passThroughJointState, passThroughJointState + 8);
  #else
  jntState.position = getJointState();
  #endif
  jointStatePub.publish(jntState);
}

void Interpolator::handleError(DXL &iDXL, int iCommResult, uint8_t iErrorStatus, const string &iErrorMsg) const
{
  string resultString;
  string errorString;
  DXLErrorHandler::resultCodeToString(iCommResult, resultString);
  DXLErrorHandler::errorFlagsToString(iErrorStatus, errorString);
  if(iCommResult != COMM_SUCCESS || iErrorStatus != 0)
    ROS_INFO("Actuator %d returned comm result: %s, error status: %s, error message: %s",
      iDXL.getId(),
      resultString.c_str(),
      errorString.c_str(),
      iErrorMsg.c_str());
}
