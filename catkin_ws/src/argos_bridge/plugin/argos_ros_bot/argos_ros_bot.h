/*
 * AUTHOR: Andrew Vardy <av@mun.ca>
 *
 * Connects an ARGoS robot with a particular configuration to ROS by publishing
 * sensor values and subscribing to a desired wheel speeds topic.
 *
 */

#ifndef ARGOS_ROS_BOT_H
#define ARGOS_ROS_BOT_H

#include <argos3/core/control_interface/ci_controller.h>
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>
#include <argos3/plugins/robots/generic/control_interface/ci_positioning_sensor.h>

#include <ros/ros.h>
#include <string>
#include "geometry_msgs/Twist.h"
#include "std_msgs/Bool.h"

using namespace argos;

class CArgosRosBot : public CCI_Controller {

public:
  struct SWheelTurningParams {
   /*
    * The turning mechanism.
    * The robot can be in three different turning states.
    */
   enum ETurningMechanism
   {
      NO_TURN = 0, // go straight
      SOFT_TURN,   // both wheels are turning forwards, but at different speeds
      HARD_TURN    // wheels are turning with opposite speeds
   } TurningMechanism;
   /*
    * Angular thresholds to change turning state.
    */
   CRadians HardTurnOnAngleThreshold;
   CRadians SoftTurnOnAngleThreshold;
   CRadians NoTurnAngleThreshold;
   /* Maximum wheel speed */
   Real MaxSpeed;

   void Init(TConfigurationNode& t_tree);
 };

  CArgosRosBot();
  virtual ~CArgosRosBot() {}

  /*
   * This function initializes the controller.
   * The 't_node' variable points to the <parameters> section in the XML
   * file in the <controllers><footbot_ccw_wander_controller> section.
   */
  virtual void Init(TConfigurationNode& t_node);

  /*
   * This function is called once every time step.
   * The length of the time step is set in the XML file.
   */
  virtual void ControlStep();

  /*
   * This function resets the controller to its state right after the
   * Init().
   * It is called when you press the reset button in the GUI.
   * In this example controller there is no need for resetting anything,
   * so the function could have been omitted. It's here just for
   * completeness.
   */
  virtual void Reset() {}

  /*
   * Called to cleanup what done by Init() when the experiment finishes.
   * In this example controller there is no need for clean anything up,
   * so the function could have been omitted. It's here just for
   * completeness.
   */
  virtual void Destroy() {}

  /*
   * The callback method for getting haptic force vector on the haptic topic.
   */
  void hapticCallback(const argos_bridge::Haptic& haptic);


  void SetWheelSpeedsFromVector(const CVector2& c_heading);

private:

  CCI_DifferentialSteeringActuator* m_pcWheels;
  CCI_PositioningSensor* m_pcState;

  // The following constant values were copied from the argos source tree from
  // the file src/plugins/robots/foot-bot/simulator/footbot_entity.cpp
  static constexpr Real HALF_BASELINE = 0.07f; // Half the distance between wheels
  static constexpr Real WHEEL_RADIUS = 0.029112741f;

  /*
   * The following variables are used as parameters for the
   * algorithm. You can set their value in the <parameters> section
   * of the XML configuration file, under the
   * <controllers><argos_ros_bot_controller> section.
   */

  Real xForce, yForce, zForce;

  // haptic sensor subscriber
  ros::Subscriber hapticSub;

  // state publisher
  ros::Publisher statePub;

  /* The turning parameters. */
  SWheelTurningParams m_sWheelTurningParams;

public:
  // We need only a single ROS node, although there are individual publishers
  // and subscribers for each instance of the class.
  static ros::NodeHandle* nodeHandle;
};

#endif
