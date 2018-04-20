/* Include the controller definition */
#include "argos_ros_bot.h"
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/core/utility/logging/argos_log.h>

#include <iostream>
#include <sstream>
#include <math.h>
using namespace std;
using namespace argos_bridge;
#define PI 3.14159265

// Initialize ROS node.  There will be only one ROS node no matter how many robots are created in
// ARGoS.  However, we will have one instance of the CArgosRosBot class for each ARGoS robot.
ros::NodeHandle* initROS() {
  int argc = 0;
  char *argv = (char *) "";
  ros::init(argc, &argv, "argos_ros_bot");
  return new ros::NodeHandle();
}
//try: multiple nodeHandles  on same namspace (argos_ros_bot), or GetNodeHandle (if exists) for this node
// and pass it around. ***Or nodelets for each bot!!***.
// or maybe shared_ptr<ros::NodeHandle> nodeHandle under the class definiton
// check the ~/.ros/log/latest for subscription, connection info etc.
ros::NodeHandle* CFootBotFlocking::nodeHandle = initROS();


/****************************************/
/****************************************/

void CFootBotFlocking::SWheelTurningParams::Init(TConfigurationNode& t_node) {
   try {
      TurningMechanism = NO_TURN;
      CDegrees cAngle;
      GetNodeAttribute(t_node, "hard_turn_angle_threshold", cAngle);
      HardTurnOnAngleThreshold = ToRadians(cAngle);
      GetNodeAttribute(t_node, "soft_turn_angle_threshold", cAngle);
      SoftTurnOnAngleThreshold = ToRadians(cAngle);
      GetNodeAttribute(t_node, "no_turn_angle_threshold", cAngle);
      NoTurnAngleThreshold = ToRadians(cAngle);
      GetNodeAttribute(t_node, "max_speed", MaxSpeed);
   }
   catch(CARGoSException& ex) {
      THROW_ARGOSEXCEPTION_NESTED("Error initializing controller wheel turning parameters.", ex);
   }
}

/****************************************/
/****************************************/

void CFootBotFlocking::SFlockingInteractionParams::Init(TConfigurationNode& t_node) {
   try {
      GetNodeAttribute(t_node, "target_distance", TargetDistance);
      GetNodeAttribute(t_node, "gain", Gain);
      GetNodeAttribute(t_node, "exponent", Exponent);
   }
   catch(CARGoSException& ex) {
      THROW_ARGOSEXCEPTION_NESTED("Error initializing controller flocking parameters.", ex);
   }
}

/****************************************/
/****************************************/

/*
 * This function is a generalization of the Lennard-Jones potential
 */
Real CFootBotFlocking::SFlockingInteractionParams::GeneralizedLennardJones(Real f_distance) { 
  
  Real fNormDistExp = ::pow( TargetDistance / f_distance, Exponent);
   return -Gain / f_distance * (fNormDistExp * fNormDistExp - fNormDistExp);
}


/****************************************/
/****************************************/

CFootBotFlocking::CFootBotFlocking() :
   m_pcWheels(NULL),
   m_pcLEDs(NULL),
   m_pcCamera(NULL),
   m_pcState(NULL) {}

/****************************************/
/****************************************/

void CFootBotFlocking::Init(TConfigurationNode& t_node) {
   /*
    * Get sensor/actuator handles
    *
    * The passed string (ex. "differential_steering") corresponds to the XML tag of the
    * device whose handle we want to have. For a list of allowed values, type at the
    * command prompt:
    *
    * $ argos3 -q actuators
    *
    * to have a list of all the possible actuators, or
    *
    * $ argos3 -q sensors
    *
    * to have a list of aCFootBotFlockingll the possible sensors.
    *
    * NOTE: ARGoS creates and initializes actuators and sensors internally, on the basis of
    *       the lists provided the configuration file at the
    *       <controllers><footbot_diffusion><actuators> and
    *       <controllers><footbot_diffusion><sensors> sections. If you forgot to
    *       list a device in the XML and then you request it here, an error occurs.
    */

    // Create the topics to publish
    stringstream stateTopic;
    stateTopic << "/" << GetId() << "/State";

    statePub = nodeHandle->advertise<State>(stateTopic.str(), 1);

    // Create the subscribers
    stringstream flockingTopic;
    // flockingTopic << "/" << GetId() << "/Flocking";
        flockingTopic << "/Flocking";


    flockingSub = nodeHandle->subscribe(flockingTopic.str(), 1, &CFootBotFlocking::flockingCallback, this);
  
   m_pcWheels = GetActuator<CCI_DifferentialSteeringActuator          >("differential_steering");
   m_pcLEDs   = GetActuator<CCI_LEDsActuator                          >("leds");
   m_pcCamera = GetSensor  <CCI_ColoredBlobOmnidirectionalCameraSensor>("colored_blob_omnidirectional_camera");
   m_pcState = GetSensor<CCI_PositioningSensor                        >("positioning");
   /*
    * Parse the config file
    */
   try {
      /* Wheel turning */
      m_sWheelTurningParams.Init(GetNode(t_node, "wheel_turning"));
      /* Flocking-related */
      m_sFlockingParams.Init(GetNode(t_node, "flocking"));
   }
   catch(CARGoSException& ex) {
      THROW_ARGOSEXCEPTION_NESTED("Error parsing the controller parameters.", ex);
   }

   /*
    * Other init stuff
    */
   /* Enable camera filtering */
   m_pcCamera->Enable();
   /* Set beacon color to all red to be visible for other robots */
   m_pcLEDs->SetSingleColor(12, CColor::RED);
}

/****************************************/
/****************************************/

void CFootBotFlocking::flockingCallback(const argos_bridge::Flocking& flocking_msg) {
  cout << GetId() << "/" <<"flockingCallback c++" << endl;
  cout << m_sFlockingParams.TargetDistance << endl;
  x = flocking_msg.x;

  y = flocking_msg.y;
  distance = flocking_msg.distance;
  // global_distance=flocking_msg.distance;
  m_sFlockingParams.TargetDistance = distance;
  // cout << global_distance << endl;
  //CFootbotFlocking::distance = flocking_msg.distance;
  cout << distance << endl;

}

void CFootBotFlocking::ControlStep() {

   /* Get readings from positioning sensor */
   const CCI_PositioningSensor::SReading& sStateReads = m_pcState->GetReading();
   State state;
   state.x = sStateReads.Position.GetX();
   state.y = sStateReads.Position.GetY();
   state.z = sStateReads.Position.GetZ();
   state.dot_x = 0;
   state.dot_y = 0;
   state.dot_z = 0;

   CQuaternion angle = sStateReads.Orientation;
   CRadians 	c_z_angle;
   CRadians	c_y_angle;
   CRadians 	c_x_angle;

   angle.ToEulerAngles(c_z_angle, c_y_angle, c_x_angle);
   x = this->x;
   y = this->y;
   CVector2 direction_1 = CVector2(x - state.x, y - state.y);
   CVector2 direction_2 = CVector2(0, 1);
   Real theta = acos((y-state.y) / (sqrt(pow(x - state.x,2) + pow(y - state.y,2))));

   statePub.publish(state);
   CVector2 c_heading(1, c_z_angle.GetValue() - theta);
  // CVector2 c_heading(this->x, this->y);
   SetWheelSpeedsFromVector(c_heading + FlockingVector());
   ros::spinOnce();
}

/****************************************/
/****************************************/

CVector2 CFootBotFlocking::VectorToLight(Real x, Real y) {
   /* Calculate a normalized vector that points to the closest light */
   CVector2 cAccum;
   cAccum = CVector2(x, y);
   // cout << cAccum << endl;
   if(cAccum.Length() > 0.0f) {
      /* Make the vector long as 1/4 of the max speed */
      cAccum.Normalize();
      cAccum *= 0.25f * m_sWheelTurningParams.MaxSpeed;
   }
   return cAccum;
}

/****************************************/
/****************************************/

CVector2 CFootBotFlocking::FlockingVector() {
   /* Get the camera readings */
   const CCI_ColoredBlobOmnidirectionalCameraSensor::SReadings& sReadings = m_pcCamera->GetReadings();
   /* Go through the camera readings to calculate the flocking interaction vector */
   cout<<m_sFlockingParams.TargetDistance<<endl;
   if(! sReadings.BlobList.empty()) {
      CVector2 cAccum;
      Real fLJ;
      size_t unBlobsSeen = 0;

      for(size_t i = 0; i < sReadings.BlobList.size(); ++i) {

         /*
          * The camera perceives the light as a yellow blob
          * The robots have their red beacon on
          * So, consider only red blobs
          * In addition: consider only the closest neighbors, to avoid
          * attraction to the farthest ones. Taking 180% of the target
          * distance is a good rule of thumb.
          */
         if(sReadings.BlobList[i]->Color == CColor::RED &&
            sReadings.BlobList[i]->Distance < m_sFlockingParams.TargetDistance * 1.80f) {
            /*
             * Take the blob distance and angle
             * With the distance, calculate the Lennard-Jones interaction force
             * Form a 2D vector with the interaction force and the angle
             * Sum such vector to the accumulator
             */
            /* Calculate LJ */
            fLJ = m_sFlockingParams.GeneralizedLennardJones(sReadings.BlobList[i]->Distance);
            /* Sum to accumulator */
            cAccum += CVector2(fLJ,
                               sReadings.BlobList[i]->Angle);
            /* Increment the blobs seen counter */
            ++unBlobsSeen;
         }
      }
      if(unBlobsSeen > 0) {
         /* Divide the accumulator by the number of blobs seen */
         cAccum /= unBlobsSeen;
         /* Clamp the length of the vector to the max speed */
         if(cAccum.Length() > m_sWheelTurningParams.MaxSpeed) {
            cAccum.Normalize();
            cAccum *= m_sWheelTurningParams.MaxSpeed;
         }
         return cAccum;
      }
      else
         return CVector2();
   }
   else {
      return CVector2();
   }
}

/****************************************/
/****************************************/

void CFootBotFlocking::SetWheelSpeedsFromVector(const CVector2& c_heading) {
   /* Get the heading angle */
   CRadians cHeadingAngle = c_heading.Angle().SignedNormalize();
   /* Get the length of the heading vector */
   Real fHeadingLength = c_heading.Length();
   /* Clamp the speed so that it's not greater than MaxSpeed */
   Real fBaseAngularWheelSpeed = Min<Real>(fHeadingLength, m_sWheelTurningParams.MaxSpeed);
   /* State transition logic */
   if(m_sWheelTurningParams.TurningMechanism == SWheelTurningParams::HARD_TURN) {
      if(Abs(cHeadingAngle) <= m_sWheelTurningParams.SoftTurnOnAngleThreshold) {
         m_sWheelTurningParams.TurningMechanism = SWheelTurningParams::SOFT_TURN;
      }
   }
   if(m_sWheelTurningParams.TurningMechanism == SWheelTurningParams::SOFT_TURN) {
      if(Abs(cHeadingAngle) > m_sWheelTurningParams.HardTurnOnAngleThreshold) {
         m_sWheelTurningParams.TurningMechanism = SWheelTurningParams::HARD_TURN;
      }
      else if(Abs(cHeadingAngle) <= m_sWheelTurningParams.NoTurnAngleThreshold) {
         m_sWheelTurningParams.TurningMechanism = SWheelTurningParams::NO_TURN;
      }
   }
   if(m_sWheelTurningParams.TurningMechanism == SWheelTurningParams::NO_TURN) {
      if(Abs(cHeadingAngle) > m_sWheelTurningParams.HardTurnOnAngleThreshold) {
         m_sWheelTurningParams.TurningMechanism = SWheelTurningParams::HARD_TURN;
      }
      else if(Abs(cHeadingAngle) > m_sWheelTurningParams.NoTurnAngleThreshold) {
         m_sWheelTurningParams.TurningMechanism = SWheelTurningParams::SOFT_TURN;
      }
   }
   /* Wheel speeds based on current turning state */
   Real fSpeed1, fSpeed2;
   switch(m_sWheelTurningParams.TurningMechanism) {
      case SWheelTurningParams::NO_TURN: {
         /* Just go straight */
         fSpeed1 = fBaseAngularWheelSpeed;
         fSpeed2 = fBaseAngularWheelSpeed;
         break;
      }
      case SWheelTurningParams::SOFT_TURN: {
         /* Both wheels go straight, but one is faster than the other */
         Real fSpeedFactor = (m_sWheelTurningParams.HardTurnOnAngleThreshold - Abs(cHeadingAngle)) / m_sWheelTurningParams.HardTurnOnAngleThreshold;
         fSpeed1 = fBaseAngularWheelSpeed - fBaseAngularWheelSpeed * (1.0 - fSpeedFactor);
         fSpeed2 = fBaseAngularWheelSpeed + fBaseAngularWheelSpeed * (1.0 - fSpeedFactor);
         break;
      }
      case SWheelTurningParams::HARD_TURN: {
         /* Opposite wheel speeds */
         fSpeed1 = -m_sWheelTurningParams.MaxSpeed;
         fSpeed2 =  m_sWheelTurningParams.MaxSpeed;
         break;
      }
   }
   /* Apply the calculated speeds to the appropriate wheels */
   Real fLeftWheelSpeed, fRightWheelSpeed;
   if(cHeadingAngle > CRadians::ZERO) {
      /* Turn Left */
      fLeftWheelSpeed  = fSpeed1;
      fRightWheelSpeed = fSpeed2;
   }
   else {
      /* Turn Right */
      fLeftWheelSpeed  = fSpeed2;
      fRightWheelSpeed = fSpeed1;
   }
   /* Finally, set the wheel speeds */
   m_pcWheels->SetLinearVelocity(fLeftWheelSpeed, fRightWheelSpeed);
}

/****************************************/
/****************************************/

/*
 * This statement notifies ARGoS of the existence of the controller.
 * It binds the class passed as first argument to the string passed as second argument.
 * The string is then usable in the XML configuration file to refer to this controller.
 * When ARGoS reads that string in the XML file, it knows which controller class to instantiate.
 * See also the XML configuration files for an example of how this is used.
 */
REGISTER_CONTROLLER(CFootBotFlocking, "argos_ros_bot_controller")
