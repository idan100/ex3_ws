#ifndef CONTROLLER_BUG1_HPP
#define CONTROLLER_BUG1_HPP

/*
 * ID: YOUR_ID_HERE
 * Bug1 Navigation Controller for Pi-Puck
 */

#include <limits>

#include <argos3/core/control_interface/ci_controller.h>
#include <argos3/plugins/robots/pi-puck/control_interface/ci_pipuck_differential_drive_actuator.h>
#include <argos3/plugins/robots/pi-puck/control_interface/ci_pipuck_color_leds_actuator.h>
#include <argos3/plugins/robots/generic/control_interface/ci_colored_blob_omnidirectional_camera_sensor.h>
#include <argos3/plugins/robots/pi-puck/control_interface/ci_pipuck_rangefinders_sensor.h>
#include <argos3/plugins/robots/pi-puck/control_interface/ci_pipuck_system_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_positioning_sensor.h>

namespace argos {

class ControllerBug1 : public CCI_Controller {

public:
   ControllerBug1() {}
   virtual ~ControllerBug1() {}

   void Init(TConfigurationNode& t_tree) override;
   void ControlStep() override;

private:

   /* ---------- Bug1 states ---------- */
   enum class EState {
      GO_TO_GOAL = 0,
      FOLLOW_BOUNDARY,
      LEAVE_BOUNDARY,   // NEW: detach + go straight before GO_TO_GOAL
      FINISHED
   };

   /* ---------- Helper functions ---------- */
   void SetAllLEDs(const CColor& c_color);
   bool ObstacleDetected() const;
   Real GetSpecificSensorReading(int n_target_idx) const;
   Real GetLeftSensorReading() const;
   Real GetFrontLeftSensorReading() const;

   /* ---------- Sensors and Actuators ---------- */
   CCI_PiPuckDifferentialDriveActuator* m_pcWheels = nullptr;
   CCI_PiPuckColorLEDsActuator* m_pcColoredLEDs = nullptr;
   CCI_ColoredBlobOmnidirectionalCameraSensor* m_pcCamera = nullptr;
   CCI_PiPuckRangefindersSensor* m_pcRangefinders = nullptr;
   CCI_PiPuckSystemSensor* m_pcSystem = nullptr;
   CCI_PositioningSensor* m_pcPositioning = nullptr;

   /* ---------- Bug1 memory ---------- */
   EState   m_eState = EState::GO_TO_GOAL;
   CVector3 m_cTargetPosition;
   CVector3 m_cHitPoint;
   CVector3 m_cBestPoint;

   Real m_fBestDist = std::numeric_limits<Real>::max();
   bool m_bHitPointSet = false;
   bool m_bLeftHitPoint = false;
   bool m_bLoopCompleted = false;

   /* ---------- NEW: leave-boundary behavior ---------- */
   CVector3 m_cLeaveStart;               // where we started detaching
   int      m_nLeaveClearTicks = 0;      // need some consecutive clear ticks
   Real     m_fLeaveStraightDist = 0.20f; // how far to go straight before GO_TO_GOAL

   /* ---------- Parameters ---------- */
   Real m_fWheelSpeed = 5.0f;
   Real m_fObstacleThreshold = 0.08f;
};

}

#endif
