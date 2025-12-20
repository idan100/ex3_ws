#ifndef CONTROLLER_BUG2_HPP
#define CONTROLLER_BUG2_HPP

/*
 * ID: 212209993, 214453821
 * Bug2 Controller Header
 */

#include <limits>
#include <cmath>
#include <argos3/core/utility/logging/argos_log.h>
#include <argos3/core/control_interface/ci_controller.h>
#include <argos3/plugins/robots/pi-puck/control_interface/ci_pipuck_differential_drive_actuator.h>
#include <argos3/plugins/robots/pi-puck/control_interface/ci_pipuck_color_leds_actuator.h>
#include <argos3/plugins/robots/generic/control_interface/ci_colored_blob_omnidirectional_camera_sensor.h>
#include <argos3/plugins/robots/pi-puck/control_interface/ci_pipuck_rangefinders_sensor.h>
#include <argos3/plugins/robots/pi-puck/control_interface/ci_pipuck_system_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_positioning_sensor.h>

namespace argos
{

   class ControllerBug2 : public CCI_Controller
   {

   public:
      /* Class constructor and destructor */
      ControllerBug2() {}
      virtual ~ControllerBug2() {}

      /* Core ARGoS functions */
      virtual void Init(TConfigurationNode &t_tree) override;
      virtual void ControlStep() override;

   private:
      /* Finite State Machine states */
      enum class EState
      {
         ALIGN = 0,
         GO_TO_GOAL,
         FOLLOW_BOUNDARY,
         FINISHED
      };

      /* Helper functions */
      void SetAllLEDs(const CColor &c_color);
      bool ObstacleDetected() const;
      Real GetSpecificSensorReading(int n_target_idx) const;
      Real GetLeftSensorReading() const;
      Real GetFrontLeftSensorReading() const;
      Real GetYaw() const;
      
      /* Bug2 specific logic */
      bool IsOnMLine(const CVector3 &c_pos);

   private:
      /* Actuators */
      CCI_PiPuckDifferentialDriveActuator *m_pcWheels = nullptr;
      CCI_PiPuckColorLEDsActuator *m_pcColoredLEDs = nullptr;

      /* Sensors */
      CCI_ColoredBlobOmnidirectionalCameraSensor *m_pcCamera = nullptr;
      CCI_PiPuckRangefindersSensor *m_pcRangefinders = nullptr;
      CCI_PiPuckSystemSensor *m_pcSystem = nullptr;
      CCI_PositioningSensor *m_pcPositioning = nullptr;

      /* State variables */
      EState m_eState;
      CVector3 m_cTargetPosition;
      CVector3 m_cHitPoint;
      CVector3 m_cBestPoint; 
      CVector3 m_cTarget;
      bool m_bTargetReached;

      /* Legacy Bug1 variables (Kept for compatibility) */
      Real m_fBestDist;
      bool m_bHitPointSet;
      bool m_bLeftHitPoint;
      bool m_bLoopCompleted;
      bool m_bLeaveAligned;
      CVector3 m_cLeaveStart;
      int m_nLeaveClearTicks;
      Real m_fLeaveStraightDist;
      Real m_fLatchedGoalYaw;
      bool m_bStraightToGoal;
      bool m_bPostLeaveAlign;
      int m_nBoundaryInitTicks;

      /* Bug2 specific logic variables */
      CVector3 m_cStartToGoalStart; 
      Real m_fHitDist;              

      /* Configuration parameters */
      Real m_fWheelSpeed = 5.0f;
      Real m_fObstacleThreshold = 0.08f;
   };

}

#endif