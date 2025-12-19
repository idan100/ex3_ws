#ifndef CONTROLLER_BUG2_HPP
#define CONTROLLER_BUG2_HPP

/*
 * ID: YOUR_ID_HERE
 * Bug2 Navigation Controller for Pi-Puck
 * Independent Implementation
 */

#include <argos3/core/control_interface/ci_controller.h>
#include <argos3/plugins/robots/pi-puck/control_interface/ci_pipuck_differential_drive_actuator.h>
#include <argos3/plugins/robots/pi-puck/control_interface/ci_pipuck_color_leds_actuator.h>
#include <argos3/plugins/robots/generic/control_interface/ci_colored_blob_omnidirectional_camera_sensor.h>
#include <argos3/plugins/robots/pi-puck/control_interface/ci_pipuck_rangefinders_sensor.h>
#include <argos3/plugins/robots/pi-puck/control_interface/ci_pipuck_system_sensor.h>
#include <argos3/plugins/robots/pi-puck/control_interface/ci_pipuck_differential_drive_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_positioning_sensor.h>

#include <limits>

namespace argos {

   class ControllerBug2 : public CCI_Controller {

   public:

      ControllerBug2() {}

      virtual ~ControllerBug2() {}

      void Init(TConfigurationNode& t_tree) override;

      void ControlStep() override;

   private:
      enum class EState {
         GO_TO_GOAL,
         FOLLOW_BOUNDARY,
         LEAVE_WALL, // Use Bug1's leave logic
         FINISHED
      };

      /* Actuators & Sensors */
      CCI_PiPuckDifferentialDriveActuator* m_pcWheels = nullptr;
      CCI_PiPuckColorLEDsActuator* m_pcColoredLEDs = nullptr;
      CCI_ColoredBlobOmnidirectionalCameraSensor* m_pcCamera = nullptr;
      CCI_PiPuckRangefindersSensor* m_pcRangefinders = nullptr;
      CCI_PiPuckSystemSensor* m_pcSystem = nullptr;
      CCI_PositioningSensor* m_pcPositioning = nullptr;

      /* Navigation Variables */
      EState m_eState;
      
      CVector3 m_cTargetPosition; 
      CVector3 m_cStartPosition;  
      CVector3 m_cHitPoint;       
      
      Real m_fDistHitToGoal;      
      bool m_bStartPosSet = false;

      // Variables for robust M-Line crossing detection
      Real m_fPrevCrossProd; 
      
      // Leave Boundary Variables (Copied from Bug1 logic)
      CVector3 m_cLeaveStart;
      int m_nLeaveClearTicks;
      Real m_fLeaveStraightDist;

      /* Parameters */
      Real m_fWheelSpeed = 5.0f;
      Real m_fObstacleThreshold = 0.08f;

      /* Helper Functions */
      void SetAllLEDs(const CColor& c_color);
      bool ObstacleDetected() const;
      Real GetSpecificSensorReading(int n_target_idx) const;
      Real GetLeftSensorReading() const;
      Real GetFrontLeftSensorReading() const;
      
      /* Bug2 Specific Math */
      Real GetCrossProduct(const CVector3& c_current_pos) const;
   };
}

#endif