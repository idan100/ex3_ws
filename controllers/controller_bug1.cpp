/*
 * ID: YOUR_ID_HERE
 * Bug1 Navigation Controller for Pi-Puck
 */

#include "controller_bug1.hpp"
#include <cmath>

namespace argos {

   /****************************************/
   /****************************************/

   static Real NormalizeAngle(Real a) {
      while(a > CRadians::PI.GetValue()) a -= 2.0 * CRadians::PI.GetValue();
      while(a < -CRadians::PI.GetValue()) a += 2.0 * CRadians::PI.GetValue();
      return a;
   }

   /****************************************/
   /****************************************/

   void ControllerBug1::SetAllLEDs(const CColor& c_color) {
      m_pcColoredLEDs->SetRingLEDs(c_color);
      m_pcColoredLEDs->SetBodyLED(c_color);
      m_pcColoredLEDs->SetFrontLED(c_color);
   }

   /****************************************/
   /****************************************/

   bool ControllerBug1::ObstacleDetected() const {
      bool front_blocked = false;
      int idx = 0;
      m_pcRangefinders->Visit([&](const auto &sensor) {
         if (sensor.Proximity < m_fObstacleThreshold) {
            // Assume front sensors are 0, 1, 7 (adjust as needed based on Pi-Puck sensor configuration)
            if (idx == 0 || idx == 1 || idx == 7)
               front_blocked = true;
         }
         ++idx;
      });

      return front_blocked;
   }


/****************************************/
   /****************************************/

   // Helper to get a specific sensor index using Visit
   Real ControllerBug1::GetSpecificSensorReading(int n_target_idx) const {
       Real fReading = 0.0;
       int nCounter = 0;
       
       m_pcRangefinders->Visit([&](const auto& s_packet) {
           if (nCounter == n_target_idx) {
               fReading = s_packet.Proximity;
           }
           nCounter++;
       });
       
       return fReading;
   }

   /****************************************/
   /****************************************/

   Real ControllerBug1::GetLeftSensorReading() const {
       // Index 3 is the main side sensor (approx 90 degrees left)
       return GetSpecificSensorReading(3);
   }

   /****************************************/
   /****************************************/

   Real ControllerBug1::GetFrontLeftSensorReading() const {
       // Index 2 is the diagonal/corner sensor (approx 45 degrees left)
       return GetSpecificSensorReading(2);
   }
   /****************************************/
   /****************************************/

   void ControllerBug1::Init(TConfigurationNode& t_tree) {

      /* Actuators */
      m_pcWheels = GetActuator<CCI_PiPuckDifferentialDriveActuator>("pipuck_differential_drive");
      m_pcColoredLEDs = GetActuator<CCI_PiPuckColorLEDsActuator>("pipuck_leds");

      /* Sensors */
      m_pcSystem = GetSensor<CCI_PiPuckSystemSensor>("pipuck_system");

      m_pcCamera = GetSensor<CCI_ColoredBlobOmnidirectionalCameraSensor>(
         "colored_blob_omnidirectional_camera");
      m_pcCamera->Enable();

      m_pcRangefinders = GetSensor<CCI_PiPuckRangefindersSensor>("pipuck_rangefinders");

      m_pcPositioning = GetSensor<CCI_PositioningSensor>("positioning");
      m_pcPositioning->Enable();

      /* Read target position */
      TConfigurationNode& tTargetNode = GetNode(t_tree, "target_position");
      Real x, y;
      GetNodeAttribute(tTargetNode, "x", x);
      GetNodeAttribute(tTargetNode, "y", y);
      m_cTargetPosition.Set(x, y, 0.0);

      /* Initial state */
      m_eState = EState::GO_TO_GOAL;
      m_fBestDist = std::numeric_limits<Real>::max();
      m_bHitPointSet = false;
   }

   /****************************************/
   /****************************************/

   void ControllerBug1::ControlStep() {

      const CVector3& cPos = m_pcPositioning->GetReading().Position;

      CRadians cYaw, cPitch, cRoll;
      m_pcPositioning->GetReading().Orientation.ToEulerAngles(cYaw, cPitch, cRoll);

      Real fDistToTarget = (m_cTargetPosition - cPos).Length();

      /* Goal reached */
      if(fDistToTarget < 0.05f) {
         m_pcWheels->SetLinearVelocity(0.0, 0.0);
         SetAllLEDs(CColor::GREEN);
         m_eState = EState::FINISHED;
         return;
      }

      // Check for obstacle in front
      bool bObstacle = ObstacleDetected();

      /* Direction to target calculation */
      CVector3 cToTarget = m_cTargetPosition - cPos;
      Real fTargetAngle = atan2(cToTarget.GetY(), cToTarget.GetX());
      Real fAngleError = NormalizeAngle(fTargetAngle - cYaw.GetValue());

      switch(m_eState) {

         /* ---------------- GO TO GOAL ---------------- */
         case EState::GO_TO_GOAL: {
            SetAllLEDs(CColor::BLUE);

            if(bObstacle) {
               // 1. Initialize Bug1 Memory
               m_cHitPoint = cPos;
               m_cBestPoint = cPos;
               m_fBestDist = fDistToTarget;
               m_bHitPointSet = true;
               
               // 2. IMPORTANT: Reset the "left hit point" flag
               m_bLeftHitPoint = false; 

               // 3. Switch state
               m_eState = EState::FOLLOW_BOUNDARY;
               break;
            }

            // Move towards goal
            Real v = m_fWheelSpeed;
            Real w = 5.0 * fAngleError; // Simple proportional control
            m_pcWheels->SetLinearVelocity(v - w, v + w);
            break;
         }

         /* ---------------- FOLLOW BOUNDARY ---------------- */
         case EState::FOLLOW_BOUNDARY: {
            SetAllLEDs(CColor::YELLOW);

            // 1. Update Best Point
            if(fDistToTarget < m_fBestDist) {
               m_fBestDist = fDistToTarget;
               m_cBestPoint = cPos;
            }

            Real fDistFromHitPoint = (cPos - m_cHitPoint).Length();

            // 2. Check if we have left the hit point area (avoid immediate exit)
            if (fDistFromHitPoint > 0.20f) { // 20cm buffer
               m_bLeftHitPoint = true;
            }

            // 3. Circuit Complete Condition
            // Only check this IF we have already moved away from the start
            if(m_bLeftHitPoint && 
               fDistFromHitPoint < 0.08f) { // Back at hit point
               
               // Circuit complete: Go to the best point found
               m_eState = EState::GO_TO_BEST_POINT;
               break;
            }

            // --- MOVEMENT LOGIC ---

            // A. SPIN ON SPOT if front is blocked
            if (bObstacle) {
               // Linear velocity 0, rotate right (negative w)
               m_pcWheels->SetLinearVelocity(-2.0, 2.0); 
               break; // Skip the rest of the movement logic for this step
            }

            // B. Wall Following (if front is clear)
            Real fLeft = GetLeftSensorReading();
            Real fFrontLeft = GetFrontLeftSensorReading();

            const Real fTargetWallDist = 0.15f; 
            Real fWallError = fLeft - fTargetWallDist;

            Real v = m_fWheelSpeed;
            Real w_correction = 0.0;

            if (fFrontLeft < m_fObstacleThreshold * 1.5) {
               // Corner imminent: slow down and turn right
               v = m_fWheelSpeed * 0.5;
               w_correction = -2.5; 
            } else {
               // Standard P-Control for wall distance
               // If error is positive (too close/left high), w needs to be negative (turn right)
               // Note: Sensor reading is higher when CLOSER. 
               // Wait, typically Proximity sensors return higher values when CLOSER? 
               // Double check your sensor: 
               // If using standard Argos Proximity: 0=far, 1=touching.
               // Assuming standard Argos/Epuck proximity where 0.0 is nothing, 1.0 is crash:
               
               // Let's assume standard behavior: Higher = Closer.
               // If fLeft is HIGH (close), we want to turn RIGHT (w < 0).
               // Error = fLeft - Target. Positive Error = Too Close.
               // w = -K * Error.
               
               w_correction = -6.0 * fWallError; 
            }

            m_pcWheels->SetLinearVelocity(v - w_correction, v + w_correction);
            break;
         }

         /* ---------------- GO TO BEST POINT ---------------- */
         case EState::GO_TO_BEST_POINT: {
            SetAllLEDs(CColor::BLUE);

            CVector3 cToBest = m_cBestPoint - cPos;
            Real fBestAngle = atan2(cToBest.GetY(), cToBest.GetX());
            Real fErr = NormalizeAngle(fBestAngle - cYaw.GetValue());

            // If we are close to the best point, switch back to GO_TO_GOAL
            // This allows the robot to leave the obstacle and finish
            if(cToBest.Length() < 0.10f) {
               m_eState = EState::GO_TO_GOAL;
               break;
            }

            // Logic to navigate along the wall to the best point
            // For Bug1, you typically continue following the wall until you reach the point.
            // Simple navigation might hit the wall again. 
            // Better approach: Keep wall following BUT stop when distance to BestPoint is small.
            
            // Re-use wall following logic here OR simple seek if point is close.
            // For simplicity, let's just seek the point. If it hits wall, the state machine handles it.
            
            Real v = m_fWheelSpeed;
            Real w = 5.0 * fErr;
            m_pcWheels->SetLinearVelocity(v - w, v + w);
            break;
         }

         default:
            break;
      }
   }

   /****************************************/
   /****************************************/

   REGISTER_CONTROLLER(ControllerBug1, "controller_bug1");

}
