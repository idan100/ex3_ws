/*
 * ID: 212209993, 214453821
 * Bug2 Controller Implementation
 */

#include "controller_bug2.hpp"

namespace argos
{

   /****************************************/
   /****************************************/

   static Real NormalizeAngle(Real a)
   {
      while (a > CRadians::PI.GetValue())
         a -= 2.0 * CRadians::PI.GetValue();
      while (a < -CRadians::PI.GetValue())
         a += 2.0 * CRadians::PI.GetValue();
      return a;
   }

   /****************************************/
   /****************************************/

   void ControllerBug2::SetAllLEDs(const CColor &c_color)
   {
      m_pcColoredLEDs->SetRingLEDs(c_color);
      m_pcColoredLEDs->SetBodyLED(c_color);
      m_pcColoredLEDs->SetFrontLED(c_color);
   }

   /****************************************/
   /****************************************/

   bool ControllerBug2::ObstacleDetected() const
   {
      bool bBlocked = false;
      int nIdx = 0;
      m_pcRangefinders->Visit([&](const auto &s_sensor)
                              {
         /* Check only front-facing sensors (0, 1, 7) */
         if (s_sensor.Proximity < m_fObstacleThreshold) {
            if (nIdx == 0 || nIdx == 1 || nIdx == 7)
               bBlocked = true;
         }
         ++nIdx; });
      return bBlocked;
   }

   /****************************************/
   /****************************************/

   Real ControllerBug2::GetSpecificSensorReading(int n_target_idx) const
   {
      Real fReading = 0.0;
      int nCounter = 0;
      m_pcRangefinders->Visit([&](const auto &s_packet)
                              {
         if (nCounter == n_target_idx)
            fReading = s_packet.Proximity;
         nCounter++; });
      return fReading;
   }

   Real ControllerBug2::GetLeftSensorReading() const { return GetSpecificSensorReading(3); }
   Real ControllerBug2::GetFrontLeftSensorReading() const { return GetSpecificSensorReading(2); }

   /****************************************/
   /****************************************/

   Real ControllerBug2::GetYaw() const
   {
      const CQuaternion &cQ = m_pcPositioning->GetReading().Orientation;
      return std::atan2(
          2.0 * (cQ.GetW() * cQ.GetZ() + cQ.GetX() * cQ.GetY()),
          1.0 - 2.0 * (cQ.GetY() * cQ.GetY() + cQ.GetZ() * cQ.GetZ()));
   }

   /****************************************/
   /****************************************/

   bool ControllerBug2::IsOnMLine(const CVector3 &c_pos)
   {
      /* M-Line is defined by Start Position and Target Position */
      CVector3 cA = m_cStartToGoalStart;
      CVector3 cB = m_cTargetPosition;
      CVector3 cP = c_pos;

      CVector3 cAB = cB - cA;
      CVector3 cAP = cP - cA;

      /* Calculation of the distance from point P to line AB using 2D cross product area */
      Real fArea = std::fabs(cAB.GetX() * cAP.GetY() - cAB.GetY() * cAP.GetX());
      Real fBase = cAB.Length();

      if (fBase == 0) return true;

      Real fDist = fArea / fBase;
      return fDist < 0.05f; /* 5cm tolerance corridor */
   }

   /****************************************/
   /****************************************/

   void ControllerBug2::Init(TConfigurationNode &t_tree)
   {
      /* Initialize actuators and sensors */
      m_pcWheels = GetActuator<CCI_PiPuckDifferentialDriveActuator>("pipuck_differential_drive");
      m_pcColoredLEDs = GetActuator<CCI_PiPuckColorLEDsActuator>("pipuck_leds");
      m_pcSystem = GetSensor<CCI_PiPuckSystemSensor>("pipuck_system");
      m_pcCamera = GetSensor<CCI_ColoredBlobOmnidirectionalCameraSensor>("colored_blob_omnidirectional_camera");
      m_pcCamera->Enable();
      m_pcRangefinders = GetSensor<CCI_PiPuckRangefindersSensor>("pipuck_rangefinders");
      m_pcPositioning = GetSensor<CCI_PositioningSensor>("positioning");
      m_pcPositioning->Enable();

      /* Set target from XML */
      TConfigurationNode &tTargetNode = GetNode(t_tree, "target_position");
      Real fX, fY;
      GetNodeAttribute(tTargetNode, "x", fX);
      GetNodeAttribute(tTargetNode, "y", fY);
      m_cTargetPosition.Set(fX, fY, 0.0);
      m_cTarget.Set(fX, fY, 0.0);

      /* Initialize state and BUG2 parameters */
      m_eState = EState::ALIGN;
      m_fBestDist = std::numeric_limits<Real>::max();
      m_bTargetReached = false;
      
      /* Record the global M-Line start point */
      m_cStartToGoalStart = m_pcPositioning->GetReading().Position;

      LOG << "[INIT] BUG2 Target=(" << fX << "," << fY << ")\n";
   }

   /****************************************/
   /****************************************/

   void ControllerBug2::ControlStep()
   {
      /* Termination condition */
      if (m_eState == EState::FINISHED)
      {
         m_pcWheels->SetLinearVelocity(0.0, 0.0);
         SetAllLEDs(CColor::GREEN);
         return;
      }

      const CVector3 &cPos = m_pcPositioning->GetReading().Position;
      Real fDistToTarget = (m_cTargetPosition - cPos).Length();

      /* Check if target reached */
      if (!m_bTargetReached && fDistToTarget < 0.08f)
      {
         m_bTargetReached = true;
         m_eState = EState::FINISHED;
         LOG << "[FINISHED] Target reached! dist=" << fDistToTarget << "\n";
         return;
      }

      bool bObstacle = ObstacleDetected();

      switch (m_eState)
      {
      case EState::ALIGN:
      {
         SetAllLEDs(CColor::BLUE);

         CVector3 cToGoal = m_cTargetPosition - cPos;
         Real fGoalAng = std::atan2(cToGoal.GetY(), cToGoal.GetX());
         Real fYaw = GetYaw();
         Real fAngErr = NormalizeAngle(fGoalAng - fYaw);

         if (std::fabs(fAngErr) > 0.08f)
         {
            Real fTurn = std::clamp<Real>(2.5f * fAngErr, -2.5f, 2.5f);
            if (fTurn > 0) m_pcWheels->SetLinearVelocity(0.0, fTurn);
            else m_pcWheels->SetLinearVelocity(-fTurn, 0.0);
         }
         else
         {
            m_pcWheels->SetLinearVelocity(0.0, 0.0);
            m_eState = EState::GO_TO_GOAL;
            LOG << "[ALIGN] → GO_TO_GOAL\n";
         }
         break;
      }

      case EState::GO_TO_GOAL:
      {
         SetAllLEDs(CColor::BLUE);

         if (bObstacle && fDistToTarget > 0.25f)
         {
            /* Obstacle hit - Switch to BUG2 Follow */
            m_cHitPoint = cPos;
            m_fHitDist = fDistToTarget; // Save distance for leave condition
            m_eState = EState::FOLLOW_BOUNDARY;
            LOG << "[GO_TO_GOAL] Obstacle hit → FOLLOW_BOUNDARY\n";
            break;
         }

         /* Simple P-control for heading and speed reduction near target */
         Real fV = m_fWheelSpeed;
         if (fDistToTarget < 0.4f) fV = 2.0f;
         if (fDistToTarget < 0.25f) fV = 1.0f;
         if (fDistToTarget < 0.15f) fV = 0.5f;

         CVector3 cToTarget = m_cTargetPosition - cPos;
         CRadians cYaw, cPitch, cRoll;
         m_pcPositioning->GetReading().Orientation.ToEulerAngles(cYaw, cPitch, cRoll);
         
         Real fTargetAngle = atan2(cToTarget.GetY(), cToTarget.GetX());
         Real fAngleError = NormalizeAngle(fTargetAngle - cYaw.GetValue());
         Real fW = 2.0 * fAngleError;

         m_pcWheels->SetLinearVelocity(fV - fW, fV + fW);
         break;
      }

      case EState::FOLLOW_BOUNDARY:
      {
         SetAllLEDs(CColor::YELLOW);

         /* BUG2 LEAVE CONDITION:
          * 1. On M-Line
          * 2. Distance to target is strictly less than at Hit Point
          * 3. Current path to goal is clear
          */
         if (IsOnMLine(cPos) && fDistToTarget < m_fHitDist && !bObstacle)
         {
            LOG << "[FOLLOW] M-Line reached & clear → ALIGN to goal\n";
            m_eState = EState::ALIGN;
            break;
         }

         /* Obstacle Avoidance / Wall Following */
         if (bObstacle)
         {
            m_pcWheels->SetLinearVelocity(-1.0, 1.0);
            break;
         }

         Real fLeft = GetLeftSensorReading();
         Real fFrontLeft = GetFrontLeftSensorReading();

         const Real fTargetWallDist = 0.15f;
         Real fWallError = fLeft - fTargetWallDist;

         Real fV = m_fWheelSpeed;
         Real fWCorr = 0.0;

         /* If wall is too close in front, turn sharply */
         if (fFrontLeft < m_fObstacleThreshold * 1.5)
         {
            fV = m_fWheelSpeed * 0.5;
            fWCorr = -2.5;
         }
         else
         {
            /* PID-style correction to maintain distance from left wall */
            fWCorr = -6.0 * fWallError;
         }

         m_pcWheels->SetLinearVelocity(fV - fWCorr, fV + fWCorr);
         break;
      }

      default:
         break;
      }
   }

   /****************************************/
   /****************************************/

   REGISTER_CONTROLLER(ControllerBug2, "controller_bug2");
}