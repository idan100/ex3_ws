/*
 * ID: YOUR_ID_HERE
 * Bug1 Navigation Controller for Pi-Puck
 */

#include "controller_bug1.hpp"
#include <cmath>
#include <argos3/core/utility/logging/argos_log.h>

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

   void ControllerBug1::SetAllLEDs(const CColor &c_color)
   {
      m_pcColoredLEDs->SetRingLEDs(c_color);
      m_pcColoredLEDs->SetBodyLED(c_color);
      m_pcColoredLEDs->SetFrontLED(c_color);
   }

   /****************************************/
   /****************************************/

   bool ControllerBug1::ObstacleDetected() const
   {
      bool front_blocked = false;
      int idx = 0;
      m_pcRangefinders->Visit([&](const auto &sensor)
                              {
      if (sensor.Proximity < m_fObstacleThreshold) {
         if (idx == 0 || idx == 1 || idx == 7)
            front_blocked = true;
      }
      ++idx; });
      return front_blocked;
   }

   /****************************************/
   /****************************************/

   Real ControllerBug1::GetSpecificSensorReading(int n_target_idx) const
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

   /****************************************/
   /****************************************/

   Real ControllerBug1::GetLeftSensorReading() const
   {
      return GetSpecificSensorReading(3);
   }

   Real ControllerBug1::GetFrontLeftSensorReading() const
   {
      return GetSpecificSensorReading(2);
   }

   Real ControllerBug1::GetYaw() const
   {
      const CQuaternion &q = m_pcPositioning->GetReading().Orientation;
      return std::atan2(
          2.0 * (q.GetW() * q.GetZ() + q.GetX() * q.GetY()),
          1.0 - 2.0 * (q.GetY() * q.GetY() + q.GetZ() * q.GetZ()));
   }

   bool ControllerBug1::TargetVisibleAndClose() const
   {
      for (const auto &blob : m_pcCamera->GetReadings().BlobList)
      {
         if (blob->Color == CColor::CYAN &&
             blob->Distance < 0.15f)
         {
            return true;
         }
      }
      return false;
   }

   /****************************************/
   /****************************************/

   void ControllerBug1::Init(TConfigurationNode &t_tree)
   {

      m_pcWheels = GetActuator<CCI_PiPuckDifferentialDriveActuator>("pipuck_differential_drive");
      m_pcColoredLEDs = GetActuator<CCI_PiPuckColorLEDsActuator>("pipuck_leds");
      m_pcSystem = GetSensor<CCI_PiPuckSystemSensor>("pipuck_system");

      m_pcCamera = GetSensor<CCI_ColoredBlobOmnidirectionalCameraSensor>(
          "colored_blob_omnidirectional_camera");
      m_pcCamera->Enable();

      m_pcRangefinders = GetSensor<CCI_PiPuckRangefindersSensor>("pipuck_rangefinders");

      m_pcPositioning = GetSensor<CCI_PositioningSensor>("positioning");
      m_pcPositioning->Enable();

      TConfigurationNode &tTargetNode = GetNode(t_tree, "target_position");
      Real x, y;
      GetNodeAttribute(tTargetNode, "x", x);
      GetNodeAttribute(tTargetNode, "y", y);
      m_cTargetPosition.Set(x, y, 0.0);

      m_eState = EState::ALIGN;
      m_fBestDist = std::numeric_limits<Real>::max();
      m_bHitPointSet = false;
      m_bLeftHitPoint = false;
      m_bLoopCompleted = false;
      m_bLeaveAligned = false;

      m_nLeaveClearTicks = 0;
      m_cLeaveStart.Set(0, 0, 0);

      m_cTarget.Set(x, y, 0);

      LOG << "[INIT] Target=(" << x << "," << y << ")\n";
   }

   /****************************************/
   /****************************************/
   void ControllerBug1::ControlStep()
   {
      if (m_eState == EState::FINISHED)
      {
         m_pcWheels->SetLinearVelocity(0.0, 0.0);
         SetAllLEDs(CColor::GREEN);
         return;
      }

      const CVector3 &cPos = m_pcPositioning->GetReading().Position;

      Real dist_to_target = (m_cTargetPosition - cPos).Length();
      if (!m_bTargetReached && dist_to_target < 0.08f)
      {
         m_bTargetReached = true;
         m_eState = EState::FINISHED;
         m_pcWheels->SetLinearVelocity(0.0, 0.0);
         SetAllLEDs(CColor::GREEN);
         LOG << "[FINISHED] HARD STOP dist=" << dist_to_target << "\n";
         return;
      }

      bool bObstacle = ObstacleDetected();

      Real dbgDist = (m_cTargetPosition - cPos).Length();
      LOG << "[DBG] state=" << (int)m_eState
          << " dist_to_target=" << dbgDist
          << " obstacle=" << bObstacle
          << "\n";

      CRadians cYaw, cPitch, cRoll;
      m_pcPositioning->GetReading().Orientation.ToEulerAngles(cYaw, cPitch, cRoll);

      switch (m_eState)
      {
      case EState::ALIGN:
      {
         SetAllLEDs(CColor::BLUE);

         CVector3 toGoal = m_cTarget - cPos;
         Real goalAng = std::atan2(toGoal.GetY(), toGoal.GetX());
         Real yaw = GetYaw();
         Real angErr = NormalizeAngle(goalAng - yaw);

         /* rotate in place until aligned */
         if (std::fabs(angErr) > 0.08f)
         {
            Real turn = std::clamp<Real>(2.5f * angErr, -2.5f, 2.5f);

            if (turn > 0)
               m_pcWheels->SetLinearVelocity(0.0, turn);
            else
               m_pcWheels->SetLinearVelocity(-turn, 0.0);
         }
         else
         {
            /* aligned → start moving */
            m_pcWheels->SetLinearVelocity(0.0, 0.0);
            m_eState = EState::GO_TO_GOAL;
            LOG << "[ALIGN] aligned → GO_TO_GOAL\n";
         }
         break;
      }

      /* ---------------- GO TO GOAL ---------------- */
      case EState::GO_TO_GOAL:
      {
         SetAllLEDs(CColor::BLUE);

         CVector3 cToTarget = m_cTargetPosition - cPos;
         Real fDistToTarget = (m_cTargetPosition - cPos).Length();

         if (bObstacle && fDistToTarget > 0.25f)
         {
            // Hit obstacle → start boundary following
            m_cHitPoint = cPos;
            m_cBestPoint = cPos;
            m_fBestDist = fDistToTarget;
            m_bHitPointSet = true;
            m_bLeftHitPoint = false;
            m_bLoopCompleted = false;
            m_eState = EState::FOLLOW_BOUNDARY;
            LOG << "[GO_TO_GOAL] hit obstacle → FOLLOW\n";
            break;
         }

         Real v = m_fWheelSpeed;

         if (fDistToTarget < 0.4f)
            v = 2.0f;
         if (fDistToTarget < 0.25f)
            v = 1.0f;
         if (fDistToTarget < 0.15f)
            v = 0.5f;
         Real w = 0.0;

         /* STRAIGHT-LINE MODE after obstacle */
         if (m_bStraightToGoal)
         {
            Real yawErr = NormalizeAngle(m_fLatchedGoalYaw - cYaw.GetValue());

            // tiny correction only, not full steering
            w = 0.3 * yawErr;

            m_pcWheels->SetLinearVelocity(m_fWheelSpeed - w,
                                          m_fWheelSpeed + w);
            // exit straight mode if obstacle appears
            if (bObstacle)
            {
               m_bStraightToGoal = false;
            }
         }
         else
         {
            // normal go-to-goal steering
            Real fTargetAngle = atan2(cToTarget.GetY(), cToTarget.GetX());
            Real fAngleError = NormalizeAngle(fTargetAngle - cYaw.GetValue());
            w = 2.0 * fAngleError;
         }

         m_pcWheels->SetLinearVelocity(v - w, v + w);
         break;
      }

      /* ---------------- FOLLOW BOUNDARY ---------------- */
      case EState::FOLLOW_BOUNDARY:
      {
         SetAllLEDs(CColor::YELLOW);

         // Update best point while following
         Real fDistToTargetNow = (m_cTargetPosition - cPos).Length();
         if (fDistToTargetNow < m_fBestDist)
         {
            m_fBestDist = fDistToTargetNow;
            m_cBestPoint = cPos;
         }

         Real fDistFromHit = (cPos - m_cHitPoint).Length();
         if (!m_bLeftHitPoint && fDistFromHit > 0.20f)
            m_bLeftHitPoint = true;

         // Detect loop completion (returned to hit point after leaving it)
         if (m_bLeftHitPoint && !m_bLoopCompleted && fDistFromHit < 0.08f)
         {
            m_bLoopCompleted = true;
            LOG << "[FOLLOW] loop completed, continue following to best point\n";
         }

         // After loop completed, continue following until reaching best point
         if (m_bLoopCompleted)
         {
            Real fDistToBest = (cPos - m_cBestPoint).Length();
            if (fDistToBest < 0.05f)
            {
               // IMPORTANT FIX: don't jump directly to GO_TO_GOAL.
               // First detach from obstacle and move straight a bit.
               m_eState = EState::LEAVE_BOUNDARY;
               m_cLeaveStart = cPos;
               m_nLeaveClearTicks = 0;
               m_bLeaveAligned = false;
               LOG << "[FOLLOW] reached best point → LEAVE_BOUNDARY (detach)\n";
               break;
            }
            // Keep following obstacle in same direction until best reached
         }

         // Obstacle avoidance while following
         if (bObstacle)
         {
            m_pcWheels->SetLinearVelocity(-2.0, 2.0);
            break;
         }

         Real fLeft = GetLeftSensorReading();
         Real fFrontLeft = GetFrontLeftSensorReading();

         const Real fTargetWallDist = 0.15f;
         Real fWallError = fLeft - fTargetWallDist;

         Real v = m_fWheelSpeed;
         Real w_corr = 0.0;

         if (fFrontLeft < m_fObstacleThreshold * 1.5)
         {
            v = m_fWheelSpeed * 0.5;
            w_corr = -2.5;
         }
         else
         {
            w_corr = -6.0 * fWallError;
         }

         m_pcWheels->SetLinearVelocity(v - w_corr, v + w_corr);
         break;
      }

      /* ---------------- LEAVE BOUNDARY (NEW) ---------------- */
      case EState::LEAVE_BOUNDARY:
      {
         SetAllLEDs(CColor::BLUE);

         CVector3 toGoal = m_cTargetPosition - cPos;
         Real goalAng = atan2(toGoal.GetY(), toGoal.GetX());
         Real yaw = GetYaw();
         Real angErr = NormalizeAngle(goalAng - yaw);

         /* Step 1: rotate in place toward target */
         if (!m_bLeaveAligned)
         {
            if (fabs(angErr) > 0.02f)
            {
               Real turn = std::clamp<Real>(2.5f * angErr, -2.5f, 2.5f);
               if (turn > 0)
                  m_pcWheels->SetLinearVelocity(0.0, turn);
               else
                  m_pcWheels->SetLinearVelocity(-turn, 0.0);
               break;
            }

            // aligned
            m_bLeaveAligned = true;
            m_cLeaveStart = cPos;
            m_nLeaveClearTicks = 0;
            LOG << "[LEAVE_BOUNDARY] aligned to target\n";
            break;
         }

         /* Step 2: ensure front is clear */
         if (bObstacle)
         {
            m_nLeaveClearTicks = 0;
            m_pcWheels->SetLinearVelocity(0.0, 0.0);
            break;
         }
         m_nLeaveClearTicks++;

         if (m_nLeaveClearTicks < 6)
         {
            m_pcWheels->SetLinearVelocity(m_fWheelSpeed * 0.4,
                                          m_fWheelSpeed * 0.4);
            break;
         }
         /* Step 3: move straight ONLY if target still far */
         Real fStraight = (cPos - m_cLeaveStart).Length();
         Real dist = (m_cTargetPosition - cPos).Length();

         if (fStraight < m_fLeaveStraightDist && dist > 0.15f)
         {
            m_pcWheels->SetLinearVelocity(m_fWheelSpeed,
                                          m_fWheelSpeed);
            break;
         }

         toGoal = m_cTargetPosition - cPos;
         m_fLatchedGoalYaw = atan2(toGoal.GetY(), toGoal.GetX());

         m_bStraightToGoal = true;
         m_eState = EState::GO_TO_GOAL;
         LOG << "[LEAVE_BOUNDARY] straight-line mode engaged\n";

         /* Step 4: fully detached → normal navigation */
         m_eState = EState::GO_TO_GOAL;
         LOG << "[LEAVE_BOUNDARY] detached & heading to target\n";
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
