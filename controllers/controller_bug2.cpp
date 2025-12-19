/*
 * ID: YOUR_ID_HERE
 * Bug2 Navigation Controller for Pi-Puck
 * Independent Implementation with Leave Boundary Logic from Bug1
 */

#include "controller_bug2.hpp"
#include <cmath>
#include <argos3/core/utility/logging/argos_log.h>

namespace argos {
    
    static Real NormalizeAngle(Real a) {
       while(a > CRadians::PI.GetValue()) a -= 2.0 * CRadians::PI.GetValue();
       while(a < -CRadians::PI.GetValue()) a += 2.0 * CRadians::PI.GetValue();
       return a;
    }

    void ControllerBug2::SetAllLEDs(const CColor& c_color) {
       m_pcColoredLEDs->SetRingLEDs(c_color);
       m_pcColoredLEDs->SetBodyLED(c_color);
       m_pcColoredLEDs->SetFrontLED(c_color);
    }
    
    bool ControllerBug2::ObstacleDetected() const {
       bool front_blocked = false;
       int idx = 0;
       m_pcRangefinders->Visit([&](const auto &sensor) {
          if (sensor.Proximity < m_fObstacleThreshold) {
             if (idx == 0 || idx == 1 || idx == 7)
                front_blocked = true;
          }
          ++idx;
       });
       return front_blocked;
    }

    Real ControllerBug2::GetSpecificSensorReading(int n_target_idx) const {
       Real fReading = 0.0;
       int nCounter = 0;
       m_pcRangefinders->Visit([&](const auto& s_packet) {
          if (nCounter == n_target_idx)
             fReading = s_packet.Proximity;
          nCounter++;
       });
       return fReading;
    }

    Real ControllerBug2::GetLeftSensorReading() const {
       return GetSpecificSensorReading(3);
    }

    Real ControllerBug2::GetFrontLeftSensorReading() const {
       return GetSpecificSensorReading(2);
    }

    Real ControllerBug2::GetCrossProduct(const CVector3& c_current_pos) const {
        CVector3 cMline = m_cTargetPosition - m_cStartPosition;
        CVector3 cToRobot = c_current_pos - m_cStartPosition;
        return cMline.GetX() * cToRobot.GetY() - cMline.GetY() * cToRobot.GetX();
    }

    void ControllerBug2::Init(TConfigurationNode& t_tree) {
        m_pcWheels = GetActuator<CCI_PiPuckDifferentialDriveActuator>("pipuck_differential_drive");
        m_pcColoredLEDs = GetActuator<CCI_PiPuckColorLEDsActuator>("pipuck_leds");
        m_pcSystem = GetSensor<CCI_PiPuckSystemSensor>("pipuck_system");
        
        m_pcCamera = GetSensor<CCI_ColoredBlobOmnidirectionalCameraSensor>("colored_blob_omnidirectional_camera");
        m_pcCamera->Enable();
        
        m_pcRangefinders = GetSensor<CCI_PiPuckRangefindersSensor>("pipuck_rangefinders");
        
        m_pcPositioning = GetSensor<CCI_PositioningSensor>("positioning");
        m_pcPositioning->Enable();
        
        TConfigurationNode& tTargetNode = GetNode(t_tree, "target_position");
        Real targetX, targetY;
        GetNodeAttribute(tTargetNode, "x", targetX);
        GetNodeAttribute(tTargetNode, "y", targetY);
        m_cTargetPosition.Set(targetX, targetY, 0.0);
        
        m_eState = EState::GO_TO_GOAL;
        m_bStartPosSet = false;
        m_fPrevCrossProd = 0.0;
        
        // Initialize Leave Boundary variables
        m_nLeaveClearTicks = 0;
        m_cLeaveStart.Set(0,0,0);
        m_fLeaveStraightDist = 0.15f; // Short distance to clear obstacle
        
        LOG << "[BUG2] Init. Target: " << m_cTargetPosition << std::endl;
    }
    
    void ControllerBug2::ControlStep() {
        const auto& sReading = m_pcPositioning->GetReading();
        CVector3 cPos = sReading.Position;
        CRadians cYaw, cPitch, cRoll;
        sReading.Orientation.ToEulerAngles(cYaw, cPitch, cRoll);

        if (!m_bStartPosSet) {
            m_cStartPosition = cPos;
            m_bStartPosSet = true;
            m_fPrevCrossProd = GetCrossProduct(cPos); 
            LOG << "[BUG2] Start Pos: " << m_cStartPosition << std::endl;
        }

        Real fDistToTarget = (m_cTargetPosition - cPos).Length();

        if(fDistToTarget < 0.05f) {
            m_pcWheels->SetLinearVelocity(0.0, 0.0);
            SetAllLEDs(CColor::GREEN); 
            m_eState = EState::FINISHED;
            return;
        }

        switch(m_eState) {
            
            /* --- GO TO GOAL --- */
            case EState::GO_TO_GOAL: {
                SetAllLEDs(CColor::BLUE);

                if (ObstacleDetected()) {
                    m_cHitPoint = cPos;
                    m_fDistHitToGoal = fDistToTarget;
                    m_eState = EState::FOLLOW_BOUNDARY;
                    m_fPrevCrossProd = GetCrossProduct(cPos); 
                    LOG << "[BUG2] Hit! Dist: " << m_fDistHitToGoal << std::endl;
                } else {
                    CVector3 cToTarget = m_cTargetPosition - cPos;
                    Real fTargetAngle = atan2(cToTarget.GetY(), cToTarget.GetX());
                    Real fAngleError = NormalizeAngle(fTargetAngle - cYaw.GetValue());

                    Real v = m_fWheelSpeed;
                    Real w = 4.0 * fAngleError;

                    if (std::fabs(fAngleError) > 0.3f) v = 0.5;
                    else if (std::fabs(fAngleError) > 0.1f) v = 1.5;

                    m_pcWheels->SetLinearVelocity(v - w, v + w);
                }
                break;
            }

            /* --- FOLLOW BOUNDARY --- */
            case EState::FOLLOW_BOUNDARY: {
                SetAllLEDs(CColor::YELLOW);

                Real fCurrCrossProd = GetCrossProduct(cPos);
                
                // M-Line Check
                bool bCrossedLine = (m_fPrevCrossProd * fCurrCrossProd <= 0.0) || 
                                    (std::abs(fCurrCrossProd) < 0.05);

                // Use slightly strict buffer (0.01) to ensure progress
                if (bCrossedLine && (fDistToTarget < m_fDistHitToGoal - 0.01f)) {
                     // SWITCH TO LEAVE_BOUNDARY logic
                     m_eState = EState::LEAVE_WALL;
                     m_cLeaveStart = cPos;
                     m_nLeaveClearTicks = 0;
                     LOG << "[BUG2] Leave Wall Triggered." << std::endl;
                     break; 
                }
                
                m_fPrevCrossProd = fCurrCrossProd;

                // Wall Follow Logic (Left)
                Real fLeft = GetLeftSensorReading();
                Real fFrontLeft = GetFrontLeftSensorReading();
                
                if(ObstacleDetected()) { 
                   m_pcWheels->SetLinearVelocity(-2.0, 2.0); 
                } 
                else {
                   const Real fTargetWallDist = 0.15f;
                   Real fWallError = fLeft - fTargetWallDist;

                   Real v = m_fWheelSpeed;
                   Real w_corr = 0.0;

                   if (fFrontLeft < m_fObstacleThreshold * 1.5) {
                      v = m_fWheelSpeed * 0.5;
                      w_corr = -2.5; 
                   } else {
                      w_corr = -6.0 * fWallError; 
                   }
                   m_pcWheels->SetLinearVelocity(v - w_corr, v + w_corr);
                }
                break;
            }

            /* --- LEAVE WALL (From Bug1 Logic) --- */
            case EState::LEAVE_WALL: {
                SetAllLEDs(CColor::BLUE); // Committed to goal

                // 1. Ensure front is clear (wait until we face away from wall)
                if(ObstacleDetected()) {
                    // Still blocked? Turn RIGHT (away from left wall)
                    m_nLeaveClearTicks = 0;
                    m_pcWheels->SetLinearVelocity(2.0, -2.0);
                    break;
                } else {
                    m_nLeaveClearTicks++;
                }

                // Require stability (wait for a few clear ticks)
                if(m_nLeaveClearTicks < 6) {
                    m_pcWheels->SetLinearVelocity(m_fWheelSpeed * 0.6, m_fWheelSpeed * 0.6);
                    break;
                }

                // 2. Drive straight away from obstacle
                Real fStraight = (cPos - m_cLeaveStart).Length();
                if(fStraight < m_fLeaveStraightDist) {
                    m_pcWheels->SetLinearVelocity(m_fWheelSpeed, m_fWheelSpeed);
                    break;
                }

                // 3. Resume Goal Seeking
                m_eState = EState::GO_TO_GOAL;
                LOG << "[BUG2] Detached safely. Resuming Goal." << std::endl;
                break;
            }

            case EState::FINISHED:
                m_pcWheels->SetLinearVelocity(0.0, 0.0);
                SetAllLEDs(CColor::GREEN);
                break;
        }
    }
    
    REGISTER_CONTROLLER(ControllerBug2, "controller_bug2");
}