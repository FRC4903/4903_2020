// Team 4903
// Code by Noor Nasri, commented for future years
#include <frc/Joystick.h> 
#include <frc/TimedRobot.h> 
#include <iostream> 
#include "rev/CANSparkMax.h"
#include "ctre/Phoenix.h"
#include <frc/SmartDashboard/SmartDashboard.h>
using namespace std;

class Robot : public frc::TimedRobot {
 public: 
  // ================== defining public variables ==================
  // left neo motors
  rev::CANSparkMax frontLeft{1, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax backLeft{2, rev::CANSparkMax::MotorType::kBrushless};

  // right neo motors
  rev::CANSparkMax frontRight{3, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax backRight{4, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax shootLeft{8, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax shootRight{9, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax sliding{12, rev::CANSparkMax::MotorType::kBrushless};

  // talons
  TalonSRX intake;
  TalonSRX convey;
  TalonSRX tilt;
  TalonSRX climbLeft;
  TalonSRX climbRight;

  // PID controllers for the neo motors
  rev::CANPIDController m_pidFL= frontLeft.GetPIDController();
  rev::CANPIDController m_pidBL= backLeft.GetPIDController();
  rev::CANPIDController m_pidFR= frontRight.GetPIDController();
  rev::CANPIDController m_pidBR= backRight.GetPIDController();
  rev::CANPIDController m_pidSL= shootLeft.GetPIDController();
  rev::CANPIDController m_pidSR= shootRight.GetPIDController();
  rev::CANPIDController m_pidSlid = sliding.GetPIDController();

  // constants
  double kP = 6e-5, kI = 1e-6, kD = 0, kIz = 0, kFF = 0.000015, kMaxOutput = 1.0, kMinOutput = -1.0; 
  map<string, int> colourPositions = {{"Green", 1}, {"Red", 2}, {"Yellow", 3}, {"Blue", 4}};
  string startingColour = "Green";
  string wantedColour = "Blue";
  int wantedSpins = 3;
  int rotationLength = 12.5*(wantedSpins*8 + abs(colourPositions[startingColour] - colourPositions[wantedColour])); // in inches

  // Robot class intializing 
  Robot(): 
    intake(5),
    convey(6),
    tilt(7),
    climbLeft(10),
    climbRight(11)
  {
    intake.SetNeutralMode(NeutralMode::Brake);
    intake.Set(ControlMode::PercentOutput, 0);
  }

  // ================== During Teleop period ==================
  void TeleopInit() override{
    InitializePID(m_pidFL);
    InitializePID(m_pidBL);
    InitializePID(m_pidFR);
    InitializePID(m_pidBR);
    InitializePID(m_pidSL);
    InitializePID(m_pidSR);
    InitializePID(m_pidSlid);

  }

  float accelLerp = 20;
  float oldSL = 0;
  float oldSR = 0;
  void TeleopPeriodic() override {  
    // arcade drive
    float j_x = m_stick.GetRawAxis(1);
    float j_y = m_stick.GetRawAxis(4);
    float mod = 0.75f; 
    moveRobot(j_x, j_y, mod);

    // setting intake speed
    bool wantIntake = m_stick.GetRawButton(1);
    intake.Set(ControlMode::PercentOutput, wantIntake ? -0.75 : 0);

    bool wantConvey = m_stick.GetRawButton(2);
    convey.Set(ControlMode::PercentOutput, wantConvey ? 0.5 : 0);

    // setting climb
    float wantedClimbL = (m_stick.GetRawAxis(2) - m_stick.GetRawButton(5)) * 0.3;
    climbLeft.Set(ControlMode::PercentOutput, wantedClimbL);

    float wantedClimbR = (m_stick.GetRawAxis(3) - m_stick.GetRawButton(6)) * 0.3;
    climbRight.Set(ControlMode::PercentOutput, wantedClimbR * -1);
  }

  void AutonomousInit() override{
    InitializePID(m_pidFL);
    InitializePID(m_pidBL);
    InitializePID(m_pidFR);
    InitializePID(m_pidBR);
    InitializePID(m_pidSL);
    InitializePID(m_pidSR);
    InitializePID(m_pidSlid);
  }

  void AutonomousPeriodic() override{
    //convey.Set(ControlMode::PercentOutput, 0.5f);
    //shootLeft.Set(0.5f);
    //shootRight.Set(0.5f);
    
    // testing functions right now
    // PIDCoeffecents(m_pidFL);
    // m_pidFL.SetReference(1500, rev::ControlType::kVelocity);
    // cout << FrontLeft.GetEncoder().GetVelocity() << endl;   
    // intake.Set(ControlMode::PercentOutput, 0.5);
  }

  void moveRobot(float j_x, float j_y, float mod){
    // not counting joystick if its close enough to 0
    if(j_x >= -0.05 && j_x <= 0.05) { j_x = 0; }
    if(j_y >= -0.05 && j_y <= 0.05) { j_y = 0; }

    // calculating the speed for left/right side in arcade drive
    double speedL = max(-1.0f, min(1.0f, +j_y - j_x)) * mod;
    speedL = j_x == 0 ? speedL : oldSL * (1 - 1/accelLerp) + (1/accelLerp) * speedL; 

    double speedR = max(-1.0f, min(1.0f, -j_y - j_x)) * mod;
    speedR = j_x == 0 ? speedR : oldSR * (1 - 1/accelLerp) + (1/accelLerp) * speedR;

    oldSL = speedL;
    oldSR = speedR;

    // setting neo motors to the set speed
    frontLeft.Set(speedL*-1);   
    backLeft.Set(speedL*-1);   
    frontRight.Set(speedR);   
    backRight.Set(speedR);  
  }

  // checks for any changes in voltage stuff, just run this function and don't question it
  void PIDCoefficients(rev::CANPIDController& m_pidController){
    // read PID coefficients from SmartDashboard
    double p = frc::SmartDashboard::GetNumber("P Gain", 0);
    double i = frc::SmartDashboard::GetNumber("I Gain", 0);
    double d = frc::SmartDashboard::GetNumber("D Gain", 0);
    double iz = frc::SmartDashboard::GetNumber("I Zone", 0);
    double ff = frc::SmartDashboard::GetNumber("Feed Forward", 0);
    double max = frc::SmartDashboard::GetNumber("Max Output", 0);
    double min = frc::SmartDashboard::GetNumber("Min Output", 0);

    // if PID coefficients on SmartDashboard have changed, write new values to controller
    if((p != kP)) { m_pidController.SetP(p); kP = p; }
    if((i != kI)) { m_pidController.SetI(i); kI = i; }
    if((d != kD)) { m_pidController.SetD(d); kD = d; }
    if((iz != kIz)) { m_pidController.SetIZone(iz); kIz = iz; }
    if((ff != kFF)) { m_pidController.SetFF(ff); kFF = ff; }
    if((max != kMaxOutput) || (min != kMinOutput)) { 
      m_pidController.SetOutputRange(min, max); 
      kMinOutput = min; kMaxOutput = max; 
    }
  }

  void InitializePID(rev::CANPIDController& m_pidController){
    // set PID coefficients
    m_pidController.SetP(kP);
    m_pidController.SetI(kI);
    m_pidController.SetD(kD);
    m_pidController.SetIZone(kIz);
    m_pidController.SetFF(kFF);
    m_pidController.SetOutputRange(kMinOutput, kMaxOutput);

    // accel stuff
    m_pidController.SetSmartMotionMaxAccel(0.01);

    // display PID coefficients on SmartDashboard
    frc::SmartDashboard::PutNumber("P Gain", kP);
    frc::SmartDashboard::PutNumber("I Gain", kI);
    frc::SmartDashboard::PutNumber("D Gain", kD);
    frc::SmartDashboard::PutNumber("I Zone", kIz);
    frc::SmartDashboard::PutNumber("Feed Forward", kFF);
    frc::SmartDashboard::PutNumber("Max Output", kMaxOutput);
    frc::SmartDashboard::PutNumber("Min Output", kMinOutput);
  }
 private:
  // ================== defining private variables ==================
  frc::Joystick m_stick{0};
};

// needed code for frc
#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
