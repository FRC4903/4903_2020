// Team 4903
// Code by Noor Nasri and Nithin Muthukumar, commented for future years
#include <frc/Joystick.h> 
#include <frc/TimedRobot.h> 
#include <iostream> 
#include "rev/CANSparkMax.h"
#include "ctre/Phoenix.h"
#include <frc/SmartDashboard/SmartDashboard.h>
#include "AHRS.h"
#include <chrono>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/trajectory/TrajectoryUtil.h>
#include <frc/geometry/Pose2d.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc/Encoder.h>
using namespace std;
using namespace frc;

class Robot : public frc::TimedRobot {
 public:
  // ================== defining public variables ==================
  // neo motors
  rev::CANSparkMax frontLeft{1, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax backLeft{2, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax frontRight{3, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax backRight{4, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax sliding{8, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax shootRight{9, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax shootLeft{12, rev::CANSparkMax::MotorType::kBrushless};
  //joystick
  frc::Joystick m_stick{0};
  frc::Joystick m_stick2{1};
  // talons
  TalonSRX intake;
  TalonSRX convey;
  TalonSRX tilt;
  TalonSRX climbLeft;
  TalonSRX climbRight;

  //encoders
  frc::Encoder tiltEncoder;

  // PID controllers for the neo motors
  rev::CANPIDController m_pidFL= frontLeft.GetPIDController();
  rev::CANPIDController m_pidBL= backLeft.GetPIDController();
  rev::CANPIDController m_pidFR= frontRight.GetPIDController();
  rev::CANPIDController m_pidBR= backRight.GetPIDController();
  rev::CANPIDController m_pidSL= shootLeft.GetPIDController();
  rev::CANPIDController m_pidSR= shootRight.GetPIDController();
  rev::CANPIDController m_pidSlid = sliding.GetPIDController();
  
  //network tables
  shared_ptr<NetworkTable> pythonTable = nt::NetworkTableInstance::GetDefault().GetTable("realTimeDB");
  shared_ptr<NetworkTable> frontLL = nt::NetworkTableInstance::GetDefault().GetTable("limelight-front");
  shared_ptr<NetworkTable> backLL = nt::NetworkTableInstance::GetDefault().GetTable("limelight-back");

  // gyro
  AHRS *ahrs;

  // constants and variables
  double kP = 6e-5, kI = 1e-6, kD = 0, kIz = 0, kFF = 0.000015, kMaxOutput = 1.0, kMinOutput = -1.0; 
  Timer *gameTimer = new Timer();
  double startX, startY;
  float accelLerp = 20;
  float oldSL = 0;
  float oldSR = 0;
  double originalAngle = -1;
  double tiltMax=67562;
  double tiltMin=-100000;
  bool reverse;
  bool isShooting = false;
  


  int wantedSpot[2] = {};
  bool pathwayExists = false;

  // Robot class intializing 
  Robot(): 
    intake(5),
    convey(6),
    tilt(7),
    climbLeft(10),
    climbRight(11),
    tiltEncoder(0,1)
  {
    try {
            /* Communicate w/navX-MXP via the MXP SPI Bus.                                       */
            /* Alternatively:  I2C::Port::kMXP, SerialPort::Port::kMXP or SerialPort::Port::kUSB */
            /* See http://navx-mxp.kauailabs.com/guidance/selecting-an-interface/ for details.   */
            ahrs = new AHRS(SPI::Port::kMXP);
        } catch (std::exception ex ) {
            std::string err_string = "Error instantiating navX-MXP:  ";
            err_string += ex.what();
        }
    ahrs->ZeroYaw();
    intake.SetNeutralMode(NeutralMode::Brake);
    intake.Set(ControlMode::PercentOutput, 0);
  }
  
  // ================== Initialization periods ==================
  void TeleopInit() override{
    
    initialize();
  }
  
  void AutonomousInit() override{
    initialize();
  }

  // ================== During Teleop period ==================
  void TeleopPeriodic() override {
    if (m_stick.GetRawButtonPressed(2)){
      isShooting = !isShooting;
    }
    if (isShooting){
      autoShoot();
      return ;

    }  
    
    reverse=m_stick.GetRawButton(5);
    updatePosition(); // update our current position

    // check if we're forced to follow a path
    if (pathwayExists){
      followPath();
    }else{
      makePath();
    }

    if (!pathwayExists){
      // arcade drive
      float j_x = m_stick.GetRawAxis(4);
      float j_y = m_stick.GetRawAxis(1);
      float mod = 0.75f; 
      moveRobot(j_x, j_y, mod);
    }
    
    // setting intake speed
    double wantIntake = (m_stick2.GetRawButton(1)-m_stick2.GetRawButton(3)) * -0.5; 
    intake.Set(ControlMode::PercentOutput, wantIntake);

    double wantConvey = -m_stick2.GetRawAxis(1)*0.4;
    convey.Set(ControlMode::PercentOutput,wantConvey);

    // setting climb
    //float wantedClimbL = (m_stick.GetRawAxis(2) - m_stick.GetRawButton(5)) * 0.3;
    //climbLeft.Set(ControlMode::PercentOutput, wantedClimbL);

    //float wantedClimbR = (m_stick.GetRawAxis(3) - m_stick.GetRawButton(6)) * 0.3;
    //climbRight.Set(ControlMode::PercentOutput, wantedClimbR * -1);

    // setting tilt
    float wantedTilt = (m_stick.GetRawButton(4) - m_stick.GetRawButton(3)) * 0.75;
    if(!(tiltEncoder.GetDistance()<tiltMin&&wantedTilt<0)||!(tiltEncoder.GetDistance()>tiltMax&&wantedTilt>0)){
      
    }
    tilt.Set(ControlMode::PercentOutput, wantedTilt);
    


    //shooters for now
    float wantedShoot = m_stick.GetRawAxis(2) * 0.75;
    shootRight.Set(wantedShoot);
    shootLeft.Set(wantedShoot*-1);
    
    cout<<tiltEncoder.GetDistance()<<endl;
  }


  // ================== Autonomous Period ==================
  void AutonomousPeriodic() override{
    //updatePosition();
    if (gameTimer ->Get() > 12.5){ // && pow(ahrs ->GetDisplacementZ(), 2) + pow(ahrs ->GetDisplacementZ(), 2)  > pow(1.5, 2)) {
      moveRobot(0, -1, -0.2f);
    }else{
      autoShoot();
    }
  }

  // ================== Functions ==================
  void initialize(){ // reset all the variables
    InitializePID(m_pidFL);
    InitializePID(m_pidBL);
    InitializePID(m_pidFR);
    InitializePID(m_pidBR);
    InitializePID(m_pidSL);
    InitializePID(m_pidSR);
    InitializePID(m_pidSlid);

    frontLeft.Set(0);
    frontRight.Set(0);
    backLeft.Set(0);
    backRight.Set(0);
    shootLeft.Set(0);
    shootRight.Set(0);
    sliding.Set(0);

    intake.Set(ControlMode::PercentOutput, 0);
    convey.Set(ControlMode::PercentOutput, 0);
    tilt.Set(ControlMode::PercentOutput, 0);
    climbLeft.Set(ControlMode::PercentOutput, 0);
    climbRight.Set(ControlMode::PercentOutput, 0);

    moveAlong = 0; 
    canMake = 0;

    gameTimer -> Start();
    gameTimer -> Reset();

    ahrs -> ResetDisplacement();
    originalAngle = ahrs -> GetAngle();

    startX = pythonTable->GetEntry("StartingX").GetDouble(0);
    startY = pythonTable->GetEntry("StartingY").GetDouble(0);
    
  } 

  void moveRobot(float j_x, float j_y, float mod){ // movement functions
    // not counting joystick if its close enough to 0
    if(j_x >= -0.05 && j_x <= 0.05) { j_x = 0; }
    if(j_y >= -0.05 && j_y <= 0.05) { j_y = 0; }

    // calculating the speed for left/right side in arcade drive
    double speedL = max(-1.0f, min(1.0f, +j_x - j_y)) * mod;
    double speedR = max(-1.0f, min(1.0f, -j_x - j_y)) * mod;

    // if we are not turning, then use custom coast. Else, use brake mode.
    speedL = (oldSL != oldSR && j_y == 0 )? speedL : oldSL * (1 - 1/accelLerp) + (1/accelLerp) * speedL; 
    speedR = (oldSL != oldSR && j_y == 0 ) ? speedR : oldSR * (1 - 1/accelLerp) + (1/accelLerp) * speedR;

    // setting neo motors to the set speed
    frontLeft.Set(speedL*-1);   
    backLeft.Set(speedL*-1);   
    frontRight.Set(speedR);   
    backRight.Set(speedR);  

    // set the old variables for the next iteration
    oldSL = speedL;
    oldSR = speedR;
  }

  void updatePosition(){ // update our position and let pygame know where we are
    nt::NetworkTableEntry offsetX = pythonTable->GetEntry("deltaX");
    nt::NetworkTableEntry offsetY = pythonTable->GetEntry("deltaY");
    nt::NetworkTableEntry offsetAngle = pythonTable->GetEntry("angle");
    offsetX.SetDouble(ahrs -> GetDisplacementX());
    offsetY.SetDouble(ahrs -> GetDisplacementZ());
    offsetAngle.SetDouble(ahrs -> GetAngle() - originalAngle);    
  }

  void makePath(){ // check if we need to make a path
    nt::NetworkTableEntry wantedX = pythonTable->GetEntry("moveX");
    nt::NetworkTableEntry wantedY = pythonTable->GetEntry("moveY");
    if (wantedX.GetDouble(0) != 0 && wantedY.GetDouble(0) != 0){
      pathwayExists = true;
      wantedSpot[0] = wantedX.GetDouble(0);
      wantedX.SetDouble(0);
      
      wantedSpot[1] = wantedY.GetDouble(0);
      wantedY.SetDouble(0);
    }
  }

  void followPath(){ // follow path thats already made
    // check if we made it
    int currentSpot[2] = {startX + ahrs -> GetDisplacementX(), startY + ahrs -> GetDisplacementZ()};
    double allowedDistance = 0.25;
    if (pow(wantedSpot[0] - currentSpot[0], 2) + pow(wantedSpot[1] - currentSpot[1], 2) < pow(allowedDistance, 2)){
      pathwayExists = false;
      moveRobot(0, 0, 0.75);
      return;
    }

    // continue moving, first get the angle
    double degreeWanted = atan((wantedSpot[1] - currentSpot[1]) / (wantedSpot[0] - currentSpot[0])) * 180.0/3.141592653589793238463;
    if (currentSpot[0] < wantedSpot[0]){ // if tan gave us the wrong angle 
      degreeWanted += 180;
    }else if (degreeWanted < 0){ // make it positive
      degreeWanted += 360;
    }
    double realAngle = originalAngle + degreeWanted > 360 ? originalAngle + degreeWanted - 360 : originalAngle + degreeWanted;

    // check if gyro is close to angle
    double currentAngle = ahrs -> GetAngle();
    if (abs(currentAngle - realAngle) > 5){
      // adjust angles
      moveRobot(((currentAngle < realAngle) ? 1 : -1), 1, 0.75);
    }else{
      // drive forward
      moveRobot(0, 1, 0.75);
    }
  }

  int moveAlong = 0; // temp solution before encoder is added
  int canMake = 0;
  void autoShoot(){ // shooting time; adjust distances; ignore the pulsing for searching; make sure tilt adjustment is good
    PIDCoefficients(m_pidSL);
    PIDCoefficients(m_pidSR);
    
    double targetOffsetAngle_Horizontal = frontLL->GetNumber("tx",0.0) - 2.48;
    double targetOffsetAngle_Vertical = frontLL->GetNumber("ty",0.0) + 9;
    double targetArea = frontLL->GetNumber("ta",0.0);
    double shootingPower = 2650;

    // shooting at all times because it takes time to get to correst speed
    m_pidSL.SetReference(shootingPower * -1, rev::ControlType::kVelocity);
    m_pidSR.SetReference(shootingPower, rev::ControlType::kVelocity);

    cout<< "Y value: " << targetOffsetAngle_Vertical << " and area of " << targetArea << " with shooter speeds of "
      << shootLeft.GetEncoder().GetVelocity() << " " << shootRight.GetEncoder().GetVelocity() << endl;

    if (moveAlong == 0 || abs(shootLeft.GetEncoder().GetVelocity() - shootingPower*-1) > 100  || abs(shootRight.GetEncoder().GetVelocity() - shootingPower) > 100 ) { // remove other velocities from the balls or wait for speed to go full
      moveAlong = 3;
      moveRobot(0, 0, 1);
      convey.Set(ControlMode::PercentOutput, 0);
      tilt.Set(ControlMode::PercentOutput, 0);
    }else if (targetArea < 0.05f && canMake < 5) { // no target and not moving onto one
      canMake = 0;
      moveRobot(1, 0, 0.4f);
      cout << "Looking for target" << endl;

    } else if (abs(targetOffsetAngle_Horizontal) < 2 || canMake > 5){
      if (abs(targetOffsetAngle_Vertical) < 3 || canMake > 5){ 
        // Let it shoot
        moveRobot(0, 0, 1);
        convey.Set(ControlMode::PercentOutput, 0.3);
        canMake++;
        if (canMake > 10){ // lose confidence again
          canMake = 4;
        }
        moveAlong--;
        cout << "Shooting" << endl;

      } else{
        // adjust the tilt
        double dir = (int)(targetOffsetAngle_Vertical > 0)*2-1;
        if (abs(targetOffsetAngle_Vertical) < 5){ // slow down the tilt when we're  close
          dir /= 2;
        }
        
        tilt.Set(ControlMode::PercentOutput, 0.65 * dir);
        moveRobot(0, 0, 1);
        canMake = 0;
        cout << "Adjusting Y" << endl;
      }
    }else{
      // turn the robot to allign with the shoot
      tilt.Set(ControlMode::PercentOutput, 0);
      moveRobot((int)(targetOffsetAngle_Horizontal > 0)*2-1, 0, (abs(targetOffsetAngle_Horizontal < 10) ? 0.15 : 0.4));
      canMake = 0;
      cout << "Adjusting X" << endl;
    }
  }

  void autoPickup(){

  }

  void autoBalance(){

  }

  void colourSpin(){

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
    m_pidController.SetSmartMotionMaxAccel(10);

    // display PID coefficients on SmartDashboard
    frc::SmartDashboard::PutNumber("P Gain", kP);
    frc::SmartDashboard::PutNumber("I Gain", kI);
    frc::SmartDashboard::PutNumber("D Gain", kD);
    frc::SmartDashboard::PutNumber("I Zone", kIz);
    frc::SmartDashboard::PutNumber("Feed Forward", kFF);
    frc::SmartDashboard::PutNumber("Max Output", kMaxOutput);
    frc::SmartDashboard::PutNumber("Min Output", kMinOutput);
  }
 
  
};

// needed code for frc
#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif

/* Autonomous things weève tested
  Network tables:
  shared_ptr<NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("dataTest");
  nt::NetworkTableEntry entryTest = table->GetEntry("X");
  nt::NetworkTableEntry entryRec = table->GetEntry("Y");
  entryTest.SetDouble(c);
  c*=1.01;
  cout<< entryRec.GetDouble(0) << endl;
  
  Colour spinning:
  rotationLength = 12.5*(wantedSpins*8 + abs(colourPositions[startingColour] - colourPositions[wantedColour])); // in inches
  if (rotationLength > 0){
    double pi = 3.141592653589793238462643383279502884197169399375105820974944592307;
    rotationLength -= 2*pi*0.02*sliding.GetEncoder().GetVelocity()/60;
    sliding.Set((rotationLength > 0) ? 0.5f : 0);
  }

  map<string, int> colourPositions = {{"Green", 1}, {"Red", 2}, {"Yellow", 3}, {"Blue", 4}};
  string startingColour = "Green";
  string wantedColour = "Blue";
  int wantedSpins = 3;
  double rotationLength = 0;
  
  Neo Encoders:
  PIDCoeffecents(m_pidFL);
  m_pidFL.SetReference(1500, rev::ControlType::kVelocity);
  cout << FrontLeft.GetEncoder().GetVelocity() << endl;   
  intake.Set(ControlMode::PercentOutput, 0.5);

  Gyro:
  cout<<"IMU_Pitch "<<ahrs->GetPitch()<<" ";
  cout<<"IMU_Yaw "<<ahrs->GetYaw()<<" ";
  cout<<"IMU_Roll "<<ahrs->GetRoll()<<endl;
*/