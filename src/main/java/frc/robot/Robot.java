/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;

import edu.wpi.first.wpilibj.Joystick;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;


//import com.ctre.phoenix.motorcontrol.ControlMode;
//import com.ctre.phoenix.motorcontrol.DemandType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj.Solenoid;

//import edu.wpi.first.wpilibj.I2C;
//import edu.wpi.first.wpilibj.util.Color;
//import com.revrobotics.ColorSensorV3;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends TimedRobot {

  private WPI_TalonFX leftTop = new WPI_TalonFX(0);
  private WPI_TalonFX leftMiddle = new WPI_TalonFX(1);
  private WPI_TalonFX leftBottom = new WPI_TalonFX(2);

  private WPI_TalonFX rightTop = new WPI_TalonFX(3);
  private WPI_TalonFX rightMiddle = new WPI_TalonFX(4);
  private WPI_TalonFX rightBottom = new WPI_TalonFX(5);

  private WPI_TalonFX shooterFalcon1 = new WPI_TalonFX(13);
  private WPI_TalonFX shooterFalcon2 = new WPI_TalonFX(14);

  private WPI_TalonSRX feederMotor1 = new WPI_TalonSRX(8);
  private WPI_TalonSRX feederMotor2 = new WPI_TalonSRX(9);

  private WPI_TalonSRX intakeMotor = new WPI_TalonSRX(10);
  
  private final Joystick m_stick = new Joystick(0);
  //public static Solenoid solenoid1 = new Solenoid(7);
  private double leftStick = 0.0;
  private double rightStick = 0.0;

  private double leftTopVelocity = 0.0;
  private double leftTopDistance = 0.0;
  private double rightTopDistance = 0.0;
  private double rightTopVelocity = 0.0;

  private double leftMiddleVelocity = 0.0;
  private double leftMiddleDistance = 0.0;
  private double rightMiddleDistance = 0.0;
  private double rightMiddleVelocity = 0.0;

  private double leftBottomVelocity = 0.0;
  private double leftBottomDistance = 0.0;
  private double rightBottomDistance = 0.0;
  private double rightBottomVelocity = 0.0;

  private double targetFalconRPM = 0.0;
  private double targetRPM_UnitsPer100ms;
  private double shooterFalcon1Distance = 0.0;
  private double shooterFalcon1Velocity = 0.0;
  private double shooter1RPM;
  private double shooter2RPM;
  private double shooterFalcon2Distance = 0.0;
  private double shooterFalcon2Velocity = 0.0;

  private double flipStick=-1.0; // +1 or -1
  private double scaleStick=1.0; // between 0.0 and 1.0

  private double intakeSpeed = 0.8;

  private AHRS ahrs = new AHRS(SPI.Port.kMXP);


  private double shooterKF = 0.0;
  private double shooterKP = 0.0;
  private double shooterKI = 0.0;
  private double shooterTargetRPM = 0.0;
  private double shooterTargetRPM_Unitsper100ms = 0.0;
  private ShuffleboardTab tab = Shuffleboard.getTab("drive");
  
  private NetworkTableEntry shooterTargetRPMNT = tab.add("Target RPM",500).getEntry();
  private NetworkTableEntry shooterKPNT = tab.add("KP",.0015).getEntry();
  private NetworkTableEntry shooterKFNT = tab.add("KF",.047).getEntry();
  private NetworkTableEntry shooterKINT = tab.add("KI",.00002 ).getEntry();
  
  // shooterKF = 0.047;
  // shooterKP = 0.0015;
  // shooterKI = 0.00002;
  //private final I2C.Port i2cPort = I2C.Port.kOnboard;
  //private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {

    // Initialize Falcons to Factory Default
    shooterFalcon1.configFactoryDefault();
    shooterFalcon2.configFactoryDefault();
    // Initialize feeder motor controllers to factory default
    feederMotor1.configFactoryDefault();
    feederMotor2.configFactoryDefault();

    intakeMotor.configFactoryDefault();
    
    leftTop.configFactoryDefault();
    leftMiddle.configFactoryDefault();
    leftBottom.configFactoryDefault();

    rightTop.configFactoryDefault();
    rightBottom.configFactoryDefault();
    rightMiddle.configFactoryDefault();

    // Set motor2 to follow motor1
    shooterFalcon2.follow(shooterFalcon1);

    // Set sense of master motor output, and follower motor to be the opposite
    shooterFalcon1.setInverted(true);
    shooterFalcon2.setInverted(InvertType.OpposeMaster);
    //shooterFalcon2.setInverted(false);

    // Set sense of master motor output, and follower motor to be the opposite
    feederMotor1.setInverted(true);
    feederMotor2.follow(feederMotor1);
    feederMotor2.setInverted(InvertType.FollowMaster);

    // Set sense of master motor output, and follower motor to be the opposite
    intakeMotor.setInverted(false);

    // Set voltage compensation to keep things consistent as battery discharges
    shooterFalcon1.configVoltageCompSaturation(11);
    shooterFalcon1.enableVoltageCompensation(false);
    shooterFalcon2.configVoltageCompSaturation(11);
    shooterFalcon2.enableVoltageCompensation(false);
    
    leftTop.configVoltageCompSaturation(11);
    leftTop.enableVoltageCompensation(true);
    leftTop.setInverted(false);
    leftMiddle.configVoltageCompSaturation(11);
    leftMiddle.enableVoltageCompensation(true);    
    leftMiddle.setInverted(false);
    leftBottom.configVoltageCompSaturation(11);
    leftBottom.enableVoltageCompensation(true);
    leftBottom.setInverted(false);

    rightTop.configVoltageCompSaturation(11);
    rightTop.enableVoltageCompensation(true);
    rightTop.setInverted(true);
    rightMiddle.configVoltageCompSaturation(11);
    rightMiddle.enableVoltageCompensation(true);
    rightMiddle.setInverted(true);
    rightBottom.configVoltageCompSaturation(11);
    rightBottom.enableVoltageCompensation(true);
    rightBottom.setInverted(true);

    // Should we try limiting current if we get stalled (a ball gets stuck in shooter?)
    // These limits aren't found in current CTRE lib -- see new names for 2020 
    //shooterFalcon1.configPeakCurrentLimit(??); // Example used 30 Amps
    //shooterFalcon1.configPeakCurrentDuration(???); // Example used 100 ms
    //shooterFalcon1.configPeakContinuousCurrentLimit(??); // Example used 20 Amps
    //shooterFalcon1.enableCurrentLimit(true); // Turn on limiting

    // Setup sensors, set current positoin to 0.0 and choose phase if needed.
    shooterFalcon1.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,0,0);
    shooterFalcon1.setSelectedSensorPosition(0,0,0);
    shooterFalcon1.setSensorPhase(false);
    shooterFalcon2.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,0,0);
    shooterFalcon2.setSelectedSensorPosition(0,0,0);
    shooterFalcon2.setSensorPhase(false);

    leftTop.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,0,0);
    leftTop.setSelectedSensorPosition(0,0,0);
    leftTop.setSensorPhase(false);
    leftMiddle.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,0,0);
    leftMiddle.setSelectedSensorPosition(0,0,0);
    leftMiddle.setSensorPhase(false);
    leftBottom.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,0,0);
    leftBottom.setSelectedSensorPosition(0,0,0);
    leftBottom.setSensorPhase(false);

    rightTop.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,0,0);
    rightTop.setSelectedSensorPosition(0,0,0);
    rightTop.setSensorPhase(false);
    rightMiddle.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,0,0);
    rightMiddle.setSelectedSensorPosition(0,0,0);
    rightMiddle.setSensorPhase(false);
    rightBottom.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,0,0);
    rightBottom.setSelectedSensorPosition(0,0,0);
    rightBottom.setSensorPhase(false);


    // Config the peak and nominal outputs
    //shooterFalcon1.configNominalOutputForward(0,0);
    //shooterFalcon1.configNominalOutputReverse(0,0);
    //shooterFalcon1.configPeakOutputForward(1,0);
    //shooterFalcon1.configPeakOutputReverse(-1,0);

    // Setup on Velocity closed loop gains for the master (in slot 0)
    //shooterFalcon1.config_kF(0,0.051,0);
    //shooterFalcon1.config_kP(0,0.04,0);
    //shooterFalcon1.config_kI(0,0.0,0);
    //shooterFalcon1.config_kD(0,1.0,0);
    
    // Set the network table to update faster
    NetworkTableInstance.getDefault().setUpdateRate(0.02);
  }

  /**
   * This function is run once each time the robot enters autonomous mode.
   */
  @Override
  public void autonomousInit() {
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
  }

  /**
   * This function is called once each time the robot enters teleoperated mode.
   */
  @Override
  public void teleopInit() {
  }

  /**
   * This function is called periodically during teleoperated mode.
   */
  @Override
  public void teleopPeriodic() {
    leftStick = flipStick*scaleStick*m_stick.getRawAxis(Joystick.AxisType.kY.value);
    rightStick = flipStick*scaleStick*m_stick.getRawAxis(Joystick.AxisType.kTwist.value);
    SmartDashboard.putNumber("left stick:", leftStick);
    SmartDashboard.putNumber("right stick:", rightStick);

    shooterTargetRPM = shooterTargetRPMNT.getDouble(2500);
    //shooterKF = SmartDashboard.getNumber("KF", 0.046);
    //shooterKP = SmartDashboard.getNumber("KP", 0.01);
    SmartDashboard.putNumber("Shooter KF Feedback", shooterKF);
    SmartDashboard.putNumber("Shooter KP Feedback", shooterKP);
    SmartDashboard.putNumber("Shooter RPM Feedback", shooterTargetRPM);

    shooterTargetRPM_Unitsper100ms = shooterTargetRPM / 600 * 2048;

    targetRPM_UnitsPer100ms = targetFalconRPM * 2048.0/600.0;

    // Read the B button, and if pressed run intake motor
    boolean testIntakePressed = m_stick.getRawButton(3);
    if (testIntakePressed) {
      intakeMotor.set(ControlMode.PercentOutput, intakeSpeed);
    } else {
      intakeMotor.set(ControlMode.PercentOutput, 0.0);
    }
    // Read right bumper, if pressed pass left stick to shooter talons, and don't drive
    // the drive train.
    boolean test_falcon_pressed = m_stick.getRawButton(6);
    if (test_falcon_pressed) {
       shooterFalcon1.set(ControlMode.Velocity, shooterTargetRPM_Unitsper100ms);
       //shooterFalcon2.set(ControlMode.PercentOutput, leftStick);
       feederMotor1.set(ControlMode.PercentOutput, rightStick);
       // shooterFalcon2, will just follow falcon1 with the opposite direction. 

       // Put some code to run shooter to a set velocity later.
      //targetRPM_UnitsPer100ms = targetFalconRPM * 2048.0/600.0;
      //shooterFalcon1.set(ControlMode.Velocity, targetRPM_UnitsPer100ms); 
       leftTop.set(ControlMode.PercentOutput, 0.0);
       leftMiddle.set(ControlMode.PercentOutput, 0.0);
       leftBottom.set(ControlMode.PercentOutput, 0.0);
       rightTop.set(ControlMode.PercentOutput, 0.0);
       rightMiddle.set(ControlMode.PercentOutput, 0.0);
       rightBottom.set(ControlMode.PercentOutput, 0.0);
    } else { // button not pressed, use thumbsticks for drivetrain
      shooterFalcon1.set(ControlMode.PercentOutput, 0.0);
      //shooterFalcon2.set(ControlMode.PercentOutput, leftStick);
      feederMotor1.set(ControlMode.PercentOutput, 0.0);

      leftTop.set(ControlMode.PercentOutput, leftStick);
      leftMiddle.set(ControlMode.PercentOutput, leftStick);
      leftBottom.set(ControlMode.PercentOutput, leftStick);
      rightTop.set(ControlMode.PercentOutput, rightStick);
      rightMiddle.set(ControlMode.PercentOutput, rightStick);
      rightBottom.set(ControlMode.PercentOutput, rightStick);
    }

    // Read Talon Sensors and display values
    readTalonsAndShowValues();

    // Read left bumper, if pressed reset NavX yaw value.
    boolean zero_yaw_pressed = m_stick.getRawButton(5);
    if (zero_yaw_pressed) {
      ahrs.zeroYaw();
    }
    // Read NavX and display values
    readNavxAndShowValues();

  }

   /**
   * This function is called periodically during disabled mode.
   */
  @Override
  public void disabledPeriodic() {
    leftStick = flipStick*scaleStick*m_stick.getRawAxis(Joystick.AxisType.kY.value);
    rightStick = flipStick*scaleStick*m_stick.getRawAxis(Joystick.AxisType.kTwist.value);
    SmartDashboard.putNumber("left stick:", leftStick);
    SmartDashboard.putNumber("right stick:", rightStick);

    shooterTargetRPM = shooterTargetRPMNT.getDouble(2500);
    //shooterKF = SmartDashboard.getNumber("KF", 0.05);
  
    shooterKF = shooterKFNT.getDouble(.047);
    shooterKP = shooterKPNT.getDouble(.0015);
    shooterKI = shooterKINT.getDouble(.00002);
    //shooterKP = SmartDashboard.getNumber("KP", 0.00058);
    SmartDashboard.putNumber("Shooter KF Feedback", shooterKF);
    SmartDashboard.putNumber("Shooter KP Feedback", shooterKP);
    SmartDashboard.putNumber("Shooter RPM Feedback", shooterTargetRPM);

    //Config the peak and nominal outputs
    shooterFalcon1.configNominalOutputForward(0,0);
    shooterFalcon1.configNominalOutputReverse(0,0);
    shooterFalcon1.configPeakOutputForward(1,0);
    shooterFalcon1.configPeakOutputReverse(-1,0);

   // Setup on Velocity closed loop gains for the master (in slot 0)
    shooterFalcon1.config_kF(0,shooterKF,0);
    shooterFalcon1.config_kP(0,shooterKP,0);
    shooterFalcon1.config_kI(0,shooterKI,0);
    shooterFalcon1.config_kD(0,0.0,0);

    if (leftStick < -0.5) {
      targetFalconRPM = 500.0;
    } else {
      targetFalconRPM = 0.0;
    }

    // Read Talon Sensors and display values
    readTalonsAndShowValues();

    // Read left bumper, if pressed reset NavX yaw value.
    boolean zero_yaw_pressed = m_stick.getRawButton(5);
    if (zero_yaw_pressed) {
      ahrs.zeroYaw();
    }
    // Read NavX and display values
    readNavxAndShowValues();
    
}

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {

  }

  public void readTalonsAndShowValues() {
    shooterFalcon1Distance = shooterFalcon1.getSelectedSensorPosition(0);
    shooterFalcon1Velocity = shooterFalcon1.getSelectedSensorVelocity(0);
    shooterFalcon2Distance = shooterFalcon2.getSelectedSensorPosition(0);
    shooterFalcon2Velocity = shooterFalcon2.getSelectedSensorVelocity(0);
    shooter1RPM = shooterFalcon1Velocity * 600.0 / 2048.0; 
    shooter2RPM = shooterFalcon2Velocity * 600.0 / 2048.0;
    SmartDashboard.putNumber("shooter1RPM:", shooter1RPM);
    SmartDashboard.putNumber("shooter2RPM:", shooter2RPM);
    SmartDashboard.putNumber("targetRPM:", targetFalconRPM);
    SmartDashboard.putNumber("targetUp100ms:", targetRPM_UnitsPer100ms);

    // leftTopVelocity = leftTop.getSelectedSensorVelocity(0);
    // leftMiddleVelocity = leftMiddle.getSelectedSensorVelocity(0);
    // leftBottomVelocity = leftBottom.getSelectedSensorVelocity(0);
    // SmartDashboard.putNumber("leftTopVel:", leftTopVelocity);
    // SmartDashboard.putNumber("leftMidVel:", leftMiddleVelocity);
    // SmartDashboard.putNumber("leftBotVel:", leftBottomVelocity);
    // rightTopVelocity = rightTop.getSelectedSensorVelocity(0);
    // rightMiddleVelocity = rightMiddle.getSelectedSensorVelocity(0);
    // rightBottomVelocity = rightBottom.getSelectedSensorVelocity(0);
    // SmartDashboard.putNumber("rightTopVel:", rightTopVelocity);
    // SmartDashboard.putNumber("rightMidVel:", rightMiddleVelocity);
    // SmartDashboard.putNumber("rightBotVel:", rightBottomVelocity);

    // leftTopDistance = leftTop.getSelectedSensorPosition(0);
    // leftMiddleDistance = leftMiddle.getSelectedSensorPosition(0);
    // leftBottomDistance = leftBottom.getSelectedSensorPosition(0);
    // SmartDashboard.putNumber("leftTopPos:", leftTopDistance);
    // SmartDashboard.putNumber("leftMidPos:", leftMiddleDistance);
    // SmartDashboard.putNumber("leftBotPos:", leftBottomDistance);
    // rightTopDistance = rightTop.getSelectedSensorPosition(0);
    // rightMiddleDistance = rightMiddle.getSelectedSensorPosition(0);
    // rightBottomDistance = rightBottom.getSelectedSensorPosition(0);
    // SmartDashboard.putNumber("rightTopPos:", rightTopDistance);
    // SmartDashboard.putNumber("rightMidPos:", rightMiddleDistance);
    // SmartDashboard.putNumber("rightBotPos%:", rightBottomDistance);

    //SmartDashboard.putNumber("left distance:", leftDistance);
    //SmartDashboard.putNumber("left velocity:", leftVelocity);
    //SmartDashboard.putNumber("right distance:", rightDistance);
    //SmartDashboard.putNumber("right velocity:", rightVelocity);
    //SmartDashboard.putNumber("shooterFalcon1 distance:", shooterFalcon1Distance);
    SmartDashboard.putNumber("shooterFalcon1 velocity:", shooterFalcon1Velocity);
    //SmartDashboard.putNumber("shooterFalcon2 distance:", shooterFalcon2Distance);
    SmartDashboard.putNumber("shooterFalcon2 velocity:", shooterFalcon2Velocity);
  }

  public void readNavxAndShowValues() {
    /* Display 6-axis Processed Angle Data */
    //SmartDashboard.putBoolean("IMU_Connected", ahrs.isConnected());
    //SmartDashboard.putBoolean("IMU_IsCalibrating", ahrs.isCalibrating());
    //SmartDashboard.putNumber("IMU_Yaw", ahrs.getYaw());
    //SmartDashboard.putNumber("IMU_Pitch", ahrs.getPitch());
    //SmartDashboard.putNumber("IMU_Roll", ahrs.getRoll());

    /* Display tilt-corrected, Magnetometer-based heading (requires */
    /* magnetometer calibration to be useful) */

    //SmartDashboard.putNumber("IMU_CompassHeading", ahrs.getCompassHeading());

    /* Display 9-axis Heading (requires magnetometer calibration to be useful) */
    //SmartDashboard.putNumber("IMU_FusedHeading", ahrs.getFusedHeading());

    /* These functions are compatible w/the WPI Gyro Class, providing a simple */
    /* path for upgrading from the Kit-of-Parts gyro to the navx MXP */

    //SmartDashboard.putNumber("IMU_TotalYaw", ahrs.getAngle());
    //SmartDashboard.putNumber("IMU_YawRateDPS", ahrs.getRate());

    /* Display Processed Acceleration Data (Linear Acceleration, Motion Detect) */

    //SmartDashboard.putNumber("IMU_Accel_X", ahrs.getWorldLinearAccelX());
    //SmartDashboard.putNumber("IMU_Accel_Y", ahrs.getWorldLinearAccelY());
    //SmartDashboard.putBoolean("IMU_IsMoving", ahrs.isMoving());
    //SmartDashboard.putBoolean("IMU_IsRotating", ahrs.isRotating());

    /* Display estimates of velocity/displacement. Note that these values are */
    /* not expected to be accurate enough for estimating robot position on a */
    /* FIRST FRC Robotics Field, due to accelerometer noise and the compounding */
    /* of these errors due to single (velocity) integration and especially */
    /* double (displacement) integration. */

    //SmartDashboard.putNumber("Velocity_X", ahrs.getVelocityX());
    //SmartDashboard.putNumber("Velocity_Y", ahrs.getVelocityY());
    //SmartDashboard.putNumber("Displacement_X", ahrs.getDisplacementX());
    //SmartDashboard.putNumber("Displacement_Y", ahrs.getDisplacementY());

    /* Display Raw Gyro/Accelerometer/Magnetometer Values */
    /* NOTE: These values are not normally necessary, but are made available */
    /* for advanced users. Before using this data, please consider whether */
    /* the processed data (see above) will suit your needs. */

    //SmartDashboard.putNumber("RawGyro_X", ahrs.getRawGyroX());
    //SmartDashboard.putNumber("RawGyro_Y", ahrs.getRawGyroY());
    //SmartDashboard.putNumber("RawGyro_Z", ahrs.getRawGyroZ());
    //SmartDashboard.putNumber("RawAccel_X", ahrs.getRawAccelX());
    //SmartDashboard.putNumber("RawAccel_Y", ahrs.getRawAccelY());
    //SmartDashboard.putNumber("RawAccel_Z", ahrs.getRawAccelZ());
    //SmartDashboard.putNumber("RawMag_X", ahrs.getRawMagX());
    //SmartDashboard.putNumber("RawMag_Y", ahrs.getRawMagY());
    //SmartDashboard.putNumber("RawMag_Z", ahrs.getRawMagZ());
    //SmartDashboard.putNumber("IMU_Temp_C", ahrs.getTempC());
    //SmartDashboard.putNumber("IMU_Timestamp", ahrs.getLastSensorTimestamp());

    /* Omnimount Yaw Axis Information */
    /* For more info, see http://navx-mxp.kauailabs.com/installation/omnimount */
    //AHRS.BoardYawAxis yaw_axis = ahrs.getBoardYawAxis();
    //SmartDashboard.putString("YawAxisDirection", yaw_axis.up ? "Up" : "Down");
    //SmartDashboard.putNumber("YawAxis", yaw_axis.board_axis.getValue());

    /* Sensor Board Information */
    //SmartDashboard.putString("FirmwareVersion", ahrs.getFirmwareVersion());

    /* Quaternion Data */
    /* Quaternions are fascinating, and are the most compact representation of */
    /* orientation data. All of the Yaw, Pitch and Roll Values can be derived */
    /* from the Quaternions. If interested in motion processing, knowledge of */
    /* Quaternions is highly recommended. */
    //SmartDashboard.putNumber("QuaternionW", ahrs.getQuaternionW());
    //SmartDashboard.putNumber("QuaternionX", ahrs.getQuaternionX());
    //SmartDashboard.putNumber("QuaternionY", ahrs.getQuaternionY());
    //SmartDashboard.putNumber("QuaternionZ", ahrs.getQuaternionZ());    
  }

}
