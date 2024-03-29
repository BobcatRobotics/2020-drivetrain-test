// package frc.robot.subsystems;

// import static frc.robot.Constants.DrivetrainConstants.GYRO_REVERSED;
// import static frc.robot.Constants.DrivetrainConstants.INVERT_MOTOR;

// import com.ctre.phoenix.motorcontrol.NeutralMode;
// import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
// import com.ctre.phoenix.motorcontrol.can.TalonSRX;
// import com.ctre.phoenix.motorcontrol.can.Talon;
// import com.kauailabs.navx.frc.AHRS;

// import edu.wpi.first.wpilibj.SPI;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
// import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import edu.wpi.first.wpilibj.motorcontrol.Talon;
// import edu.wpi.first.wpilibj.motorcontrol.PWMTalonSRX;
// import frc.robot.Constants;
// import frc.robot.subsystems.NavxGyro;
// import frc.robot.RobotContainer;

// public class Drivetrain extends SubsystemBase {

//    private Talon ltMotor;
//    private Talon lmMotor;
//    private Talon lbMotor;
//    private Talon rtMotor;
//    private Talon rmMotor;
//    private Talon rbMotor;
//    private TalonSRX srx;
//    public static final NavxGyro navx = new NavxGyro(SPI.Port.kMXP);


//     // The motors on the left side of the drive.
//     // private final SpeedControllerGroup leftMotors =
//     /* new SpeedControllerGroup(
//         new Talon(LTMotorPort),
//         new Talon(LMMotorPort),
//         new Talon(LLMotorPort)); */

//     // The motors on the right side of the drive.
//    // private final SpeedControllerGroup rightMotors = 
//     /* new SpeedControllerGroup(
//         new Talon(RTMotorPort),
//         new Talon(RMMotorPort),
//         new Talon(RLMotorPort));
//  */
//     // motor properties
//     private double rightPower = 0.0;
//     private double leftPower = 0.0;

//     private boolean invertRight = false; // Whether or not to invert the right motor
//     private boolean invertLeft = false; // Whether or not to invert the left motor

//     //private final DifferentialDrive diffDrive = new DifferentialDrive(leftMotors, rightMotors); // The robot's drive


//     // private final AHRS gyro = new AHRS(SPI.Port.kMXP); // The gyro sensor

//     private final DifferentialDriveOdometry odometry; // Odometry class for tracking robot pose

//     /**
//      * Method use to drive the robot
//      */
//     public Drivetrain() {

//         ltMotor = new Talon(Constants.DrivetrainConstants.LTMotorPort);
//         lmMotor = new Talon(Constants.DrivetrainConstants.LMMotorPort);
//         lbMotor = new Talon(Constants.DrivetrainConstants.LLMotorPort);
//         rtMotor = new Talon(Constants.DrivetrainConstants.RTMotorPort);
//         rmMotor = new Talon(Constants.DrivetrainConstants.RMMotorPort);
//         rbMotor = new Talon(Constants.DrivetrainConstants.RLMotorPort);
        

//         ltMotor.setNeutralMode(NeutralMode.Brake);
//         lmMotor.setNeutralMode(NeutralMode.Brake);
//         lbMotor.setNeutralMode(NeutralMode.Brake);
//         rtMotor.setNeutralMode(NeutralMode.Brake);
//         rmMotor.setNeutralMode(NeutralMode.Brake);
//         rbMotor.setNeutralMode(NeutralMode.Brake);

//         ltMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,0,0);
//         lmMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,0,0);
//         lbMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,0,0);
//         rtMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,0,0);
//         rmMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,0,0);
//         rbMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,0,0);
        
//         ltMotor.setInverted(false);
//         lmMotor.setInverted(false);
//         lbMotor.setInverted(false);
//         rmMotor.setInverted(true);
//         rtMotor.setInverted(true);
//         rbMotor.setInverted(true);
//         odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));
//         resetOdometry();
//         resetEncoders();
//         zeroHeading();
//     }

//     /**
//      * Get the left motor power
//      * @return The left motor power
//      */
//     public double getLeftPower() {
//         return leftPower;
//     }

//     /**
//      * Get the right motor power
//      * @return The right motor power
//      */
//     public double getRightPower() {
//         return rightPower;
//     }

//     /**
//      * Set the right power
//      * @param pwr Value to set the power to
//      */
//      public void setRightPower(double pwr) {
//          if (pwr > 1.0) {
//              rightPower = 1.0;
//              return;
//          } else if (pwr < -1.0) {
//              rightPower = -1.0;
//              return;
//          }

//          rightPower = pwr;
//      }

//      /**
//       * Set the left power
//       * @param pwr Value to set the power to
//       */
//      public void setLeftPower(double pwr) {
//         if (pwr > 1.0) {
//             leftPower = 1.0;
//             return;
//         } else if (pwr < -1.0) {
//             leftPower = -1.0;
//             return;
//         }

//         leftPower = pwr;
//      }

//      /**
//       * Stop the right motor
//       */
//      public void stopRightMotor() {
//          rightPower = 0.0;
//          rtMotor.stopMotor();
//          rmMotor.stopMotor();
//          rbMotor.stopMotor();
//      }

//     /**
//      * Stop the left motor
//      */
//     public void stopLeftMotor() {
//         leftPower = 0.0;
//         ltMotor.stopMotor();
//         lmMotor.stopMotor();
//         lbMotor.stopMotor();
//     }

//     /**
//      * Drive with default values from the joysticks
//      */
//     public void drive() {
//         drive(rightPower, leftPower);
//     }
    
//     /**
//      * Drive with custom values
//      * @param rightP Right motor power
//      * @param leftP Left motor power
//      */
//     public void drive(double rightP, double leftP) {
//         if (invertRight) {
//             rightP *= INVERT_MOTOR;
//         }
        
//         if (invertLeft) {
//             leftP *= INVERT_MOTOR;
//         }
        
//         rtMotor.set(rightP);
//         rmMotor.set(rightP);
//         rbMotor.set(rightP);

//         ltMotor.set(leftP);
//         lmMotor.set(leftP);
//         lbMotor.set(leftP);
//     }

//     public void checkmotors(){
//         rtMotor.getSelectedSensorVelocity();
//         rmMotor.getSelectedSensorVelocity();
//         rbMotor.getSelectedSensorVelocity();
//         ltMotor.getSelectedSensorVelocity();
//         lmMotor.getSelectedSensorVelocity();
//         lbMotor.getSelectedSensorVelocity();
//     }
        

//     /**
//      * Stops the drive train
//      */
//     public void stop() {
//         stopRightMotor();
//         stopLeftMotor();
//     }

//     /**
//      * Called periodically by the CommmandScheduler
//      */
//     @Override
//     public void periodic() {
//         //odometry.update(Rotation2d.fromDegrees(getHeading()), leftEncoder.getDistance(), rightEncoder.getDistance()); // Update the odometry in the periodic block (gets location on field)
//         //periodic called ever .02s
//         odometry.update(Rotation2d.fromDegrees(getHeading()), ltMotor.getSensorCollection().getIntegratedSensorPosition(), rtMotor.getSensorCollection().getIntegratedSensorPosition());
//     }

//     /**
//      * Returns the currently-estimated pose of the robot
//      * @return The pose. (position on the field)
//      */
//     public Pose2d getPose() {
//         return odometry.getPoseMeters();
//     }

//     /**
//      * Returns the current wheel speeds of the robot
//      * @return The current wheel speeds.
//      */
//     public DifferentialDriveWheelSpeeds getWheelSpeeds() {
//         //return new DifferentialDriveWheelSpeeds(leftEncoder.getRate(), rightEncoder.getRate());
//         return new DifferentialDriveWheelSpeeds(ltMotor.getSelectedSensorVelocity(), rtMotor.getSelectedSensorVelocity());
//     }

//     /**
//      * Resets the odometry to a default pose Pose2d(0, 0, new Rotation2d(0))
//      * Resets the encoders, also automatically resets heading
//      */
//     public void resetOdometry() {
//         resetOdometry(new Pose2d(0, 0, new Rotation2d(0)));
//     }

//     /**
//      * Resets the odometry to the specified pose
//      * Resets the encoders, also automatically resets heading
//      * @param pose The pose to which to set the odometry
//      */
//     public void resetOdometry(Pose2d pose) {
//         resetEncoders();
//         odometry.resetPosition(pose, Rotation2d.fromDegrees(getHeading()));
//     }

//     /**
//      * Drives the robot using arcade controls
//      * @param fwd the commanded forward movement
//      * @param rot the commanded rotation
//      */
//     public void arcadeDrive(double fwd, double rot) {
//         //diffDrive.arcadeDrive(fwd, rot);
//     }

//     /**
//      * Controls the left and right sides of the drive directly with voltages
//      * @param leftVolts the commanded left output
//      * @param rightVolts the commanded right output
//      */
//     public void tankDriveVolts(double leftVolts, double rightVolts) {
//         ltMotor.setVoltage(leftVolts);
//         lmMotor.setVoltage(leftVolts);
//         lbMotor.setVoltage(leftVolts);
//         rtMotor.setVoltage(rightVolts);
//         rmMotor.setVoltage(rightVolts);
//         rbMotor.setVoltage(rightVolts);        
//         odometry.update(Rotation2d.fromDegrees(navx.getYaw()), ltMotor.getSensorCollection().getIntegratedSensorPosition(), rtMotor.getSensorCollection().getIntegratedSensorPosition());
        
//     } 

//     /**
//      * Resets the drive encoders to currently read a position of 0
//      */
//     public void resetEncoders() {
//         ltMotor.setSelectedSensorPosition(0.0);
//         lmMotor.setSelectedSensorPosition(0.0);
//         lbMotor.setSelectedSensorPosition(0.0);
//         rtMotor.setSelectedSensorPosition(0.0);
//         rmMotor.setSelectedSensorPosition(0.0);
//         rbMotor.setSelectedSensorPosition(0.0);

//         // leftEncoder.reset();
//         // rightEncoder.reset();
//     }

//     /**
//      * Gets the average distance of the two encoders
//      * @return the average of the two encoder readings
//      */
//     public double getAverageEncoderDistance() {
//         return ltMotor.getSensorCollection().getIntegratedSensorPosition()/2048 *( .5 * Math.PI);
//     }

//     /**
//      * Sets the max output of the drive.  Useful for scaling the drive to drive more slowly
//      * @param maxOutput the maximum output to which the drive will be constrained
//      */
//     /* public void setMaxOutput(double maxOutput) {
//         diffDrive.setMaxOutput(maxOutput);
//     } */

//     /**
//      * Zeroes the heading of the robot
//      */
//     public void zeroHeading() {
//         navx.reset();
//     }

//     /**
//      * Returns the heading of the robot
//      * @return the robot's heading in degrees, from -180 to 180
//      */
//     public double getHeading() {
//         return navx.pidGet() ;
//     }

//     /**
//      * Returns the turn rate of the robot
//      * @return The turn rate of the robot, in degrees per second
//      */
//     public double getTurnRate() {
//         return navx.getRate() ;
//     }
// }