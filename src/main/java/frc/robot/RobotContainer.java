/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import static frc.robot.Constants.RobotContainerConstants.gamepadPort;
import static frc.robot.Constants.RobotContainerConstants.leftStickPort;
import static frc.robot.Constants.RobotContainerConstants.rightStickPort;
import static frc.robot.Constants.RouteFinderConstants.kMaxAccelerationMetersPerSecondSquared;
import static frc.robot.Constants.RouteFinderConstants.kMaxSpeedMetersPerSecond;
import static frc.robot.Constants.RouteFinderConstants.kTrackwidthMeters;
import static frc.robot.Constants.RouteFinderConstants.kaVoltSecondsSquaredPerMeter;
import static frc.robot.Constants.RouteFinderConstants.ksVolts;
import static frc.robot.Constants.RouteFinderConstants.kvVoltSecondsPerMeter;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

import javax.xml.crypto.dsig.Transform;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
// import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.math.controller.PIDController;

public class RobotContainer {
  // Subsystems should be private here and have to be passed to commands because it is better coding practice.

  // Joysticks
  public static final Joystick rightStick = new Joystick(rightStickPort);
  public static final Joystick leftStick = new Joystick(leftStickPort);
  public static final Joystick gamepad = new Joystick(gamepadPort);
  
  // Drivetrain
  // public static final Drivetrain drivetrain = new Drivetrain();

  // Shooter
  // public static final Shooter shooter = new Shooter();

  // // Feeder
  // public static final Feeder feeder = new Feeder();

  // // Intake
  // public static final Intake intake = new Intake();

  // // Limelight
  // public static final Limelight limelight = new Limelight();

  // // Turret
  // public static final Turret turret = new Turret(limelight);

  // //Climber
  // public static final Climber climber = new Climber();

  //Trajectory
  public static Trajectory trajectory;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
     /**
     * COMMAND INFO From the official wpilib docs: "While users are able to create
     * commands by explicitly writing command classes (either by subclassing
     * CommandBase or implementing Command), for many commands (such as those that
     * simply call a single subsystem method) this involves a lot of wasteful
     * boilerplate code. To help alleviate this, many of the prewritten commands
     * included in the command-based library may be inlined - that is, the command
     * body can be defined in a single line of code at command construction."
     * 
     * TLDR: You shouldn't create a whole new file for a command that only calls one
     * method.
     * 
     * (I didn't know what these did so here is an explanation if anyone is
     * confused) Also method references and lambda expressions let you pass
     * subroutines as parameters Method reference: subsystem::method Lambda
     * expression: () -> subsystem.method() They essentially do the same thing
     */
    // Attaches a commmand to each button
    // Starts the shooter motors when the Y button is pressed
    // new JoystickButton(gamepad, Constants.Left_Bumper_Button).whenPressed(new RunShooter(shooter, feeder, limelight, gamepad));
    // Takes in balls from the ground when the right trigger is held
    // new JoystickButton(gamepad, Constants.Right_Bumper_Button).whenHeld(new IntakeIn(intake,gamepad));
    // new JoystickButton(gamepad, Constants.Right_Bumper_Button).whenReleased(new IntakeStop(intake,gamepad));
    // // Pushes out balls onto the ground when the right bumper is held
    // new JoystickButton(gamepad, Constants.Right_Trigger_Button).whenHeld(new IntakeOut(intake,gamepad));
    // new JoystickButton(gamepad, Constants.Right_Trigger_Button).whenReleased(new IntakeStop(intake,gamepad));
    //Run the feeder system
    // new JoystickButton(gamepad, Constants.Left_Trigger_Button).whenHeld(new FeederRun(feeder, intake, shooter, gamepad));
    // //Lift Intake pneau
    // new JoystickButton(gamepad, Constants.X_Button).whenPressed(new IntakeUp(intake));
    // //lower intake pneau
    // new JoystickButton(gamepad, Constants.Y_Button).whenPressed(new IntakeDown(intake));

    // new JoystickButton(gamepad, Constants.Left_Joystick_Pressed).whenHeld(new WithdrawClimber(climber, gamepad));

    // new JoystickButton(gamepad,Constants.Left_Joystick_Pressed).whenReleased(new StopClimber(climber));
    // // Starts targeting when the up arrow on the D-pad is pressed
    // new POVButton(gamepad, povUp).whenPressed(new DeployClimber(climber, gamepad));
    // // Ends targeting when the down arrow on the D-pad is pressed
    // new POVButton(gamepad, povDown).whenHeld(new ClearAllBalls(intake, feeder, shooter, gamepad));
    // new POVButton(gamepad, povDown).whenReleased(new StopBelts(intake, feeder, shooter, gamepad));
    //raise shooter angle
    // new POVButton(gamepad, povRight).whenPressed(new PancakeUp(shooter, gamepad));
    //lower shooter angle
    // new POVButton(gamepad, povLeft).whenPressed(new PancakeDown(shooter, gamepad));
    // Heads to a position when the left bumper is pressed
    // new JoystickButton(gamepad, Button.kBumperLeft.value)
    //     .whenPressed(RouteFinder.getPathCommand(RouteFinder.trajectorygen(pointx, pointy, rotation)));
    
    // Driving
    // new PerpetualCommand(new DriveTele(drivetrain, rightStick, leftStick)).schedule();
    // Turret
   // new PerpetualCommand(new MoveTurret(turret, gamepad)).schedule(); 
  }

}