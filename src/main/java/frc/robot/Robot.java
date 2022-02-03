/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {
  
  private SendableChooser<String> autoChooser;
  private SendableChooser<String> pushBotChooser;
  private RobotContainer m_robotContainer;
  private Joystick gamepad;
  private Joystick rightStick;
  private CommandBase desiredAutoCommand = null;
  private ShuffleboardTab tab = Shuffleboard.getTab("Things Tab");
  private NetworkTableEntry waitNT = tab.add("WaitTime",0.0).getEntry();
  private NetworkTableEntry sideNT = tab.add("Direction","Right").getEntry();
  private double waitTime = 0;
  private Compressor c;
  private Solenoid s;

  private WPI_VictorSPX intakeBar;
  private WPI_VictorSPX intakeMotor2;
  private int port1 = 6;
  private int port2 = 7;

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
    c = new Compressor(1, PneumaticsModuleType.REVPH);
    s = new Solenoid(1,PneumaticsModuleType.REVPH , 0);
    m_robotContainer = new RobotContainer();
    gamepad = m_robotContainer.gamepad;
    rightStick = m_robotContainer.rightStick;
    intakeBar = new WPI_VictorSPX(port1);
    intakeMotor2 = new WPI_VictorSPX(port2);
    
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
    c.disable();
  }

  @Override
  public void autonomousInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void autonomousPeriodic() {
    CommandScheduler.getInstance().run();
    
  }

  @Override
  public void teleopInit() {
    c.enableDigital();
    c.start();
    
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void teleopPeriodic() {
    //This should fire off commands to the robot based on the user input to controller?
    //Every 20 ms
    CommandScheduler.getInstance().run();

    //solenoit
    if(gamepad.getRawButtonPressed(Constants.A_Button)) {
      s.toggle();
    }
    
    if(gamepad.getRawButtonPressed(Constants.Left_Bumper_Button)) {
      intakeBar.set(.5);
    }
    
    if(gamepad.getRawButtonPressed(Constants.Left_Trigger_Button)) {
      intakeBar.set(-0.5);
    }
    if(gamepad.getRawButtonPressed(Constants.Right_Bumper_Button)) {
      intakeMotor2.set(.5);
    }
    
    if(gamepad.getRawButtonPressed(Constants.Right_Trigger_Button)) {
      intakeMotor2.set(-0.5);
    }

    if(gamepad.getRawButtonPressed(Constants.Y_Button)) {
      intakeBar.set(0.0);
      intakeMotor2.set(0.0);
    }
    // Display values to driver station
  }

  @Override
  public void testInit() {
    // limelight.setLED(1);
    // limelight.setCAM(0);
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }

  
}
