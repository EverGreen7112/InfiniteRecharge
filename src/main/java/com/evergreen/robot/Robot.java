/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.evergreen.robot;

import java.util.List;

import com.evergreen.everlib.CommandEG;
import com.evergreen.everlib.oi.joysticks.F310GamePad;
import com.evergreen.everlib.oi.joysticks.F310GamePad.F310;
import com.evergreen.everlib.structure.Tree;
import com.evergreen.everlib.subsystems.motors.subsystems.MotorSubsystem;
import com.evergreen.robot.CommandList.ClimbingCommands;

/**
 * This is the class representing our robot!
 * <p>
 * It contains each of its subsystems, and the code it runs at each stage of the game.
 * <p>
 * created and constructed in {@link Main} - The code actually ran.
 * The DriverStation will call each game stage corresponding <code>init()</code> method at its start,
 * and then repeatedly call its <code>periodic()</code> method. 
 * <p>
 * <b>Note That these methods aren't overriden in here, and it is preferable to only use the supplied {@link Tree} methods for
 * organizing the robot code.</b>
 */
public class Robot extends Tree implements SubsystemComponents, RobotMap {

  
  //----------Subsystem Declerations----------
  //-----Motor Subsystems-----
  public static final MotorSubsystem climbElevator = 
    new MotorSubsystem("climbing elevator",ClimbingComponents.elevator);
  public static final MotorSubsystem climbPuller =
    new MotorSubsystem("climbing puller", ClimbingComponents.puller);
  // public static final DriveTank chassis = new DriveTank(...);
  public static final MotorSubsystem collector = new MotorSubsystem(
    "Collector",
     CollectorComponents.motor);

  //-----Piston Subsytem-----
  // public static final PistonSubsystem subsystemC = new PistonSubsystem(..);
  
  //-----Joysticks-----
  public static final F310GamePad joystickButton = new F310GamePad("Button JS", JoystickPorts.buttonJS);
  // public static final ExtremeProJoystick joystickLeft = new Joystick(...);
  // public static final ExtremeProJoystick joystickRight = new Joystick(...);

  //public static final JoystickEG joystick = new JoystickEG("name", JoystickPorts.button); //Change name!


  //-----Network Tables-----
  // public final NetworkTable imageProccesing = NetworkTableInstance.getDefault().getTable("...");


  @Override
  protected void componentSetup() {
    // SubsystemAComponents.motors.setInverted(...);
  }
  
  
  @Override
  protected void bindButtons() {
    joystickButton.getButton(F310.X).whileHeld(CommandList.CollectorCommands.collect); //TODO Change button type when decided
    joystickButton.getButton(F310.Y).whenPressed(ClimbingCommands.pullUp);
  }
  
  @Override
  protected void commandConfig() {
    //subsystem.setDefaultCommand(...);
  }
  
  @Override
  protected void log() {
    // DashboardStreams.addLoggable(...);
    // DashboardStreams.addDouble(...);
  }
  
  @Override
  protected void whenEnabled() {
    // SubsystemC.setForward();
  }

  @Override
  protected void test() {
    //SubsystemA.move(...);
  }
  
  @Override
  protected void autoConfig() {    
    // subsystem.setDefaultCommand(...)
  }

  @Override
  protected void teleopConfig() {
    // subsystem.setDefaultCommand(...);
  }

  @Override
  protected List<CommandEG> getAutoCommands() {
    // return List.of (...);
    return super.getAutoCommands();
  }

  @Override
  protected List<CommandEG> getTeleopCommands() {
    // return List.of(...);
    return super.getTeleopCommands();
  }


}
