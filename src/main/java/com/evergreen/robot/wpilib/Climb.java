/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.evergreen.robot.wpilib;

import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.evergreen.robot.RobotMap.MotorPorts;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climb extends SubsystemBase {
  
  //creates an instance of Climb
  private static Climb m_instance;

  //creates speed constants
  double CLIMB_UP_SPEED = 0.5;
  double CLIMB_PULL_SPEED = 0.5; 

  //creates suppliers for the speeds
  private Supplier<Double> climbUpSpeed =
     () -> Preferences.getInstance().getDouble("Climb/Elevator Speed", CLIMB_UP_SPEED);
  private Supplier<Double> climbPullSpeed =
    () -> Preferences.getInstance().getDouble("Climb/Pull-Up Speed", CLIMB_PULL_SPEED);
  
  //creates the speed controllers
  private SpeedController m_climbPull = new WPI_TalonSRX(MotorPorts.climbPull);
  private SpeedController m_climbUp = new WPI_TalonSRX(MotorPorts.climbUp);

  //creates object for the hook elevating method
  public CommandBase m_up = new RunCommand(() -> climbUp(climbUpSpeed.get()), Climb.getInstance()) {
    @Override
    public void end(boolean interrupted) {
      m_climbUp.set(0.0);
    }
  };

  //creates object for pulling up method
  public CommandBase m_pull = new RunCommand(() -> climbPull(climbPullSpeed.get()), Climb.getInstance()) {
    @Override
    public void end(boolean interrupted) {
      m_climbPull.set(0.0);
    }
  };

  /**
   * Creates a new Climb.
   */
  private Climb() {
    //uploads the speed constants to the shuffelboard
    Preferences.getInstance().putDouble("Climb/Elevator Speed", CLIMB_UP_SPEED);
    Preferences.getInstance().putDouble("Climb/Pull-Up Speed", CLIMB_PULL_SPEED);
  }

  //creates get instance method
  public static Climb getInstance() {
    if (m_instance == null) m_instance = new Climb();
    return m_instance;
  }

  //creates the hook elevating method
  public void climbUp(double upSpeed) {
    m_climbUp.set(upSpeed);

  }

  //creates pulling up method
  public void climbPull(double pullSpeed) {
    m_climbPull.set(pullSpeed);
  }
  
  //ends climbing by setting all speeds to 0
  public void endClimb() {
    m_climbUp.set(0);
    m_climbPull.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
