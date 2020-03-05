/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.evergreen.robot.wpilib.commands;

import com.evergreen.robot.wpilib.Chassis;
import com.evergreen.robot.wpilib.DoubleArgCommand;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class MoveChassisByTime extends CommandBase implements DoubleArgCommand {
  /**
   * Creates a new MoveChassisByTime.
   */
  private double m_startTime;
  private double TIME_CONST;
  private double m_speed = Preferences.getInstance().getDouble("Chassis/Speed", 0.7);

  public MoveChassisByTime() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_startTime = System.currentTimeMillis();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Chassis.getInstance().move(m_speed);
  }
  
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return System.currentTimeMillis() - m_startTime > TIME_CONST;
  }
  
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
      Chassis.getInstance().move(0);
    }

  @Override
  public void setValue(double value) {
    TIME_CONST = value;
  }

  @Override
  public double getValue() {
    // TODO Auto-generated method stub
    return TIME_CONST;
  }
}
