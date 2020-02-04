/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.evergreen.robot.wpilib;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.evergreen.robot.RobotMap.DigitalPorts;
import com.evergreen.robot.RobotMap.MotorPorts;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Storage extends SubsystemBase {

  // Declares the Storage single instance
  private static Storage m_instance;

  // The passing speed and time constants
  private double PASS_SPEED = 0.5;
  private long PASS_TIME = 2000; //Notice the time is in milliseconds

  //New speed controller and ultrasonic sensor for passing the power cells.
  private SpeedController m_passMotor = new WPI_VictorSPX(MotorPorts.passer);
  private Ultrasonic ultrasonic = 
    new Ultrasonic(DigitalPorts.storageUltrasonicA, DigitalPorts.storageUltrasonicB);

  /**
   * Passes a Power Cell to the Shooter
   */
  public CommandBase passByTime = new RunCommand(() -> m_passByTime(getSpeed(), getTime()), Storage.getInstance()) {
    @Override
    public void end(boolean interrupted) {
      m_passMotor.set(0);
    }
  };

  /**
   * Creates a new Storage.
   */
  private Storage() {
    Preferences.getInstance().putDouble("Storage/Passing Speed", PASS_SPEED);
    Preferences.getInstance().putLong("Storage/Passing Time", PASS_TIME);
  }

  /**
   * Gets the Storage single instance
   */
  public static Storage getInstance() {
    if (m_instance == null)
      m_instance = new Storage();
    return m_instance;
  }

  /**
   * Sets the passing motor to input speed
   * 
   * @throws InterruptedException
   */
  public void m_passByTime(double m_speed, long m_time) {
    m_passMotor.set(m_speed);
    try {
      Thread.sleep(m_time);
    } catch (InterruptedException e) {
      e.printStackTrace();
      throw new RuntimeException();
    }
    m_passMotor.set(0);
  }

  public void m_passBySensor(double m_speed, double dist) {
//TODO: add the pass by sensor command + everything it needs.
  }


  /**
   * Gets the storage passing motor speed
   */
  public double getSpeed() {
    return Preferences.getInstance().getDouble("Storage/Passing Speed", PASS_SPEED);
  }

  /**
   * Gets the storage passing motor time (in milliseconds)
   */
  public long getTime() {
    return Preferences.getInstance().getLong("Storage/Passing Time", PASS_TIME);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
