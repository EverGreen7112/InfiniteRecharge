/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.evergreen.robot.wpilib;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.evergreen.robot.RobotMap.DigitalPorts;
import com.evergreen.robot.RobotMap.EncoderPorts;
import com.evergreen.robot.RobotMap.MotorPorts;
import com.revrobotics.CANDigitalInput.LimitSwitch;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Rolletta extends SubsystemBase {

  private static Rolletta m_instance;

  private SpeedController Spinner = new WPI_TalonSRX(MotorPorts.spiner);
  private SpeedController Lifter = new WPI_VictorSPX(MotorPorts.lifter);

  private Encoder spinner = new Encoder(EncoderPorts.rollettaA,EncoderPorts.rollettaB);

  private DigitalInput upperSwitch = new DigitalInput(DigitalPorts.rollettaUpperSwitch);
  private DigitalInput lowerSwitch = new DigitalInput(DigitalPorts.rollettaLowerSwitch);
  
  /**
   * Creates a new Rolletta.
   */
  public Rolletta() {
  
  }

  private static Rolletta getInstance() {
    if (m_instance == null) m_instance = new Rolletta();
    return m_instance;
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
