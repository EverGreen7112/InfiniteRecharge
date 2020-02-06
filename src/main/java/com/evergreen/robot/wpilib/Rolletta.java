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
import com.evergreen.robot.RobotMap.DigitalPorts;
import com.evergreen.robot.RobotMap.MotorPorts;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANDigitalInput.LimitSwitch;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Rolletta extends SubsystemBase {
  //Creates the single Rolletta instance
  private static Rolletta m_instance;

  //Color RGB constants
  private double 
    BLUE_R,
    BLUE_G,
    BLUE_B,
    RED_R,
    RED_G,
    RED_B;

  //General Constants
  private double LIFT_SPEED; 
  private boolean isLifting = true;
  private final Color BLUE = ColorMatch.makeColor(r, g, b);

  //The speed controllers
  private SpeedController m_spinner = new WPI_TalonSRX(MotorPorts.spiner);
  private SpeedController m_lifter = new WPI_VictorSPX(MotorPorts.lifter);

  //The encoder for the spinning motor
  private Encoder m_spinner_encoder = new Encoder(DigitalPorts.rollettaA,DigitalPorts.rollettaB);

  //The Color Sensor
  ColorSensorV3 m_colorSensor = new ColorSensorV3(Port.kMXP);
  //The color matcher - detects known colors
  private final ColorMatch m_colorMatcher = new ColorMatch();


  //The switches to control the ifter motor
  private DigitalInput m_upperSwitch = new DigitalInput(DigitalPorts.rollettaUpperSwitch);
  private DigitalInput m_lowerSwitch = new DigitalInput(DigitalPorts.rollettaLowerSwitch);

  
  //Commands:
  /**
   * Lifting and Lowering the mechanism
   */
  public CommandBase lift = 
  new RunCommand( () -> move(isLifting, isLifting ? m_upperSwitch : m_lowerSwitch), Rolletta.getInstance()) {
    @Override
    public void end(boolean interrupted){
      m_lifter.set(0);
    }
  };
  
  /**
   * Creates a new Rolletta.
   */
  public Rolletta() {
    Preferences.getInstance().putDouble("Rolletta/Lift Speed", LIFT_SPEED);
  }
  
  /**
   * Returns the single instance
   */
  private static Rolletta getInstance() {
    if (m_instance == null) m_instance = new Rolletta();
    return m_instance;
  }

  /**
   * Lifts or Lowers the Rolltta mechanism according to the lifting parameter.
   * <p> If it is up, it will lower the mechanism and vice verca.
   * @param lifting
   * @param untilHit
   */
  public void move(boolean lifting, DigitalInput untilHit) {

    double liftSpeed = lifting ? GET_SPEED() : -GET_SPEED();

    
    if (untilHit.get()) {
      m_lifter.set(0);
      isLifting = !isLifting;
    }
    
    else  m_lifter.set(liftSpeed);
    
  }

  /**
   * <b>Rotation Control method:</b>
   * <p> Spins the control panel between 3-5 times.
   */
  public void rotationControl() {

  }

  /**
   * <b>Position Control method:</b>
   * <p> Spins the control panel to the color given from the FMS.
   */
  public void positionControl() {

  }

  /**
   * Gets the lifting speed constant from the shuffleboard.
   **/
  public double GET_SPEED() {
    return Preferences.getInstance().getDouble("Rolletta/Lift Speed", LIFT_SPEED);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}