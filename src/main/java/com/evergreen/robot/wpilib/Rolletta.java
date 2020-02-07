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
import com.evergreen.robot.RobotMap.MotorPorts;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

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
  
  //General Constants
  private double LIFT_SPEED; 
  private boolean isLifting = true;
  //Color RGB constants
  private double 
    BLUE_R,
    BLUE_G,
    BLUE_B,    
    GREEN_R,
    GREEN_G,
    GREEN_B,
    RED_R,
    RED_G,
    RED_B,
    YELLOW_R,
    YELLOW_G,
    YELLOW_B;
    //Control Panel calibrated colors - this is what we expect to read on field
    private Color BLUE = ColorMatch.makeColor(BLUE_R, BLUE_G, BLUE_B);
    private Color GREEN = ColorMatch.makeColor(GREEN_R, GREEN_G, GREEN_B);
    private Color RED = ColorMatch.makeColor(RED_R, RED_G, RED_B);
    private Color YELLOW = ColorMatch.makeColor(YELLOW_R, YELLOW_G, YELLOW_B);
    
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
    new RunCommand( () -> move(isLifting, isLifting ? m_upperSwitch : m_lowerSwitch), Rolletta.getInstance()){
    @Override
    public void end(boolean interrupted){
      m_lifter.set(0);
    }
  };

   /**
   * Calibrates the BLUE RBG components, ought to run on disabled
   */
  public CommandBase calibrateBlue = 
    new RunCommand(() -> calibrateBlue(), Rolletta.getInstance()).withTimeout(3.5);

   /**
   * Calibrates the GREEN RBG components, ought to run on disabled
   */
  public CommandBase calibrateGreen = 
    new RunCommand(() -> calibrateGreen(), Rolletta.getInstance()).withTimeout(3.5);
  
   /**
   * Calibrates the RED RBG components, ought to run on disabled
   */
  public CommandBase calibrateRed = 
    new RunCommand(() -> calibrateRed(), Rolletta.getInstance()).withTimeout(3.5);
  
   /**
   * Calibrates the YELLOW RBG components, ought to run on disabled
   */
  public CommandBase calibrateYellow = 
    new RunCommand(() -> calibrateYellow(), Rolletta.getInstance()).withTimeout(3.5);
  
  /**
   * Creates a new Rolletta.
   */
  public Rolletta() {
    //Adds to the shuffleboard constatns, detected color, etc.
    Preferences.getInstance().putDouble("Rolletta/Lift Speed", LIFT_SPEED);
    Preferences.getInstance().putString("Rolletta/Detected color", getColorString());
    Preferences.getInstance().putDouble("Rolletta/Blue/R", BLUE_R);
    Preferences.getInstance().putDouble("Rolletta/Blue/G", BLUE_G);
    Preferences.getInstance().putDouble("Rolletta/Blue/B", BLUE_B);
    Preferences.getInstance().putDouble("Rolletta/Green/R", GREEN_R);
    Preferences.getInstance().putDouble("Rolletta/Green/G", GREEN_G);
    Preferences.getInstance().putDouble("Rolletta/Green/B", GREEN_B);
    Preferences.getInstance().putDouble("Rolletta/Red/R", RED_R);
    Preferences.getInstance().putDouble("Rolletta/Red/G", RED_G);
    Preferences.getInstance().putDouble("Rolletta/Red/B", RED_B);
    Preferences.getInstance().putDouble("Rolletta/Yellow/R", YELLOW_R);
    Preferences.getInstance().putDouble("Rolletta/Yellow/G", YELLOW_G);
    Preferences.getInstance().putDouble("Rolletta/Yellow/B", YELLOW_B);

  }
  
  /**
   * Returns the single instance
   */
  public static Rolletta getInstance() {
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



  // Color methods:

  /**
   * Adds all the expected colors. Ought to be used after field calibration.
   */
  public void addAllColors(){
    m_colorMatcher.addColorMatch(BLUE);
    m_colorMatcher.addColorMatch(GREEN);
    m_colorMatcher.addColorMatch(RED);
    m_colorMatcher.addColorMatch(YELLOW);
  }

  /**
   * Calibrates the BLUE RBG components, ought to run on disabled
   */
  public void calibrateBlue() {
    Color blueDetected = m_colorSensor.getColor();
    BLUE_R = blueDetected.red;
    BLUE_G = blueDetected.green;
    BLUE_B = blueDetected.blue;
  }
  
  /**
   * Calibrates the GREEN RBG components, ought to run on disabled
   */
  public void calibrateGreen() {
    Color greenDetected = m_colorSensor.getColor();
    GREEN_R = greenDetected.red;
    GREEN_G = greenDetected.green;
    GREEN_B = greenDetected.blue;
  }

  /**
   * Calibrates the RED RBG components, ought to run on disabled
   */
  public void calibrateRed() {
    Color redDetected = m_colorSensor.getColor();
    RED_R = redDetected.red;
    RED_G = redDetected.green;
    RED_B = redDetected.blue;
  }

  /**
   * Calibrates the YELLOW RBG components, ought to run on disabled
   */
  public void calibrateYellow() {
    Color yellowDetected = m_colorSensor.getColor();
    YELLOW_R = yellowDetected.red;
    YELLOW_G = yellowDetected.green;
    YELLOW_B = yellowDetected.blue;
  }

  /**
   * Gets the color String
   */
  public String getColorString() {
    String colorString;
    ColorMatchResult match = m_colorMatcher.matchClosestColor(m_colorSensor.getColor());

    if (match.color == BLUE) {
      colorString = "Blue";
    } else if (match.color == GREEN) {
      colorString = "Green";
    } else if (match.color == RED) {
      colorString = "Red";
    } else if (match.color == YELLOW) {
      colorString = "Yellow";
    } else {
      colorString = "Unknown";
    }
    return colorString;
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