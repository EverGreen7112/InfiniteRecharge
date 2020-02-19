/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.evergreen.robot.wpilib;

import java.util.Set;
import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.evergreen.everlib.shuffleboard.loggables.DashboardStreams;
import com.evergreen.robot.RobotMap.DigitalPorts;
import com.evergreen.robot.RobotMap.MotorPorts;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Rolletta extends SubsystemBase {
  //Creates the single Rolletta instance
  private static Rolletta m_instance;
  
  //General Constants
   private double LIFT_SPEED = 0.3;  
   //45 degrees offset to make sure it's not too little
   private double ROTATION_CONTROL_SETPOINT = 360 * 3 + 45; 
   private double ROBOT_SENSOR_OFFSET = 90;
   private boolean m_isLifting ;

  //Color RGB constants][
  private double 
    BLUE_R,
    BLUE_G,
    BLUE_B,    
    GREEN_R,
    GREEN_G,
    GREEN_B, //Add 60 degrees offset to make sure.
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
  private WPI_TalonSRX m_spinner = new WPI_TalonSRX(MotorPorts.spinner);
  public SpeedController m_lifter = new WPI_VictorSPX(MotorPorts.lifter);

  //The encoder for the spinning motor
  //private Encoder m_spinnerEncoder = new Encoder(DigitalPorts.rollettaA,DigitalPorts.rollettaB);

  //The Color Sensor
  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(Port.kMXP);
  //The color matcher - detects known colors
  private final ColorMatch m_colorMatcher = new ColorMatch();


  //The switches to control the ifter motor
  //TODO fix ports
  private DigitalInput m_upperSwitch = new DigitalInput(DigitalPorts.rolletaMicroSwitchUp);
  private DigitalInput m_lowerSwitch = new DigitalInput(DigitalPorts.rolletaMicroSwitchDown);


  private double m_rightOffset = 0;
  private double m_leftOffset = 0;

  
  public boolean isDown() {
    return !m_lowerSwitch.get();
  }

  public boolean isUp() {
    return m_upperSwitch.get();
  }

  //Commands:
  /**
   * Lifting and Lowering the mechanism
   */
  public CommandBase lift() { 
    return  new CommandBase() {

      @Override
      public void initialize() {
          addRequirements(Rolletta.getInstance());
      }

      @Override
      public boolean isFinished() {
        return move(m_isLifting);
      }

      @Override
      public void end(boolean interrupted) {
        m_isLifting = !m_isLifting;
        m_lifter.set(0);        
      }


    };
    
  };

   /**
   * Calibrates the BLUE RBG components, ought to run on disabled
   */
  public CommandBase m_calibrateBlue() {
    return new RunCommand(() -> calibrateBlue(), this).withTimeout(3.5);
  }

   /**
   * Calibrates the GREEN RBG components, ought to run on disabled
   */
  public CommandBase m_calibrateGreen() {
    return new RunCommand(() -> calibrateGreen(), this).withTimeout(3.5);
}
   /**
   * Calibrates the RED RBG components, ought to run on disabled
   */
  public CommandBase m_calibrateRed() {
    return new RunCommand(() -> calibrateRed(), this).withTimeout(3.5);
  }
   /**
   * Calibrates the YELLOW RBG components, ought to run on disabled
   */
  public CommandBase m_calibrateYellow() {
    return new RunCommand(() -> calibrateYellow(), this).withTimeout(3.5);
  }

  public boolean isLifting() {
    return m_isLifting;
  }

  /**
   * Creates a new Rolletta.
   */
  public Rolletta() {
    //Adds to the shuffleboard constatns, detected color, etc.
    m_lifter.setInverted(true);
    m_isLifting= !isUp();

    Preferences.getInstance().putDouble("Rolletta/Lift Speed", LIFT_SPEED);
    Preferences.getInstance().putString("Rolletta/Detected color", getCurrentColor());
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
    Preferences.getInstance().putDouble("Rolleta/PID/kP", 0);
    Preferences.getInstance().putDouble("Rolleta/PID/kI", 0);
    Preferences.getInstance().putDouble("Rolleta/PID/kD", 0);
    Preferences.getInstance().putDouble("Rolleta/PID/Tolerance", 0);

  }
  
  /**
   * Returns the single instance
   */
  public static synchronized Rolletta getInstance() {
    if (m_instance == null) m_instance = new Rolletta();
    return m_instance;
  }

  /**
   * Lifts or Lowers the Rolltta mechanism according to the lifting parameter.
   * <p> If it is up, it will lower the mechanism and vice verca.
   * @param lifting
   * @param untilHit
   */
  public boolean move(boolean lifting) {
    double liftSpeed = lifting ? GET_SPEED() : -GET_SPEED();
    Supplier<Boolean> untilHit = lifting ? this::isUp : this::isDown;

    if (!untilHit.get()) {
      m_lifter.set(liftSpeed);
      return false;
    }
    
    return true;


    
  }

  /**
   * <b>Rotation Control method:</b>
   * <p> 
   * Spins the control panel between 3-5 times.
   */
  public void rotationControl() {
    spinDegrees(ROTATION_CONTROL_SETPOINT); 
  }

  /**
   * <b>Position Control method:</b>
   * <p>Spins the control panel to the color given from the FMS.
   */
  public void positionControl() {


    if (toLeft(getTargetAngle()) - getLeftAngle() > getTargetAngle() - getRightAngle())
      spinTo(getTargetAngle());
    else
      spinTo(toLeft(getTargetAngle()));
  }


  /**
   * @return the kP constant for the rolleta spinning PID
   */
  private double GET_KP() {
    return Preferences.getInstance().getDouble("Rolleta Spinner/PID/kP", 0);
  }

  /**
   * @return The kI constant for the rolleta spinning PID 
   */
  private double GET_KI() {
    return Preferences.getInstance().getDouble("Rolleta Spinner/PID/kI", 0);
  }

  /**
   * @return the kD constant for the rolleta spinning PID
   */
  private double GET_KD() {
    return Preferences.getInstance().getDouble("Rolleta Spinner/PID/kD", 0);
  }

  /**
   * @return The tolerance constant for the rolleta spinning PID
   */
  private double GET_TOLERANCE() {
    return Preferences.getInstance().getDouble("Rolleta Spinner/PID/Tolerance", 0);
  }

  /**
   * @return The PID controller of the rolleta spin
   */
  private PIDController getController() {
    return new PIDController(GET_KP(), GET_KI(), GET_KD());
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
  public String getCurrentColor() {    
    ColorMatchResult match = m_colorMatcher.matchClosestColor(m_colorSensor.getColor());

    if (match.color == RED) {
      return "R";
    } else if (match.color == GREEN) {
      return "G";
    } else if (match.color == BLUE) {
      return "B";
    } else if (match.color == YELLOW) {
      return "Y";
    } else {
      return "Unknown";
    }
  }


  // Sensor Methods
  
  /**
   * @return the angle we need to target, modulu 180 with RED=0.
   */
  public double getTargetAngle() {
    return getColorAngle(DriverStation.getInstance().getGameSpecificMessage());
  }

  /**Resets the spinner encoder and the current offset according to current sensor input.*/
  public void resetSensor() {
    m_spinner.setSelectedSensorPosition(0);
    m_rightOffset = getColorAngle(getCurrentColor()) + ROBOT_SENSOR_OFFSET;
    m_leftOffset = toLeft(getColorAngle(getCurrentColor())) - ROBOT_SENSOR_OFFSET;
  }

  /**
   * @return The current negative angle according 
   * to the encoder within modulu 180, accounting for 
   * offset in counter-clcokwise rotation.
   **/
  public double getLeftAngle() {
    return toLeft(getRawAngle() + m_leftOffset);
  }

  /**
   * @return The currentangle according 
   * to the encoder within modulu 180, accounting for 
   * offset in clcokwise rotation.
   **/
  public double getRightAngle() {
    return (getRawAngle() + m_rightOffset) % 180;
  }

  /**
   * @return The current angle according to the encoder, accumelated with each round since
   * {@link #resetSensor()}
   */
  public double getRawAngle() {
    return m_spinner.getSelectedSensorPosition();
  }

  
  

  /**
   * Gets the lifting speed constant from the shuffleboard.
   **/
  public double GET_SPEED() {
    return Preferences.getInstance().getDouble("Rolletta/Lift Speed", LIFT_SPEED);
  }


 

  /**
   * Calculates the angle of a certain color on the control panel by its string 
   * representation. Red is concidered 0 degrees, and each color advances the angle 
   * by 45 degrees (as it is one eighth of the circle).
   * 
   * @param color - the string representation of the color to mesure the angle of 
   * (e.g "R" for red, "B" for blue, etc.).
   * 
   * @return The angle of the input color on the control panel.
   */
  public double getColorAngle(String color) {
    switch (color) {
      case("R"):
        return 0;
      case("G"):
        return 45;
      case("B"):
        return 90;
      case("Y"):
        return 135;
      default:
        return -1;
    }
  }

  /**
   * Utilises a PID loop to rotate to a certain angle on the control panel.
   * 
   * @param degrees - the angle to rotate to (As the colors repeat after half a circle, 
   * this should be between 0 and 180).
   */
  public void spinTo(double degrees) {
    resetSensor();
    PIDController controller = getController();
    controller.setTolerance(GET_TOLERANCE());
    controller.setSetpoint(degrees);

    while (Math.abs(controller.getPositionError()) > GET_TOLERANCE()) {
      m_spinner.set(controller.calculate(getLeftAngle()));
    }

    m_spinner.set(0);
  }

  /**
   * Utilises a PID loop to spin the control panel a certain amount of degrees. 
   * @param degrees - the amount of degrees to rotate the control panel.
   */
  public void spinDegrees(double degrees) {
    resetSensor();
    PIDController controller = getController();
    controller.setTolerance(GET_TOLERANCE());
    controller.setSetpoint(degrees);

    while (Math.abs(controller.getPositionError()) > GET_TOLERANCE()) {
      m_spinner.set(controller.calculate(getRawAngle()));
    }

    m_spinner.set(0);

  }

  /**
   * @return A {@link CommandBase} representatoion of {@link #rotationControl()}
   */
  public PIDCommand getRotationControl() {
    return new PIDCommand(
      getController(), 
      this::getRawAngle,
      () -> ROTATION_CONTROL_SETPOINT, 
      (output) -> m_spinner.set(output),
      this);
  }

  /**
   * @return A {@link CommandBase} representatoion of {@link #positionControl()}
   */
  public PIDCommand getPositionControl() {

    resetSensor();

    double currentLeft = getLeftAngle();
    double currentRight = getRightAngle();
    double target = getTargetAngle();
    double rightError = target - currentRight;
    double leftError = toLeft(target) - currentLeft;

    if (rightError < leftError) {

      return new PIDCommand(
        getController(),
        this::getRightAngle, 
        this::getTargetAngle,
        (output) -> m_spinner.set(output), 
        this);
    }

    else {

      return new PIDCommand(
        getController(),
        this::getLeftAngle,
        () -> toLeft(getTargetAngle()),
        (output) -> m_spinner.set(output), 
        this);

    }
  }

  private double toLeft(double rightAngle) {
    return rightAngle % -180;
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Rolleta/isLifting", m_isLifting);
    SmartDashboard.putNumber("Rolleta/Lifter Speed", m_lifter.get());
    SmartDashboard.putNumber("Rolleta/Spinner Speed", m_spinner.get());
  }
}