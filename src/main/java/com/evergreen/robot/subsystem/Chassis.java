/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.evergreen.robot.subsystem;

import java.io.IOException;
import java.nio.file.Path;
import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.evergreen.robot.Pistachio;
import com.evergreen.robot.utils.Utilites;
import com.evergreen.robot.utils.RobotMap.DigitalPorts;
import com.evergreen.robot.utils.RobotMap.MotorPorts;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Filesystem;


import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


/**The CHASSIS - the robot's base and driving subsystem. 
 * <p>
 * Contained here are all methods for driving, rotating, or at all otherwise changing the position
 * of the robot itself.
 * <p>
 * Pistachio's chassis is a drivetrain - having a left and right side controlled independently, 
 * rotating clockwise with left side and clockwise with the right, and driving forward using them both.
 * <p>
 * Specifically, Pistachio's chassis hass three motors on each side, each connected to a speed controller with the middle 
 * one being a {@link WPI_TalonSRX TalonSRX}. Each side has an external encoder connected to the roborio's DIO ports, and
 * its angle can be determined using the {@link ADXRS450_Gyro ADXRS450 Gyro}, connected to the roborio's CS0.   
 */
public class Chassis extends SubsystemBase {
  
  /**
   * The single instance of the chassis. for more detail see the {@link #getInstance()} documentation.
   */
  private static Chassis m_instance;
  
  
  //=====================================MOTORS=====================================
  /**The talon controller of the Right-Middle motor */
  private WPI_TalonSRX m_rightTalon = new WPI_TalonSRX(MotorPorts.chassisRightMiddle);
  
  /**The talon controller of the Left-Middle motor */
  private WPI_TalonSRX m_leftTalon = new WPI_TalonSRX(MotorPorts.chassisLeftMiddle);
  
  /**The Victor end */
  private SpeedControllerGroup m_rightVictors = new SpeedControllerGroup(new WPI_VictorSPX(MotorPorts.chassisRightBack), new WPI_VictorSPX(MotorPorts.chassisRightFront) );
  private SpeedControllerGroup m_leftVictors = new SpeedControllerGroup(new WPI_VictorSPX(MotorPorts.chassisLeftBack), new WPI_VictorSPX(MotorPorts.chassisLeftFront));
  //================================================================================
  
  //====================================SENSORS=====================================
  private Gyro m_gyro = new ADXRS450_Gyro();
  private Encoder m_rightEncoder = new Encoder(DigitalPorts.rightEncoderA, DigitalPorts.rightEncoderB);
  private Encoder m_leftEncoder = new Encoder(DigitalPorts.leftChassisEncoderA, DigitalPorts.leftChassisEncoderB);
  public Encoder getRightEncoder(){
    return m_rightEncoder;
  }
  //================================================================================
  
  
  
  //===============================CONTROL CONSTANTS================================
  //=======================PID Constants========================
  private double 
  ANGLE_KP = 0.016,
  ANGLE_KI = 0.01,
  ANGLE_KD = 0.093,
  ANGLE_TOLERANCE = 1.7,
  
  VELOCITY_KP = 0,
  VELOCITY_KI = 0,
  VELOCITY_KD = 0,
  VELOCITY_TOLERANCE = 1,
  
  DISTANCE_KP = 0.15,
  DISTANCE_KI = 0.015,
  DISTANCE_KD = 0.12,
  DISTANCE_TOLERANCE = 0.04;
  //================================================

  //==================Command Constants=============
  /**The minimum distance from the power port in which we are able to hit the outer target. */
  private double MIN_THROW_DISTANCE = 2.2;
  
  /**The maximum distance from the power port in which we are able to hit the outer target. */
  private double MAX_THROW_DISTANCE = 2.7;
  
  /**The multiplier of the joystick's value, in default drive.*/
  private double SPEED_MODIFIER = 0.5;

  /**
   * The amount of time, in milliseconds, the chassis will be moved during the
   * {@link #moveByTimeCMD()}. 
   * <p>
   * <b> Note that this is not at all precise; 
   * At the end of the command, <i>acceleration</i> will be 0, but not
   * necessarily speed.</b>
   */
  private double MOVE_BY_TIME_PERIOD = 2000;

  /**
   * The percentage speed that the chassis will move at during the 
   * {@link #moveByTimeCMD()}.
   * */
  private double MOVE_BY_TIME_VELOCITY = 0.7;
  //================================================
  
  //==================Profiling Constants=============
  private final double 
  CHASSIS_WIDTH = 0.54,
  kS = 0,
  kV = 0,
  kA = 0,
  MAX_VELOCITY = 0,
  MAX_ACCELERATION = 0;
  //==================================================
  
  //================================================================================
  
  
  //================================CONTORL OBJECTS========================================
  //==================PID Controllers=============
  private PIDController 
    m_anglePIDController = new PIDController(ANGLE_KP, ANGLE_KI, ANGLE_KD),
    m_velocityPIDController = new PIDController(VELOCITY_KP, VELOCITY_KI, VELOCITY_KD),
    m_distancePIDController = new PIDController(DISTANCE_KP, DISTANCE_KI, DISTANCE_KD);
  //==============================================
  
  //==================Profiling Components=============
  /**The kinematics calculator for seperating the chassis 
   * needed speed vector into its seperate sides.*/
  private DifferentialDriveKinematics m_kinematics = new DifferentialDriveKinematics(CHASSIS_WIDTH);
  
  /**The position calculator, using the {@link #m_leftEncoder left} and {@link #m_rightEncoder right} encoders
   * and the {@link #m_gyro gyro} to determine the x-y position of the robot and its angle at each moment.
   */
  private DifferentialDriveOdometry m_odometry;
  
  /**The voltage calculator, using found kS, kV and kA constants to compute the voltage needed
   * to achieve a specific speed in the robot's wheels.
   */
  private SimpleMotorFeedforward m_feedorward = new SimpleMotorFeedforward(kS, kV, kA); 
  
  /**A configuration of the trajectory - just hiow fast can we allow the robot to move? */
  private TrajectoryConfig m_trajectoryConfig = new TrajectoryConfig(MAX_VELOCITY, MAX_ACCELERATION);
  
  /**A complex feedback loop calculatpr which achieved desired values (much like PID, except more precise.) */
  private RamseteController m_ramseteController = new RamseteController();
  //==================================================

  //==========================================================================================


  
  /**
   * @return the kinematics calculator for seperating the chassis needed speed vector into its seperate sides, as a 
   * {@link DifferentialDriveKinematics} object.
   * */
  

  public Encoder getLeftEncoder() {
    return m_leftEncoder;
  }

  /**@return the chassis' {@link DifferentialDriveOdometry} object -
   *  The position calculator, using the {@link #m_leftEncoder left} and {@link #m_rightEncoder right} encoders
   * and the {@link #m_gyro gyro} to determine the x-y position of the robot and its angle at each moment.
   */
  public DifferentialDriveOdometry getOdometry(){
    return m_odometry;
  }
  
  
  //==================================CONSTRUCTORS==================================
  /**
   * The private, single constructor of the chassis (see {@link #getInstance()}). 
   * The constructor configures the motors and sensors, and puts constants on the
   * shuffleboard. 
   */
  private Chassis() {
    
    //--------Component Configuration---------  
    m_rightVictors.setInverted(true);
    m_rightTalon.setInverted(true);
    
    m_gyro.calibrate();
    m_rightEncoder.setDistancePerPulse(0.002699259);
    m_leftEncoder.setDistancePerPulse(0.002699259);
    m_rightEncoder.setReverseDirection(true);
    m_distancePIDController.setTolerance(DISTANCE_TOLERANCE);
    
    //-------Preferences Initialization-------
    Preferences.getInstance().putDouble("Chassis/angle/KP", ANGLE_KP);
    Preferences.getInstance().putDouble("Chassis/angle/KI", ANGLE_KI);
    Preferences.getInstance().putDouble("Chassis/angle/KD", ANGLE_KD);
    Preferences.getInstance().putDouble("Chassis/angle/TOLERANCE", ANGLE_TOLERANCE);
    Preferences.getInstance().putDouble("Chassis/velocity/KP", VELOCITY_KP);
    Preferences.getInstance().putDouble("Chassis/velocity/KI", VELOCITY_KI);
    Preferences.getInstance().putDouble("Chassis/velocity/KD", VELOCITY_KD);
    Preferences.getInstance().putDouble("Chassis/velocity/TOLERANCE", VELOCITY_TOLERANCE);
    Preferences.getInstance().putDouble("Chassis/distance/KP", DISTANCE_KP);
    Preferences.getInstance().putDouble("Chassis/distance/KI", DISTANCE_KI);
    Preferences.getInstance().putDouble("Chassis/distance/KD", DISTANCE_KD);
    Preferences.getInstance().putDouble("Chassis/distance/TOLERANCE", VELOCITY_TOLERANCE);
    Preferences.getInstance().putDouble("Chassis/Speed", 0.8);
    //----------------------------------------
    
  }
  public double SpeedModifier = 1;
  
    //====================================GETTERS=====================================
    //==========================Instance==========================
    /**
     * There is only one physical chassis - so we'd really only ever want to 
     * create one chassis. Fittingly, if we tried, errors would be thrown as 
     * we would try to create more objects from the same ports, and distaters 
     * would wait to happen as we could try to tell the chassis to move forward 
     * and backward, turn and turn right, all at the same time.
     * <p>
     * To avoid this, we made the {@link #Chassis() constructor} private, 
     * and created a single, static instance, within the class itself. 
     * This method is its getter - and the only way to access an object 
     * of the chassis.
     * @return The single instance of the chassis. 
     */
    public static Chassis getInstance() {
      if (m_instance == null) m_instance = new Chassis();
      return m_instance;
    }
    //============================================================
  
    //=========================Profiling==========================
    /**
     * @return the kinematics calculator for seperating the chassis needed speed vector into its seperate sides, as a 
     * {@link DifferentialDriveKinematics} object.
     * */
    public DifferentialDriveKinematics getKinematics() {
      return m_kinematics;
    }
  
    /**@return the chassis' {@link DifferentialDriveOdometry} object, needed for trajectory tracking -
     * The position calculator, using the {@link #m_LeftEncoder left} and {@link #m_rightEncoder right} encoders
     * and the {@link #m_gyro gyro} to determine the x-y position of the robot and its angle at each moment.
     */
   
    
    /**
     * @return The {@link SimpleMotorFeedforward feedforward} component of the motion profiling, using 
     * found constants (kS, kV, kA) to predict the required voltage for a desored speed setpoint.
     */
    public SimpleMotorFeedforward getFeedForword() {
      return m_feedorward;
    }
    
    /**
     * @return The speed and acceleration configuration for our motion profiling.
     * <i>Deprecated - redundant with PathWeaver.</i> 
     */
    @Deprecated
    public TrajectoryConfig getTrajectoryConfig() {
      return m_trajectoryConfig;
    }
    
    /**
     * @return The {@link RamseteController} calculator for the motion profiling,
     * using a complex feedback loop to compute required motor input and achieve
     * desired speed.  
     */
    public RamseteController getRamseteController() {
      return m_ramseteController;
    }
    //============================================================
    
    //=========================Constants==========================
    //==================PID===================
    //=======Angle========
    /**
     * @return The kP constant of the chassis' angle PID feedback loop,
     * accounting for its proportional component.
     */
    public double getAngleKP(){
      return Preferences.getInstance().getDouble("Chassis/distance/KD", ANGLE_KP);
    }
  
    /**
     * @return The kI Constant of the chassis' angle PID feedback loop,
     * accounting for its integral component.
     */
    public double getAngleKI(){
      return ANGLE_KI;
    }
    
    /**
     * @return The kD Constant of the chassis' angle PID feedback loop,
     * accounting for its derivative component.
     */
    public double getAngleKD() {
      return ANGLE_KD;
    }
    //====================

    //======Velocity======
    /**
     * @return The kP Constant of the chassis' velocity PID feedback loop,
     * accounting for its proportional component.
     */
    public double getVelocityKP() {
      return VELOCITY_KP;
    }
    
    /**
     * @return The kI Constant of the chassis' velocity PID feedback loop,
     * accounting for its integral component.
     */
    public double getVelocityKI() {
      return VELOCITY_KI;
    }
    
    /**
     * @return The kD Constant of the chassis' velocity PID feedback loop,
     * accounting for its derivative component.
     */
    public double getVelocityKD() {
      return VELOCITY_KD;
    }
    //====================
  
    //======Distance======
    /**
     * @return The kP Constant of the chassis' distance PID feedback loop,
     * accounting for its proportional component.
     */
    public double getDistanceKP() {
      return DISTANCE_KP;
    }
    
    /**
     * @return The kI Constant of the chassis' angle PID feedback loop,
     * accounting for its derivative component.
     */
    public double getDistanceKI() {
      return DISTANCE_KI;
    }
    
    /**
     * @return The kD constnat of the chassis' distance PID feedback loop, 
     * accounting fot its derivative component.
     */
    public double getDistanceKD() {
    return DISTANCE_KD;
    }
    //====================
    //========================================
    //============================================================
    //================================================================================
    
  
  //====================================SETTERS=====================================
  //===========================Motors===========================
  /**
   * Sets the left side of the chassis moving at an input percentage speed.
   * @param speed - the speed to move the chassis' left side 
   * <p>
   * The speed should be A percentage between -1 to 1, with the sign inidicating direction
   * and the number what percentage of thew maximum voltage output
   * the motors.)
   */
  public void setLeftSpeed(double speed){
    m_leftVictors.set(speed);
    m_leftTalon.set(speed);
  }
  
  /**
   * Sets the right side of the chassis moving at an input percentage speed.
   * @param speed - the speed to move the chassis' left side 
   * <p>
   * The speed should be A percentage between -1 to 1, with the sign inidicating direction
   * and the number what percentage of thew maximum voltage output
   * the motors.)
   */
  public void setRightSpeed(double speed){
    m_rightVictors.set(speed);
    m_rightTalon.set(speed);
  }
  //============================================================

  //==========================Commands==========================
  /**
   * @return A command which moves the chassis for a certain amount
   * of milliseconds. <p>
   * The speed and duration of this command can be determined 
   * at the chassis constants {@link #MOVE_BY_TIME_VELOCITY} and
   *  {@link #MOVE_BY_TIME_PERIOD}, respectively.
   */
  public CommandBase moveByTimeCMD() {
    return new CommandBase() {
      double m_start;
      
      @Override
      public void initialize() {
        m_start = System.currentTimeMillis();  
      }
      
      @Override
      public void execute() {
        move(MOVE_BY_TIME_VELOCITY);
      }
      
      @Override
      public boolean isFinished() {
        return System.currentTimeMillis() - m_start > MOVE_BY_TIME_PERIOD;
      }
      
      @Override
      public void end(boolean interrupted) {
        move(0);
      }
      
    };
  }
  //============================================================
  
  //returning the distance from left sensor
  public double getLeftDistance() {
    return m_leftEncoder.getDistance();
  } 
  //returning the distance from right sensor
  public double getRightDistance() {
    return m_rightEncoder.getDistance();
  }
//returning the speed of the right motors
public double getRightVelocity(){
  return m_rightEncoder.getRate();
}
//returning the speed of the left motor
 public double getLeftVelocity(){
   return m_leftEncoder.getRate();
 }
 // returning the average speed
 public DifferentialDriveWheelSpeeds getVelocity() {
   return new DifferentialDriveWheelSpeeds(
     m_leftEncoder.getRate(), 
     m_rightEncoder.getRate());
    }
    
    //returning the average distance from both sensors
    public double getDistance(){
      return getRightDistance();
    }
    
    //returning the angle tolerance
    @SuppressWarnings("unused")
    private double getPIDAngleTolerance(){
      return ANGLE_TOLERANCE;
    }
    //returning the distance tolerance
    public double getPIDDistanceTolerance(){
      return DISTANCE_TOLERANCE;
    }
    //returning the velocity tolerance
    
    @SuppressWarnings("unused")
    private double getPIDVelocityTolerance(){
      return VELOCITY_TOLERANCE;
    }
    //returning the angle pid controller
    public PIDController getAnglePID(){
      return m_anglePIDController;
    }
    //returning the distance pid controller
    public PIDController getDistancePID(){
      return m_distancePIDController;
    }
    //returning the velocity pid controller
    @SuppressWarnings("unused")
    private PIDController getVelocityPID(){
      return m_velocityPIDController;
    }
    public void resetGyro(){
      m_gyro.reset();
    }
    public void calibrateGyro(){
      m_gyro.calibrate();
    }
    //returning the right talon mototr
    public WPI_TalonSRX getRightTalonSRX(){
      return m_rightTalon;
    }
    
    //returning the left talon motor
    public WPI_TalonSRX getLefTalonSRX(){
      return m_leftTalon;
    }
    //returning the right victor momtors
    public SpeedControllerGroup getRightControllerGroup(){
      return m_rightVictors;
    }
    //returning the left victor motors
public SpeedControllerGroup getLeftControllerGroup(){
  return m_leftVictors;
}

  public void testMotorsPeriodic() {
    
    if (Pistachio.m_operatorJoystick.getRawButton(1))
    new WPI_VictorSPX(MotorPorts.chassisLeftBack).set(0.25);
  else
      new WPI_VictorSPX(MotorPorts.chassisLeftBack).set(0);

    
  if (Pistachio.m_operatorJoystick.getRawButton(2))
    new WPI_VictorSPX(MotorPorts.chassisLeftFront).set(0.25);
  else
      new WPI_VictorSPX(MotorPorts.chassisLeftFront).set(0);

  if (Pistachio.m_operatorJoystick.getRawButton(3))
       new WPI_VictorSPX(MotorPorts.chassisRightBack).set(0.25);
  else
      new WPI_VictorSPX(MotorPorts.chassisRightBack).set(0);

  if (Pistachio.m_operatorJoystick.getRawButton(4))
    new WPI_VictorSPX(MotorPorts.chassisRightFront).set(0.25);
  else
      new WPI_VictorSPX(MotorPorts.chassisRightFront).set(0);

  
  if (Pistachio.m_operatorJoystick.getRawButton(5))
    new WPI_TalonSRX(MotorPorts.chassisLeftMiddle).set(0.25);
  else
      new WPI_TalonSRX(MotorPorts.chassisLeftMiddle).set(0);
    
  if (Pistachio.m_operatorJoystick.getRawButton(6))
    new WPI_TalonSRX(MotorPorts.chassisRightMiddle).set(0.25);
  else
      new WPI_TalonSRX(MotorPorts.chassisRightMiddle).set(0);
  } 

//rotating the chassis in a certin speed
public void rotate(double speed){
  
  m_leftVictors.set(speed);
  m_leftTalon.set(speed);
  m_rightVictors.set(-speed);
  m_rightTalon.set(-speed);
}

//moving the entire chassis with a specific speed 
  public void move(double speed){
    
    m_leftVictors.set(speed);
    m_leftTalon.set(speed);
    m_rightVictors.set(speed);
    m_rightTalon.set(speed);
  } 
  
  //moving the chassis with a specific speed for right and left motors
  public void drive(double speedR, double speedL){
    m_leftVictors.set(speedL);
    m_leftTalon.set(speedL);
    m_rightTalon.set(speedR);
    m_rightVictors.set(speedR);
  }

  //TODO call this
  public void initOdometry(double x, double y) {
    m_odometry = new DifferentialDriveOdometry(
      getHeading(), new Pose2d(x, y, getHeading()));
  }

  public Rotation2d getHeading() {
    return Rotation2d.fromDegrees(-m_gyro.getAngle());
  }
 
  public void setVoltage(double left, double right) {
    drive(left / 12, right / 12);
  }
  /**
   * 
   * @return the gyro angle modulo 360
   */
  public double getAbsuluteAngle(){
    return m_gyro.getAngle()%360;
  }
  /**
   * 
   * @return the gyro angle(continue
   * from 360 to 361 degrees)
   */
  public double getOriginalAngle(){
    return m_gyro.getAngle();
  }

  public RamseteCommand follow(TrajectoryOption trajectory) throws IOException {//have Not check yet
    return
      new RamseteCommand(
        trajectory.getTrajectory(), 
        m_odometry::getPoseMeters, 
        m_ramseteController, 
        m_feedorward, 
        m_kinematics, 
        this::getVelocity,
        new PIDController(getDistanceKP(), getDistanceKI(), getDistanceKD()), 
        new PIDController(getDistanceKP(), getDistanceKI(), getDistanceKD()), 
        this::setVoltage, 
        this);

    
  }


  public static enum TrajectoryOption {
    MOCK("Mock");

    String m_name;

    private TrajectoryOption(String name) {
      m_name = name;
    }

    public String getName() {
      return m_name;
    }

    public Trajectory getTrajectory() throws IOException {
      Path trajectoryJson = Filesystem.getDeployDirectory().toPath().resolve("paths/" + m_name + ".wpilib.json");
      return TrajectoryUtil.fromPathweaverJson(trajectoryJson);
      
    }
  }

  public double getSpeedModifier() {
    return SPEED_MODIFIER;
  }
  
  public CommandBase turnToPowerPortCMD() {
    
   Supplier<Double> something = () ->Preferences.getInstance().getDouble("PP/somthing", 0);
    return new PIDCommand(
      new PIDController(
        ANGLE_KP,
        ANGLE_KI,
        ANGLE_KD),
      Utilites::getPowerPortToRobotAngle,
      0.0,
      this::rotate,
      this
    ) { 
          public void execute() {
            super.execute();
            SmartDashboard.putNumber(
              "PID OUTPUT",
              getController().calculate(Utilites.getPowerPortToRobotAngle()));
          }

          public boolean isFinished(){
              return (Pistachio.m_righJoystick.getRawButtonPressed(6) || !Utilites.seePowerPort());
          }
          
          public void end(boolean Interrupted) {
              drive(0, 0);
          }
      };
  }

  public Command defaultDriveCMD() {
    return new RunCommand(
      () -> drive(-Pistachio.getRightJoystick() * getSpeedModifier(), -Pistachio.getLeftJoystick() * getSpeedModifier()), 
      this);
  }
  
  public double getAngle() {
    return m_gyro.getAngle();
  }

  public WPI_TalonSRX getLeftTalon() {
    return m_leftTalon;
  }
  @Override
  public void periodic() {
    SmartDashboard.putBoolean(
      "CanThrow", 
      Utilites.getDirectDistanceFromPowerPort() < MAX_THROW_DISTANCE
        && Utilites.getDirectDistanceFromPowerPort() > MIN_THROW_DISTANCE);

  }
}