/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.evergreen.robot.wpilib;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.evergreen.robot.RobotMap.MotorPorts;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Chassis extends SubsystemBase {
  /**
   * Creates a new Chassis.
   */
private static Chassis m_instance;
   //declaring the front speed controllers (talon)
  private WPI_TalonSRX m_rightFront = new WPI_TalonSRX(MotorPorts.chassisRightFront);
  private WPI_TalonSRX m_leftFront = new WPI_TalonSRX(MotorPorts.chassisLeftFront);
  //declaring the gyro
  private Gyro m_gyro = new ADXRS450_Gyro();
  

  //declaring the other speed controllers
  private SpeedControllerGroup m_rightBack = new SpeedControllerGroup(new WPI_VictorSPX(MotorPorts.chassisRightBack), new WPI_VictorSPX(MotorPorts.chassisRightUp));
  private SpeedControllerGroup m_leftBack = new SpeedControllerGroup(new WPI_VictorSPX(MotorPorts.chassisLeftBack), new WPI_VictorSPX(MotorPorts.chassisLeftUp));
  //creating pid componets for angle velocity and distance
  private double 
    ANGLE_KP = 0,
    ANGLE_KI = 0,
    ANGLE_KD = 0,
    ANGLE_TOLERANCE = 1,

    VELOCITY_KP = 0,
    VELOCITY_KI = 0,
    VELOCITY_KD = 0,
    VELOCITY_TOLERANCE = 1,
    
    DISTANCE_KP = 0,
    DISTANCE_KI = 0,
    DISTANCE_KD = 0,
    DISTANCE_TOLERANCE = 1;
//creating pid controllers for angle velocity and distance
  private PIDController 
    m_anglePID = new PIDController(ANGLE_KP, ANGLE_KI, ANGLE_KD),
    m_velocityPID = new PIDController(VELOCITY_KP, VELOCITY_KI, VELOCITY_KD),
    m_distancePID = new PIDController(DISTANCE_KP, DISTANCE_KI, DISTANCE_KD);
  
 
  
  

  private Chassis() {
    //entering the PID componets into the prefernces
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
  }


  // creating a chassis object
  public static Chassis getInstance(){
    if (m_instance==null){
      m_instance = new Chassis();
    }
    return m_instance;
  }
 
//rotating chassis to a choosen set point with pid
  public void rotateTo(double setpoint){
    
    m_anglePID.setSetpoint(setpoint);
    m_anglePID.setTolerance(ANGLE_TOLERANCE);

    while (Math.abs(m_anglePID.getPositionError()) > getPIDAngleTolerance()) {
      double speed = m_anglePID.calculate(m_gyro.getAngle());
      m_rightFront.set(speed);
      m_rightBack.set(speed);
      m_leftFront.set(-speed);
      m_leftBack.set(-speed);
    }
  }
  //sets same speed to all left motors
  public void setLeftSpeed(double speed){
    m_leftBack.set(speed);
    m_leftFront.set(speed);
  }
  // sets same speed to all right motors
  public void setRightSpeed(double speed){
    m_rightBack.set(speed);
    m_rightFront.set(speed);
  }
  
//returning the angle kp
  public double getAngleKp(){
    return Preferences.getInstance().getDouble("Chassis/distance/KD", ANGLE_KP);
  }
//returning the angle ki
  public double getAngleKi(){
    return ANGLE_KI;
  }
//returning the angle kd
  public double getAngleKd() {
    return ANGLE_KD;
  }
//returning the velocity kp
  public double getVelocityKp() {
    return VELOCITY_KP;
  }
//returning the velocity ki
  public double getVelocityKi() {
    return VELOCITY_KI;
  }
//returning the velocity kd
  public double getVelocityKd() {
    return VELOCITY_KD;
  }
//returning the distance kp
  public double getDistanceKp() {
    return DISTANCE_KP;
  }
//returning the distance ki
  public double getDistanceKi() {
    return DISTANCE_KI;
  }
//returning the distance kd
  public double getDistanceKd() {
    return DISTANCE_KD;
  }
//returning the distance from left sensor
  public double getLeftDistance() {
    return m_leftFront.getSelectedSensorPosition();
  } 
//returning the distance from right sensor
  public double getRightDistance() {
    return m_rightFront.getSelectedSensorPosition();
  }
//returning the speed of the right motors
 public double getRightVelocity(){
   return m_rightFront.getSelectedSensorVelocity();
 }
 //returning the speed of the left motor
 public double getLeftVelocity(){
   return m_leftFront.getSelectedSensorVelocity();
 }
 // returning the average speed
 private double getVelocity(){
   return (getLeftVelocity()+getRightVelocity())/2;
 }
 //returning the average distance from both sensors
 public double getDistance(){
   return (getLeftDistance()+getRightDistance())/2;
 }
 //returning the angle tolerance
 private double getPIDAngleTolerance(){
   return ANGLE_TOLERANCE;
 }
 //returning the distance tolerance
 public double getPIDDistanceTolerance(){
  return DISTANCE_TOLERANCE;
}
//returning the velocity tolerance
private double getPIDVelocityTolerance(){
  return VELOCITY_TOLERANCE;
}
//returning the angle pid controller
public PIDController getAnglePID(){
  return m_anglePID;
}
//returning the distance pid controller
public PIDController getDistancePID(){
  return m_distancePID;
}
//returning the velocity pid controller
private PIDController getVelocityPID(){
  return m_velocityPID;
}
//returing the gyro object
public Gyro getGyro(){
  return m_gyro;
}
//returning the right talon mototr
public TalonSRX getRightTalonSRX(){
  return m_rightFront;
}
//returning the left talon motor
public TalonSRX getLefTalonSRX(){
  return m_leftFront;
}
//returning the right victor momtors
public SpeedControllerGroup getRightControllerGroup(){
  return m_rightBack;
}
//returning the left victor motors
public SpeedControllerGroup getLeftControllerGroup(){
  return m_leftBack;
}
//rotating the chassis in a certin speed
public void rotate(double speed){
  m_leftBack.set(speed);
  m_leftFront.set(speed);
  m_rightBack.set(-speed);
  m_rightFront.set(-speed);
}

//moving the entire chassis with a specific speed 
  public void move(double speed){
    
    m_leftBack.set(speed);
    m_leftFront.set(speed);
    m_rightBack.set(speed);
    m_rightFront.set(speed);
  } 
//moving the chassis with a specific speed for right and left motors
  public void drive(double speedR, double speedL){
    m_leftBack.set(speedL);
    m_leftFront.set(speedL);
    m_rightFront.set(speedR);
    m_rightBack.set(speedR);
  }
 
  
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
