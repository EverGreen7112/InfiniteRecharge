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
import com.evergreen.robot.wpilib.RobotMap.MotorPorts;

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
  private SpeedController rightFront = new WPI_TalonSRX(MotorPorts.chassisRightFront);
  private SpeedController leftFront = new WPI_TalonSRX(MotorPorts.chassisLeftFront);
  
  private Gyro gyro = new ADXRS450_Gyro();
  private Encoder leftChassis = new s

  //declaring the other speed controllers
  private SpeedControllerGroup rightBack = new SpeedControllerGroup(new WPI_VictorSPX(MotorPorts.chassisRightBack), new WPI_VictorSPX(MotorPorts.chassisRightUp));
  private SpeedControllerGroup leftBack = new SpeedControllerGroup(new WPI_VictorSPX(MotorPorts.chassisLeftBack), new WPI_VictorSPX(MotorPorts.chassisLeftUp));
  //creating pid componets for angle velocity and distance
  private double angleKp=0,angleKi=0,angleKd=0,velocityKp=0,velocityKi=0,velocityKd=0,distanceKp=0,distanceKi=0,distanceKd=0;
  private PIDController anglePID = new PIDController(angleKp, angleKi, angleKd);
  private PIDController velocityPID = new PIDController(velocityKp, velocityKi, velocityKd);
  private PIDController distancePID = new PIDController(distanceKp, distanceKi, distanceKd);
  
  

  private Chassis() {
    //entering the PID componets into the prefernces
    Preferences.getInstance().putDouble("Chassis/angle/KP", angleKp);
    Preferences.getInstance().putDouble("Chassis/angle/KI", angleKi);
    Preferences.getInstance().putDouble("Chassis/angle/KD", angleKd);
    Preferences.getInstance().putDouble("Chassis/velocity/KP", velocityKp);
    Preferences.getInstance().putDouble("Chassis/velocity/KI", velocityKi);
    Preferences.getInstance().putDouble("Chassis/velocity/KD", velocityKd);
    Preferences.getInstance().putDouble("Chassis/distance/KP", distanceKp);
    Preferences.getInstance().putDouble("Chassis/distance/KI", distanceKi);
    Preferences.getInstance().putDouble("Chassis/distance/KD", distanceKd);
  }
  public static Chassis getInstance(){
    if (m_instance==null){
      m_instance = new Chassis();
    }
    return m_instance;
  }
  public void move(double speed){
    
    leftBack.set(speed);
    leftFront.set(speed);
    rightBack.set(speed);
    rightFront.set(speed);
  } 
  public void drive(double speedR, double speedL){
    leftBack.set(speedL);
    leftFront.set(speedL);
    rightFront.set(speedR);
    rightBack.set(speedR);
  }
  public void turnLeft(double setpoint){
    anglePID.setSetpoint(setpoint);
    anglePID.setTolerance(1);
    rightFront.set();
    rightBack.set(speed);
  }
  public void turnRight(double speed){
    leftFront.set(speed);
    leftBack.set(speed);
  }
  public double getAngleKp(){
    return angleKp;
  }
  public double getAngleKi(){
    return angleKi;
  }
  public double getAngleKd(){
    return angleKd;
  }
  public double getVelocityKp(){
    return velocityKp;
  }
  public double getVelocityKi(){
    return velocityKi;
  }
  public double getVelocityKd(){
    return velocityKd;
  }
  public double getDistanceKp(){
    return distanceKp;
  }
  public double getDistanceKi(){
    return distanceKi;
  }
  public double getDistanceKd(){
    return distanceKd;
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
