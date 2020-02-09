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

<<<<<<< HEAD

=======
>>>>>>> wpilib_chassis
public class Chassis extends SubsystemBase {
  /**
   * Creates a new Chassis.
   */
private static Chassis m_instance;
   //declaring the front speed controllers (talon)
<<<<<<< HEAD
  private WPI_TalonSRX m_rightFront = new WPI_TalonSRX(MotorPorts.chassisRightFront);
  private WPI_TalonSRX m_leftFront = new WPI_TalonSRX(MotorPorts.chassisLeftFront);
  
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

  private PIDController 
    m_anglePID = new PIDController(ANGLE_KP, ANGLE_KI, ANGLE_KD),
    m_velocityPID = new PIDController(VELOCITY_KP, VELOCITY_KI, VELOCITY_KD),
    m_distancePID = new PIDController(DISTANCE_KP, DISTANCE_KI, DISTANCE_KD);
=======
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
>>>>>>> wpilib_chassis
  
  

  private Chassis() {
    //entering the PID componets into the prefernces
<<<<<<< HEAD
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

=======
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
>>>>>>> wpilib_chassis
  public static Chassis getInstance(){
    if (m_instance==null){
      m_instance = new Chassis();
    }
    return m_instance;
  }
<<<<<<< HEAD

  public void move(double speed){
    
    m_leftBack.set(speed);
    m_leftFront.set(speed);
    m_rightBack.set(speed);
    m_rightFront.set(speed);
  } 
  public void drive(double speedR, double speedL){
    m_leftBack.set(speedL);
    m_leftFront.set(speedL);
    m_rightFront.set(speedR);
    m_rightBack.set(speedR);
  }

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
  public void setLeftSpeed(double speed){
    m_leftBack.set(speed);
    m_leftFront.set(speed);
  }
  public void setRightSpeed(double speed){
    m_rightBack.set(speed);
    m_rightFront.set(speed);
  }
  

  public double getAngleKp(){
    return Preferences.getInstance().getDouble("Chassis/distance/KD", ANGLE_KP);
  }

  public double getAngleKi(){
    return ANGLE_KI;
  }

  public double getAngleKd() {
    return ANGLE_KD;
  }

  public double getVelocityKp() {
    return VELOCITY_KP;
  }

  public double getVelocityKi() {
    return VELOCITY_KI;
  }

  public double getVelocityKd() {
    return VELOCITY_KD;
  }

  public double getDistanceKp() {
    return DISTANCE_KP;
  }

  public double getDistanceKi() {
    return DISTANCE_KI;
  }

  public double getDistanceKd() {
    return DISTANCE_KD;
  }

  public double getLeftDistance() {
    return m_leftFront.getSelectedSensorPosition();
  } 

  public double getRightDistance() {
    return m_rightFront.getSelectedSensorPosition();
  }

 public double getRightVelocity(){
   return m_rightFront.getSelectedSensorVelocity();
 }
 public double getLeftVelocity(){
   return m_leftFront.getSelectedSensorVelocity();
 }
 private double getVelocity(){
   return (getLeftVelocity()+getRightVelocity())/2;
 }
 public double getDistance(){
   return (getLeftDistance()+getRightDistance())/2;
 }
 private double getPIDAngleTolerance(){
   return ANGLE_TOLERANCE;
 }
 public double getPIDDistanceTolerance(){
  return DISTANCE_TOLERANCE;
}
private double getPIDVelocityTolerance(){
  return VELOCITY_TOLERANCE;
}
public PIDController getAnglePID(){
  return m_anglePID;
}
public PIDController getDistancePID(){
  return m_distancePID;
}
private PIDController getVelocityPID(){
  return m_velocityPID;
}
public Gyro getGyro(){
  return m_gyro;
}
public TalonSRX getRightTalonSRX(){
  return m_rightFront;
}
public TalonSRX getLefTalonSRX(){
  return m_leftFront;
}
public SpeedControllerGroup getRightControllerGroup(){
  return m_rightBack;
}
public SpeedControllerGroup getLeftControllerGroup(){
  return m_leftBack;
}
public void rotate(double speed){
  m_leftBack.set(speed);
  m_leftFront.set(speed);
  m_rightBack.set(-speed);
  m_rightFront.set(-speed);
}


=======
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
>>>>>>> wpilib_chassis
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
