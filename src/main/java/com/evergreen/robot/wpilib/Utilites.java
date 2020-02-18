package com.evergreen.robot.wpilib;

import com.evergreen.robot.wpilib.subsystem.Shooter;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

/**
 * Utilites
 */
//Reset when at zero zero
 public class Utilites {
    public static final double GRAVITY_CONSTANT = -9.80665;
    /**
     * the time between acccelarate the shooter and pass the ball to the shooter in sec.
     */
    //TODO: tune
    //may cause recursive
    public static final double TIME_TIL_SHOOTING = 0.3;
    private static CommandBase waitForShooting = new WaitCommand(TIME_TIL_SHOOTING);
    //TODO: check
    public static final Pose2d POWER_PORT_POSE2D = new Pose2d(0, 0,new Rotation2d(0));
    //TODO: fix ready for shoot, vison work angle getting
    private static NetworkTable m_table = NetworkTableInstance.getDefault().getTable("Vision");
       
    public static double getXDistanceFromPowerPort(){
        if(isVisionDistanceWork()&&isVisonAngleWork()){
        return getDirectDistanceFromPowerPort()* Math.sin(getPowerPortToAllinceStationAngle());
        }
        return getPose().getTranslation().getX()-POWER_PORT_POSE2D.getTranslation().getX();
    }
    public static double getYDistanceFromPowerPort(){
       if(isVisionDistanceWork()&&isVisonAngleWork()){
        return getDirectDistanceFromPowerPort()* Math.cos(getPowerPortToAllinceStationAngle());
       }
       return getPose().getTranslation().getY()-POWER_PORT_POSE2D.getTranslation().getY();
    }
    public static double getDirectDistanceFromPowerPort(){
       if(isVisionDistanceWork()){
        return m_table.getEntry("Distance").getDouble(0);
       }
       
       return getPose().getTranslation().getDistance(POWER_PORT_POSE2D.getTranslation());
       
    }
    
    /**
     * @return a angle in whatsapp 
     */
    public static double getPowerPortToAllinceStationAngle(){
        if(isVisonAngleWork()){
            return 90 -Chassis.getInstance().getGyro().getAngle()+getPOwerPortToRobotAngle();
        }
        return Math.asin(getXDistanceFromPowerPort()/getDirectDistanceFromPowerPort());
    }
    /**
     * 
     * @return b angle, see whatsapp
     */
    public static double getPOwerPortToRobotAngle(){ 
        if(isVisonAngleWork()){
        return m_table.getEntry("Angle").getDouble(0);
        }
        return Chassis.getInstance().getGyro().getAngle()- Math.atan(getYDistanceFromPowerPort()/getXDistanceFromPowerPort());
    }
    
    public static boolean seePowerPort(){
        return m_table.getEntry("SeePowerPort").getBoolean(false);
    }
    public static Pose2d getPose(){
        return Chassis.getInstance().getOdometry().getPoseMeters();
    }
    /**
     * 
     * @param dX delta x
     * @param dY delta y
     * @return calculating hypotenuse according to pitagurs formua
     */
    public static double Pitaguras(double dX, double dY){
        return Math.sqrt(
            Math.pow(dX, 2) +
            Math.pow(dY, 2)
        );
    }
    /////////////////////////////is work///////////////////////////////
    /**
     * 
     * @return if we checked the passBySensor command and it work.
     */
    //TODO: check
    public static boolean isPassBySensorWork(){
        return true;
    }
    /**
     * @return we are want to shoot to the inner  true, else change it to false; 
     */
    //TODO: check
    public static boolean isShootingToInnerWork(){
        return true;
    }
    //TODO: check
    public static boolean isVisionDistanceWork(){
        if(isVisonWork()){
        return true;
        }
        return false;
    }
    //TODO: check
    public static boolean isVisonAngleWork(){
        if(isVisonWork()){
            return true;
            }
            return false;
    }
    //TODO: check
    public static boolean isVisonWork(){
        return m_table.getEntry("CameraInput").getBoolean(true);
    }
    /////////////////////////////////////////////////////////////////
    /**
     * 
     * @param accelerateCommand that used to accelerate the shooter
     * @return new command that first acelerate and then pass the ball to the shooter
     */
    public static CommandBase toFullShootingCommand(CommandBase accelerateCommand){
        if(isPassBySensorWork()){
            return new SequentialCommandGroup(
                accelerateCommand,waitForShooting,Storage.getInstance().passBySensor);
        }
        return new SequentialCommandGroup(
                accelerateCommand,waitForShooting,Storage.getInstance().passByTime);
    }
    
}