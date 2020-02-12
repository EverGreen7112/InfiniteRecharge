package com.evergreen.robot.wpilib;

import com.evergreen.robot.wpilib.subsystem.Shooter;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

/**
 * Utilites
 */
public class Utilites {
    public static final double GRAVITY_CONSTANT = 9.80665;
    /**
     * the time between acccelarate the shooter and pass the ball to the shooter in sec.
     */
    //TODO: tune
    public static final double TIME_TIL_SHOOTING = 0.3;
    private static CommandBase waitForShooting = new WaitCommand(TIME_TIL_SHOOTING);


    private static NetworkTable m_table = NetworkTableInstance.getDefault().getTable("Vision");
       
    public static double getDistanceFromPowerPort(){
        //TODO: ask vision about key
        return  m_table.getEntry("key").getDouble(0);
    }
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
    public static boolean isShootingToInnerWork(){
        return true;
    }
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