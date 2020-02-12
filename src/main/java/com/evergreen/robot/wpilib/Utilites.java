package com.evergreen.robot.wpilib;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * Utilites
 */
public class Utilites {
    private static NetworkTable m_table = NetworkTableInstance.getDefault().getTable("Vision");
       
    public static double getDistanceFromPowerPort(){
        //TODO: ask vision about key
        return  m_table.getEntry("key").getDouble(0);
    }
    public static final double GRAVITY_CONSTANT = 9.80665;
    
}