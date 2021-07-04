package com.evergreen.robot.utils;

import com.evergreen.robot.subsystem.Chassis;
import com.evergreen.robot.subsystem.Shooter;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.opencv.core.Mat;

public class Vision {

    private static final NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight");
    private static final double CAMERA_ANGLE = 14*(Math.PI/180);
    private static final double CAMERA_HEIGHT = 0.605;


    public static double getXDistanceFromPowerPort() {
        return getDirectDistanceFromPowerPort() * Math.sin(getPowerPortToAllinceStationAngle());
    }

    public static double getYDistanceFromPowerPort() {
        return getDirectDistanceFromPowerPort() * Math.cos(getPowerPortToAllinceStationAngle());
    }

    // By https://docs.limelightvision.io/en/latest/cs_estimating_distance.html
    public static double getDirectDistanceFromPowerPort() {
        double cameraToPP = getPitchAngle();
        double contourHeight = Shooter.PowerPorts.OUTER.getHeight();

        return (contourHeight - CAMERA_HEIGHT) / Math.tan(cameraToPP + CAMERA_ANGLE);
    }

    public static double getPowerPortToAllinceStationAngle() {
        return 90 - Chassis.getInstance().getAbsuluteAngle() + getPowerPortToRobotAngle();
    }

    public static double getPitchAngle() {
        return limelight.getEntry("ty").getDouble(-1);
    }


    /**
     * @return b angle, see whatsapp
     */
    public static double getPowerPortToRobotAngle() {
        return limelight.getEntry("tx").getDouble(-1);
    }

    public static boolean seePowerPort() {
        return SmartDashboard.getBoolean("SeePowerPort", false);
    }
}
