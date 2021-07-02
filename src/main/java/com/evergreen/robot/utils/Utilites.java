package com.evergreen.robot.utils;

import com.evergreen.robot.subsystem.Chassis;
import com.evergreen.robot.subsystem.Storage;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    public static final double TIME_TIL_SHOOTING = 0.8;

    public static CommandBase waitForShooting() {
        return new WaitCommand(TIME_TIL_SHOOTING);
    }

    public static final int lowH = 0, lowS = 0, lowV = 0, highH = 0, highS = 0, highV = 0;
    //TODO: check
    public static final Pose2d POWER_PORT_POSE2D = new Pose2d(0, 2.38415, new Rotation2d(0));
    //TODO: fix ready for shoot, vison work angle getting

    /**
     * @param dX delta x
     * @param dY delta y
     * @return calculating hypotenuse according to pitagurs formua
     */
    public static double pythagoras(double dX, double dY) {
        return Math.sqrt(
                Math.pow(dX, 2) +
                        Math.pow(dY, 2)
        );
    }

    /**
     * @param accelerateCommand that used to accelerate the shooter
     * @return new command that first acelerate and then pass the ball to the shooter
     */
    public static CommandBase toFullShootingCommand(CommandBase accelerateCommand, CommandBase aimCommand) {
        return new SequentialCommandGroup(
                accelerateCommand,
                waitForShooting(),
                Storage.getInstance().passByTime());
    }
}