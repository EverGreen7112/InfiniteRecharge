package com.evergreen.robot.wpilib;

import java.io.IOException;
import java.util.Set;

import com.evergreen.everlib.structure.Tree;
import com.evergreen.robot.RobotMap.ButtonPorts;
import com.evergreen.robot.RobotMap.CameraPorts;
import com.evergreen.robot.RobotMap.JoystickPorts;

import edu.wpi.cscore.CameraServerJNI;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.evergreen.robot.wpilib.Chassis.TrajectoryOption;
import com.evergreen.robot.wpilib.commands.FollowTrajectory;
import com.evergreen.robot.wpilib.commands.ResetGyro;
import com.evergreen.robot.wpilib.commands.RotateTilSeePort;
import com.evergreen.robot.wpilib.commands.Stop;
import com.evergreen.robot.wpilib.commands.TurnPowePortInfront;
import com.evergreen.robot.wpilib.subsystem.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * Robot
 */

public class Robot extends TimedRobot {
    boolean m_activate = true;
    boolean m_lifting = false;
    public static Joystick m_leftJoystick = new Joystick(JoystickPorts.leftChassisJS),
            m_righJoystick = new Joystick(JoystickPorts.rightChasisJS), m_operatorJoystick = new Joystick(2);
    private static UsbCamera m_backCamera;

    private static Button buttonX = new JoystickButton(m_operatorJoystick, 1);
    private static Button buttonA = new JoystickButton(m_operatorJoystick, 2);
    private static Button buttonB = new JoystickButton(m_operatorJoystick, 3);
    private static Button buttonY = new JoystickButton(m_operatorJoystick, 4);

    // Creates JS objects

    public static double getRightJoystick() {
        return m_righJoystick.getY();
    }

    public static double getLeftJoystick() {
        return m_leftJoystick.getY();
    }

    @Override
    public void testInit() {
        // Rolletta.getInstance().toggle().schedule();
        // new
        // JoystickButton(m_joystick,1).whileHeld(Collector.getInstance().collectCmd());
        // new JoystickButton(m_joystick,
        // 3).whileHeld(Collector.getInstance().collectCmd(-0.7));
        // Collector.getInstance().collectCmd().schedule();
        // Collector.getInstance().collectCmd().schedule();
        // CommandScheduler.getInstance().run();

        // System.out.println("PRINTING WHY");
        // Command cmd
        new JoystickButton(m_operatorJoystick, ButtonPorts.operatorJSY)
                .whenPressed(Rolletta.getInstance().m_calibrateYellow());
        new JoystickButton(m_operatorJoystick, ButtonPorts.operatorJSB)
                .whenPressed(Rolletta.getInstance().m_calibrateRed());
        new JoystickButton(m_operatorJoystick, ButtonPorts.operatorJSX)
                .whenPressed(Rolletta.getInstance().m_calibrateBlue());
        new JoystickButton(m_operatorJoystick, ButtonPorts.operatorJSA)
                .whenPressed(Rolletta.getInstance().m_calibrateGreen());

    }

    @Override
    public void testPeriodic() {
        Rolletta.getInstance().m_lifter.set(0.3);
        // new PrintCommand("message").schedule();
        CommandScheduler.getInstance().run();

    }

    @Override
    public void robotPeriodic() {
        // new PrintCommand("CHECK").schedule();
        // Preferences.getInstance().putDouble("AAAaAAAAa",
        // Utilites.getPowerPortToRobotAngle());
        Preferences.getInstance().putDouble("PP/distance2", Utilites.getDirectDistanceFromPowerPort());
        CommandScheduler.getInstance().run();

        Preferences.getInstance().putBoolean("PP/inRange",
                Preferences.getInstance().getDouble("PP/distance2", 0) > Preferences.getInstance()
                        .getDouble("PP/minDistance", -1000)
                        && Preferences.getInstance().getDouble("PP/distance2", 0) > Preferences.getInstance()
                                .getDouble("PP/maxDistance", 1000))

        ;
        // System.out.println("TEST");

        // System.out.println("REFLECTIVE CENTER " +
        // SmartDashboard.getNumber("Distance", -2));
    }

    @Override
    public void disabledInit() {
        new JoystickButton(m_operatorJoystick, ButtonPorts.operatorJSB)
                .whenPressed(new InstantCommand(Rolletta.getInstance()::calibrateRed));
        new JoystickButton(m_operatorJoystick, ButtonPorts.operatorJSA)
                .whenPressed(new InstantCommand(Rolletta.getInstance()::calibrateGreen));

        new JoystickButton(m_operatorJoystick, ButtonPorts.operatorJSX)
                .whenPressed(new InstantCommand(Rolletta.getInstance()::calibrateBlue));

        new JoystickButton(m_operatorJoystick, ButtonPorts.operatorJSX)
                .whenPressed(new InstantCommand(Rolletta.getInstance()::calibrateYellow));

    }

    @Override
    public void disabledPeriodic() {

        SmartDashboard.putNumber("BLUE", Rolletta.getInstance().getColorSensor().getBlue());

        SmartDashboard.putNumber("GREEN", Rolletta.getInstance().getColorSensor().getGreen());

        SmartDashboard.putNumber("RED", Rolletta.getInstance().getColorSensor().getRed());

    }

    @Override
    public void robotInit() {
        m_backCamera = CameraServer.getInstance().startAutomaticCapture();
        CommandScheduler.getInstance().registerSubsystem(Shooter.getInstance(), Chassis.getInstance(),
                Climb.getInstance(), Collector.getInstance(), Rolletta.getInstance(), Storage.getInstance());
        // Preferences.getInstance().putDouble("PP/Kp", 0);
        // Preferences.getInstance().putDouble("PP/Ki", 0);
        // Preferences.getInstance().putDouble("PP/Kd", 0);
        Preferences.getInstance().putDouble("PP/minDistance", 2.5);
        Preferences.getInstance().putDouble("PP/maxDistance", 3);

        Autonomous.getInstance();

        // Shooter.getInstance().m_thrower.m_motor.set(0.4);

    }
    public CommandBase passInitainLine = new CommandBase() {
        public void initialize(){
            Chassis.getInstance().move(-0.6);
        try {
            Thread.sleep(600);
        } catch (InterruptedException e) {
            e.printStackTrace();
            throw new RuntimeException();
        }
        Chassis.getInstance().move(0);
        }
    };
    public CommandBase shoot = new CommandBase() {
        public void initialize(){
            new RotateTilSeePort(true, 0.3).schedule();
            CommandBase turn =  Chassis.getInstance().turnToPPCmd();
            turn.schedule();
            try {
                Thread.sleep(1000);
            } catch (InterruptedException e) {
                e.printStackTrace();
                throw new RuntimeException();
            }
            //finish turn
            CommandBase throwPC = Shooter.getInstance().getShootToUpper();
            throwPC.schedule();
            try {
                Thread.sleep(4000);
            } catch (InterruptedException e) {
                e.printStackTrace();
                throw new RuntimeException();
            }
            //finish throw

            


        }
    };
    @Override
    public void autonomousInit() {
        

        try {
            new FollowTrajectory(TrajectoryOption.MOCK, true).schedule();
        } catch (IOException e1) {
            passInitainLine.schedule();
        }
        CommandScheduler.getInstance().schedule();


























        
        // try {
        //     Thread.sleep(6500);
        // } catch (InterruptedException e) {
        //     e.printStackTrace();
        // }
        // Shooter.getInstance().m_thrower.m_motor.set(0.3);
        // try {
        //     Thread.sleep(500);
        // } catch (InterruptedException e) {
        //     e.printStackTrace();
        // }
        // Storage.getInstance().m_passMotor.set(0.3);
        
        // try {
        //     Thread.sleep(5000);
        // } catch (InterruptedException e) {
        //     e.printStackTrace();
        // }
        Chassis.getInstance().move(-0.6);
        try{
        Thread.sleep(600);
        } catch (InterruptedException e) {
            e.printStackTrace();
            throw new RuntimeException();
        }
        Chassis.getInstance().move(0);

        
        // try{
        //     Shooter.getInstan22222222222222222222222222221ce().getAimDown().schedule();;
        //     CommandScheduler.getInstance().run();
        //     Thread.sleep(1000);
        //     Shooter.getInstance().getAimUp().schedule();;
        //     CommandScheduler.getInstance().run();
        //     Thread.sleep(200);
        //     //  Chassis.getInstance().turnToPPCmd().schedule();
        //     // new Stop().schedule();
        //     Shooter.getInstance().getAccelerateToThrow().schedule();;
        //     CommandScheduler.getInstance().run();
        //     Thread.sleep(3500);
        //     Storage.getInstance().getPass().schedule();;
        //     CommandScheduler.getInstance().run();
        //     Thread.sleep(8000);
        //     Chassis.getInstance().move(0.6);

        //     Thread.sleep(1000);
        // } catch (InterruptedException e) {
        //     e.printStackTrace();
        //     throw new RuntimeException();
        // }
        // Chassis.getInstance().move(0);
        
        
        

        // Chassis.getInstance().move(0.6);
        // try {
        //     Thread.sleep(1000);
        // } catch (InterruptedException e) {
        //     e.printStackTrace();
        //     throw new RuntimeException();
        // }
        // Chassis.getInstance().move(0);


        
        
    }

    @Override
    public void autonomousPeriodic() {
        CommandScheduler.getInstance().run();

        
    }
    //

    @Override
    public void teleopPeriodic() {
        CommandScheduler.getInstance().run();
        

    }

    @Override
    public void teleopInit() {
        Rolletta.getInstance().addAllColors();
        Chassis.getInstance().setDefaultCommand(Chassis.getInstance().getDefaultDrive()); // checed

        new JoystickButton(m_operatorJoystick, ButtonPorts.operatorJSY).whenPressed(Shooter.getInstance().getAimUp());
        new JoystickButton(m_operatorJoystick, ButtonPorts.operatorJSA).whenPressed(Shooter.getInstance().getAimDown());
        new JoystickButton(m_operatorJoystick, ButtonPorts.operatorJSLT)
                .whileHeld(Shooter.getInstance().getAccelerateToThrow());
        new JoystickButton(m_operatorJoystick, ButtonPorts.operatorJSRT).whileHeld(Storage.getInstance().getPass());
        new JoystickButton(m_operatorJoystick, ButtonPorts.operatorJSLB).whileHeld(Climb.getInstance().m_up());
        new JoystickButton(m_operatorJoystick, ButtonPorts.operatorJSRB).whileHeld(Climb.getInstance().getClimbDown());
        new JoystickButton(m_operatorJoystick, ButtonPorts.operatorJSX).whileHeld(Collector.getInstance().collectCmd());
        new JoystickButton(m_operatorJoystick, ButtonPorts.operatorJSB).whileHeld(Climb.getInstance().m_pull());
        new JoystickButton(m_operatorJoystick, ButtonPorts.operatorJSStart).whenPressed(Rolletta.getInstance().getRotationControl());
        new JoystickButton(m_operatorJoystick, ButtonPorts.operatorJSBack)
         .whenPressed(Rolletta.getInstance().getPositionControl());
        new JoystickButton(m_righJoystick, 5)
         .whenPressed(Chassis.getInstance().turnToPPCmd());
        new JoystickButton(m_righJoystick, 1).whileHeld(new CommandBase() {
            @Override
            public void initialize() {
                Chassis.getInstance().SpeedModifier = 1;
            }
            @Override
            public void end(boolean interrupted) {
                Chassis.getInstance().SpeedModifier = 0.5;
            }
        });
        // new JoystickButton(m_operatorJoystick, ButtonPorts.operatorJSRS).whileHeld(new CommandBase() {
        //     @Override
        //     public void initialize() {
        //         Shooter.getInstance().m_thrower.m_motor.set(0.1);
        //     }
        //     @Override
        //     public void end(boolean interrupted) {
        //         Shooter.getInstance().m_thrower.m_motor.set(0);
        //     }

        // });

        Rolletta.getInstance().getLiftTrigger().whileActiveOnce(Rolletta.getInstance().toggle());










        // new JoystickButton(m_operatorJoystick,
        // ButtonPorts.operatorJSY).whenPressed(Rolletta.getInstance().m_calibrateYellow());
        // new JoystickButton(m_operatorJoystick,
        // ButtonPorts.operatorJSB).whenPressed(Rolletta.getInstance().m_calibrateRed());
        // new JoystickButton(m_operatorJoystick,
        // ButtonPorts.operatorJSX).whenPressed(Rolletta.getInstance().m_calibrateBlue());
        // new JoystickButton(m_operatorJoystick,
        // ButtonPorts.operatorJSA).whenPressed(Rolletta.getInstance().m_calibrateGreen());
        // new JoystickButton(m_operatorJoystick,
        // ButtonPorts.operatorJSStart).whenPressed(new RunCommand(() ->
        // Rolletta.getInstance().addAllColors(),
        // Rolletta.getInstance()).withTimeout(3.5));



        // new JoystickButton(m_operatorJoystick,
        // ButtonPorts.operatorJSLT).whileHeld(Shooter.getInstance().getShooterSpeedControl());
        // new JoystickButton(m_righJoystick, 8).whileHeld(new InstantCommand(() ->
        // CommandScheduler.getInstance().cancelAll()));
        // new JoystickButton(m_leftJoystick, 8).whileHeld(new InstantCommand(() ->
        // CommandScheduler.getInstance().cancelAll()));

       
        // new JoystickButton(m_operatorJoystick,
        // ButtonPorts.operatorJSStart).whenPressed(Rolletta.getInstance().getRotationControl());
        // new JoystickButton(m_operatorJoystick,
        // ButtonPorts.operatorJSBack).whenPressed(Rolletta.getInstance().getPositionControl());
        // TODO: add climb down on RB

    };

}
