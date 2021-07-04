package com.evergreen.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.evergreen.robot.RobotMap.ButtonPorts;
import com.evergreen.robot.RobotMap.JoystickPorts;
import com.evergreen.robot.RobotMap.MotorPorts;
import com.evergreen.robot.commands.chassisutils.MoveChassisTo;
import com.evergreen.robot.commands.pid.RotateTo;
import com.evergreen.robot.commands.sensor.RotateTilSeePort;
import com.evergreen.robot.commands.util.Autonomous;
import com.evergreen.robot.commands.util.CheckSpeedControllers;
import com.evergreen.robot.subsystem.Chassis;
import com.evergreen.robot.subsystem.Climb;
import com.evergreen.robot.subsystem.Collector;
import com.evergreen.robot.subsystem.Rolletta;
import com.evergreen.robot.subsystem.Shooter;
import com.evergreen.robot.subsystem.Storage;
import com.evergreen.robot.utils.Utilites;

import com.evergreen.robot.utils.Vision;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;


/**
 * Robot
 */

public class Pistachio extends TimedRobot {
    boolean m_activate = true;
    boolean m_lifting = false;
    public static Joystick 
        m_righJoystick = new Joystick(JoystickPorts.rightChasisJS), 
        m_leftJoystick = new Joystick(JoystickPorts.leftChassisJS),
        m_operatorJoystick = new Joystick(2);

    // Creates JS objects

    public static double getRightJoystick() {
        return m_righJoystick.getY();
    }

    public static double getLeftJoystick() {
        return m_leftJoystick.getY();
    }

    @Override
    public void testInit() {
        new MoveChassisTo(1).schedule();
        // new JoystickButton(m_operatorJoystick, ButtonPorts.operatorJSA).whileHeld(new CheckSpeedControllers());
        
        
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
        // new JoystickButton(m_operatorJoystick, ButtonPorts.operatorJSY)
        //         .whenPressed(Rolletta.getInstance().m_calibrateYellow());
        // new JoystickButton(m_operatorJoystick, ButtonPorts.operatorJSB)
        //         .whenPressed(Rolletta.getInstance().m_calibrateRed());
        // new JoystickButton(m_operatorJoystick, ButtonPorts.operatorJSX)
        //         .whenPressed(Rolletta.getInstance().m_calibrateBlue());
        // new JoystickButton(m_operatorJoystick, ButtonPorts.operatorJSA)
        CommandScheduler.getInstance().run();
        //         .whenPressed(Rolletta.getInstance().m_calibrateGreen());
        Shooter.getInstance();
    }

    @Override
    public void testPeriodic() {
        CommandScheduler.getInstance().run();
    }

    

    @Override
    public void robotPeriodic() {
        // new PrintCommand("CHECK").schedule();
        // Preferences.getInstance().putDouble("AAAaAAAAa",
        // Utilites.getPowerPortToRobotAngle());
        Preferences.getInstance().putDouble("PP/distance2", Vision.getDirectDistanceFromPowerPort());
        Preferences.getInstance().putDouble("PP/ty", Vision.getPitchAngle());
        CommandScheduler.getInstance().run();

        Preferences.getInstance().putBoolean("PP/inRange",
                Preferences.getInstance().getDouble("PP/distance2", 0) > Preferences.getInstance()
                        .getDouble("PP/minDistance", -1000)
                        && Preferences.getInstance().getDouble("PP/distance2", 0) > Preferences.getInstance()
                                .getDouble("PP/maxDistance", 1000))

        ;
        Preferences.getInstance().putDouble("Shooter/throwing ratio", Shooter.getInstance().getThrowerSpeed()/Shooter.getInstance().throwerDistancePerPulse());
        // System.out.println("TEST");

        // System.out.println("REFLECTIVE CENTER " +
        // SmartDashboard.getNumber("Distance", -2));

        
        
        if (Chassis.getInstance().getOdometry() != null) {
            SmartDashboard.putNumber("Robot X", Chassis.getInstance().getOdometry().getPoseMeters().getTranslation().getX());
            SmartDashboard.putNumber("Robot Y", Chassis.getInstance().getOdometry().getPoseMeters().getTranslation().getY());
            SmartDashboard.putNumber("Robot Alpha", Chassis.getInstance().getOdometry().getPoseMeters().getRotation().getDegrees());
            Chassis.getInstance().getOdometry().update(
                Rotation2d.fromDegrees(Chassis.getInstance().getAngle()),
                Chassis.getInstance().getLeftDistance(),
                Chassis.getInstance().getRightDistance()
            );
        }

        else Chassis.getInstance().initOdometry(0, 0);

        SmartDashboard.putNumber("CHASSIS LEFT RAW", Chassis.getInstance().getLeftEncoder().getRaw());
        SmartDashboard.putNumber("CHASSIS LEFT TICKS", Chassis.getInstance().getLeftEncoder().get());
        SmartDashboard.putNumber("CHASSIS LEFT DISTANCE", Chassis.getInstance().getLeftEncoder().getDistance
        ());

        SmartDashboard.putNumber(
            "CHASSIS LEFT TALON", Chassis.getInstance().getLeftTalon().getSelectedSensorPosition());
        SmartDashboard.putNumber("CHASSIS RIGHT", Chassis.getInstance().getRightDistance());
        
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
        CameraServer.getInstance().startAutomaticCapture();
        CommandScheduler.getInstance().registerSubsystem(Shooter.getInstance(), Chassis.getInstance(),
                Climb.getInstance(), Collector.getInstance(), Rolletta.getInstance(), Storage.getInstance());
        Preferences.getInstance().putDouble("PP/Kp", 0);
        Preferences.getInstance().putDouble("PP/Ki", 0);
        Preferences.getInstance().putDouble("PP/Kd", 0);
        Preferences.getInstance().putDouble("PP/minDistance", 2.5);
        Preferences.getInstance().putDouble("PP/maxDistance", 3);
        SmartDashboard.putNumber("lowh", Utilites.lowH);
        SmartDashboard.putNumber("lows", Utilites.lowS);
        SmartDashboard.putNumber("lowv", Utilites.lowV);
        SmartDashboard.putNumber("highh", Utilites.highH);
        SmartDashboard.putNumber("highs", Utilites.highS);
        SmartDashboard.putNumber("highv", Utilites.highV);

        Autonomous.getInstance();

        // Shooter.getInstance().m_thrower.m_motor.set(0.4);

    }
     /**
     * just pass the Initiaon line
     */
    public CommandBase passInitiaonLine(){
        return new CommandBase() {
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
    };}
    /**
     * shoot from every place
     */
    public CommandBase shoot(long shootTime){
        return new CommandBase() {
        public void initialize(){
            new RotateTilSeePort(true, 0.3).schedule();
            CommandBase turn =  Chassis.getInstance().turnToPowerPortCMD();
            turn.schedule();
            try {
                Thread.sleep(1000);
            } catch (InterruptedException e) {
                e.printStackTrace();
                throw new RuntimeException();
            }
            turn.cancel();
            CommandBase throwPC = Shooter.getInstance().getShootToUpper();
            throwPC.schedule();
            try {
                Thread.sleep(shootTime);
            } catch (InterruptedException e) {
                e.printStackTrace();
                throw new RuntimeException();
            }
            throwPC.cancel();



        }
    };
}   
    /**
     * collect from infront the trunch and bakc from it
     * @param powerCell 1 or 2  or 3
     * @param allienceTrunch collect from allience trunch if not true: power cell param not meter and the robot will collect 2 powercel
     * assuiming that the robot is infront of the lower power cell
     */
     public void collectFromFrontTrunch(int powerCell,boolean allienceTrunch){
        MoveChassisTo m_move;
        CommandBase m_collect;
        //collect from our trunch
        if(allienceTrunch){
            switch (powerCell) {
               //one power cell
                case 1:
                   
                    m_move = new MoveChassisTo(3.6);
                    m_collect = Collector.getInstance().collectCmd();
                    //move to the power cellls and collect them
                    m_move.schedule();
                    m_collect.schedule();
                    CommandScheduler.getInstance().run();
                    while(!m_move.isFinished()){
                 
                    }
                    m_collect.cancel();
                    //return from the trunch
                    new MoveChassisTo(-3.6).schedule();
                    CommandScheduler.getInstance().run();
                break;
            //two power cells
            case 2:
                m_move = new MoveChassisTo(4.4);
                m_collect = Collector.getInstance().collectCmd();
                //move to the power cellls and collect them
                m_move.schedule();
                m_collect.schedule();
                CommandScheduler.getInstance().run();
                while(!m_move.isFinished()){
            
                }
                m_collect.cancel();
                //return from the trunch
                new MoveChassisTo(-4.4).schedule();
                    CommandScheduler.getInstance().run();
           break;
           //three power cells
           case 3:
                m_move = new MoveChassisTo(5.5);
                m_collect = Collector.getInstance().collectCmd();
                //move to the power cellls and collect them
                m_move.schedule();
                m_collect.schedule();
                CommandScheduler.getInstance().run();
                while(!m_move.isFinished()){
        
                }
                m_collect.cancel();
                //return from the trunch
                new MoveChassisTo(-5.5).schedule();
                CommandScheduler.getInstance().run();
       break;

        }
       //collect from opponent trunch 
        }else{
            //collect one power cell
            m_move = new MoveChassisTo(3.3);
            m_collect = Collector.getInstance().collectCmd();
            m_move.schedule();
            m_collect.schedule();
            CommandScheduler.getInstance().run();
            while(!m_move.isFinished()){
                 
            }
            m_collect.cancel();
            //collect another power cell
            CommandBase m_rotate = new RotateTo(-90);
            m_rotate.schedule();
            CommandScheduler.getInstance().run();
            while(!m_rotate.isFinished()){}
            CommandBase m_move2 = new MoveChassisTo(0.8);
            m_move2.schedule();
            m_collect.schedule();
            CommandScheduler.getInstance().run();
            while(!m_move2.isFinished()){

            }
            m_collect.cancel();
            //return 0.8 meter upper from starting point
            m_rotate.schedule();
            CommandScheduler.getInstance().run();
            while(!m_rotate.isFinished())
            new MoveChassisTo(-3.3).schedule();
            CommandScheduler.getInstance().run();



        }
    }
    
    
    public CommandBase returnFromOurTrunch(){
        return new SequentialCommandGroup(
        new RotateTo(90),
        new MoveChassisTo(2),
        new RotateTo(90));
    }
    public CommandBase returnFromOpponentTrunch(){
        return new SequentialCommandGroup(
            new RotateTo(-90),
            new MoveChassisTo(4.4),
            new RotateTo(-90)
        );
    }
    
    /**
     * 
     * start inftront the power cell collect two ball and return
     */
    public CommandBase collectFromOurTrunch(int powerCells){
        return new SequentialCommandGroup(
        
        (new RotateTo(90)),
        (new MoveChassisTo(2)),
        new RotateTo(90),
        new InstantCommand(() -> collectFromFrontTrunch(powerCells, true)),
        new RotateTo(90),
        new MoveChassisTo(2),
        new RotateTo(90)
        );
    }
    public CommandBase collectFromOpponentTrunch(){
        return new SequentialCommandGroup(
            (new RotateTo(-90)),
            (new MoveChassisTo(5.2)),
            new RotateTo(-90),
            new InstantCommand(() -> collectFromFrontTrunch(0, false)),
            new RotateTo(-90),
            new MoveChassisTo(4.4),
            new RotateTo(-90)
            );
        
    }


    //TODO: check and tune drive to and rotate to,check all method above
    @Override
    public void autonomousInit() {
        //TODO: check and tune drive to and rotate to,check all method above
        new MoveChassisTo(-2).schedule();
        CommandScheduler.getInstance().run();
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        // Autonomous.getInstance().schedule();
        
        
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


        
        // try{
        //     Shooter.getInstance().getAimDown().schedule();;
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
        // Autonomous.getInstance().update();

        // if (m_operatorJoystick.getRawButton(1))
        //     Chassis.getInstance().
    }

    @Override
    public void teleopPeriodic() {
        CommandScheduler.getInstance().run();
        Preferences.getInstance().putDouble("talon1_encoder", Chassis.getInstance().getRightTalonSRX().getSelectedSensorPosition());
        Preferences.getInstance().putDouble("talon15_encoder", Chassis.getInstance().getLefTalonSRX().getSelectedSensorPosition());
    }

    @Override
    public void teleopInit() {
        Rolletta.getInstance().addAllColors();
        Chassis.getInstance().setDefaultCommand(Chassis.getInstance().simpleDriveCMD()); // checed
        Preferences.getInstance().putDouble("PP/somthing", 0);
        new JoystickButton(m_operatorJoystick, ButtonPorts.operatorJSY).whenPressed(Shooter.getInstance().getAimUp());
        new JoystickButton(m_operatorJoystick, ButtonPorts.operatorJSA).whenPressed(Shooter.getInstance().getAimDown());
        new JoystickButton(m_operatorJoystick, ButtonPorts.operatorJSLT).whileHeld(Shooter.getInstance().getAccelerateToThrow());
        new JoystickButton(m_operatorJoystick, ButtonPorts.operatorJSRT).whileHeld(Storage.getInstance().getPass());
        new JoystickButton(m_operatorJoystick, ButtonPorts.operatorJSLB).whileHeld(Climb.getInstance().m_up());
        new JoystickButton(m_operatorJoystick, ButtonPorts.operatorJSRB).whileHeld(Climb.getInstance().getClimbDown());
        new JoystickButton(m_operatorJoystick, ButtonPorts.operatorJSX).whileHeld(Collector.getInstance().collectCmd());
        new JoystickButton(m_operatorJoystick, ButtonPorts.operatorJSB).whileHeld(Climb.getInstance().m_pull());
        new JoystickButton(m_righJoystick, 5).whenPressed(Chassis.getInstance().turnToPowerPortCMD());
		new JoystickButton(m_righJoystick, 8).whileHeld(new InstantCommand(() ->
        CommandScheduler.getInstance().cancelAll()));
        new JoystickButton(m_leftJoystick, 8).whileHeld(new InstantCommand(() ->
        CommandScheduler.getInstance().cancelAll()));
        new JoystickButton(m_righJoystick, 1).whileHeld(new CommandBase() {//turbo mode
            
            @Override
            public void initialize() {
                Chassis.getInstance().setSpeedModifier(1);
            }

            @Override
            public void end(boolean interrupted) {
                Chassis.getInstance().resetSpeed();
            }
        });

		new JoystickButton(m_leftJoystick, 1).whileHeld(new CommandBase() {//slow mode
            
            @Override
            public void initialize() {
                Chassis.getInstance().setSpeedModifier(0.2);
            }

            @Override
            public void end(boolean interrupted) {
                Chassis.getInstance().resetSpeed();
            }
        });

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
        
        // TODO: add climb down on RB

        Chassis.getInstance().initOdometry(0, 0);

    };
}
