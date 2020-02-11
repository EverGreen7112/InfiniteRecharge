package com.evergreen.robot.wpilib.subsystem;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.evergreen.robot.RobotMap;
import com.evergreen.robot.wpilib.Utilites;

import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandGroupBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Supplier;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Preferences;
import com.evergreen.robot.wpilib.commands.*;

/**
 * Shooter subsystem, basically divided to two parts:
 * <ul>
 * <li> <b>the aimer</b> - the part that aim the thrower</li>
 * <li> <b>the thrower</b> - the two wheels which shoot the power cell</li>
 * </ul>
 */
public class Shooter extends SubsystemBase implements RobotMap {
    
    private static Shooter m_shooter = new Shooter();
    ////////////////////////compoments/////////////////////////////////////////
    // TODO:put correct SpeedController Type
    /**
     * the part that aim the thrower
     */
    public SpeedController m_aimer = new WPI_VictorSPX(MotorPorts.aimer);
    /**
     * the two wheels which shoot the power cell
     */
    public SpeedController m_thrower = new WPI_TalonSRX(MotorPorts.thrower);
    public Encoder m_aimerEncoder = new Encoder(DigitalPorts.aimerEncoderA, DigitalPorts.aimerEncoderB);
    public Encoder m_throwerEncoder = new Encoder(DigitalPorts.throwerEncoderA, DigitalPorts.throwerEncoderB);
    
    //////////////////////////////////////////////////////////////////////////
    
    /**
     * shooter highet in cm
     */
    public final double SHOOTER_HIGHET = 29;
    /**
     * if the aimer is at the upper position return true if it at the lower position
     * return false if between return last position
     */
    public boolean m_atUpperPosition = true;
    /**
     * if we are want to shoot to the inner port is true, else change it to false; 
     */
    public boolean shootToInner() {
        return true;
    } 
    ////////////////////////tuneable values//////////////////////////////////////
    // TODO: tune the values
    public double aimingUpAngle() {
        return Preferences.getInstance().getDouble("aimingUpAngle", 3);
    }

    public double aimingDownAngle() {
        return Preferences.getInstance().getDouble("aimingDownAngle", 3);
    }

    public double aimerKp() {
        return Preferences.getInstance().getDouble("aimerKp", 0);
    }

    public double aimerKi() {
        return Preferences.getInstance().getDouble("aimerKi", 0);
    }

    public double aimerKd() {
        return Preferences.getInstance().getDouble("aimerKd", 0);
    }

    public double throwerKp() {
        return Preferences.getInstance().getDouble("throwerKp", 0);
    }

    public double throwerKi() {
        return Preferences.getInstance().getDouble("throwerKi", 0);
    }

    public double throwerKd() {
        return Preferences.getInstance().getDouble("throwerKd", 0);
    }
    
    public double aimerAnglePerPulse() {
        return Preferences.getInstance().getDouble("aimer/anglePerPulse", 1);
    }
    
    public double throwerDistancePerPulse() {
        return Preferences.getInstance().getDouble("thrower/distancePerPulse", 0.1);
    }
    
    /**
     * 
     * @return the speed that used to drop the ball
     */
    public double droppingSpeed() {
        return Preferences.getInstance().getDouble("thrower/droppingSpeed", 0.06);
    }
    
    ////////////////////////////////////////////////////////////////////////////
    ////////////////////////////encoders output/////////////////////////////////
    public double getAimerAngle() {
        return m_aimerEncoder.getDistance();
    }

    public double getThrowerSpeed() {
        return m_throwerEncoder.getRate();
    }
    ////////////////////////////////////////////////////////////////////////////
    
    ////////////////////////////pid controllers/////////////////////////////////
    private PIDController m_aimController = new PIDController(aimerKp(), aimerKi(),
            aimerKd());

    private PIDController m_throwController = new PIDController(throwerKp(), throwerKi(),
                    throwerKd());
    ////////////////////////////////////////////////////////////////////////////

    ////////////////////////////pid controllers getters/////////////////////////////////
    public PIDController getAimController() {
        return m_aimController;
    }

    public PIDController getThrowController() {
        return m_throwController;
    }
    ////////////////////////////////////////////////////////////////////////////

    ///////////////////////////commands////////////////////////////////////////////////
    public CommandBase m_aimUp = new PIDCommand(m_aimController, Shooter.getInstance()::getAimerAngle,
            aimingUpAngle(), Shooter.getInstance().m_aimer::set, Shooter.getInstance()) {
             @Override
            public void end(boolean interrupted){
                super.end(interrupted);;
                m_atUpperPosition = true;
            }
            };
    public CommandBase m_aimDown = new PIDCommand(m_aimController, Shooter.getInstance()::getAimerAngle,
            aimingDownAngle(), Shooter.getInstance().m_aimer::set, Shooter.getInstance()) {
            @Override
            public void end(boolean interrupted){
               super.end(interrupted);;
                m_atUpperPosition =false;
            }
            };
    public CommandBase m_aimToggle = new PIDCommand(m_aimController, Shooter.getInstance()::getAimerAngle,
           ()->{ 
               if(m_atUpperPosition){
                    return aimingDownAngle();
                }
                return aimingUpAngle();
        },Shooter.getInstance().m_aimer::set, Shooter.getInstance()) {
            public void end(boolean interrupted){
                super.end(interrupted);
                m_atUpperPosition =!m_atUpperPosition;
            }
         };
        
        
     /**
     * throw the ball in the correct speed by pid, if aimer at upper shoot to inner/outer according to {@link #shootToInner()} 
     * if aimer at the lower position shoot to the bootom.
     */
    public CommandBase m_throw = new PIDCommand(m_aimController, Shooter.getInstance()::getThrowerSpeed, () ->{
   if(m_atUpperPosition){
        if(shootToInner()){
            return PowerPorts.INNER.motorSpeed;
        }
        return PowerPorts.OUTER.motorSpeed;
    
    }
    return PowerPorts.BOTTOM.motorSpeed;
    }
    ,
    Shooter.getInstance().m_thrower::set, Shooter.getInstance());
    /**
    * aim and shoot to outer or inner port according to {@link #shootToInner()} 
    */
    public CommandBase shootToUpper =  m_aimUp.andThen(m_throw);
     /**
     * aim and shoot to bottom port
     */
    public CommandBase shootToBottom = m_aimDown.andThen(m_throw);
     /**
      *  used to drop the ball and leave it for our aliince members.
      */  
    public CommandBase drop = new InstantCommand(() -> m_thrower.set(droppingSpeed()),Shooter.getInstance()) {
        public void end(boolean interrupted){
            m_thrower.set(0);
        }
    };
    /**
     * used to pass the ball to our allience members
     */
    public PassPowerCell m_pass = new PassPowerCell(700);
    /////////////////////////////////////////////////////////////////////////
    /**
     * update the distance that we want to throw to
     */
    public void updatePassDistance(){
        //TODO decide how to implemnt by m_pass.setDistance()
    }
    /**
     * Construct new Shooter and at the values to the shufflBoard
     */
    private Shooter() {
        Preferences.getInstance().putDouble("aimingUpAngle", 30);
        Preferences.getInstance().putDouble("aimingDownAngle", 15);
        Preferences.getInstance().putDouble("aimerKp", 0);
        Preferences.getInstance().putDouble("aimerKi", 0);
        Preferences.getInstance().putDouble("aimerKd", 0);
        Preferences.getInstance().putDouble("throwerKp", 0);
        Preferences.getInstance().putDouble("throwerKi", 0);
        Preferences.getInstance().putDouble("throwerKd", 0);
        Preferences.getInstance().putDouble("thrower/speedGoal", 0.1);
        Preferences.getInstance().putDouble("aimer/anglePerPulse", 1);
        Preferences.getInstance().putDouble("thrower/distancePerPulse", 1);
        Preferences.getInstance().putDouble("thrower/droppingSpeed", 0.06);
        m_aimerEncoder.setDistancePerPulse(aimerAnglePerPulse());
        m_throwerEncoder.setDistancePerPulse(throwerDistancePerPulse());

    }
    
    public static Shooter getInstance() {
        return m_shooter;
    }
    
   
    /**
     * Present the power ports and their constant in cm for calculate the motorspeed
     */
    public enum PowerPorts {
        //sizes in cm
        INNER(0, 249,Shooter.getInstance().aimingUpAngle(), "circle"), 
        OUTER(74.295, 249,Shooter.getInstance().aimingUpAngle(), "hexagon"),
        BOTTOM(74.295, 46,Shooter.getInstance().aimingDownAngle(), "rectangle");

        public final double X_GOAL;
        public final double HIGHT_GOAL;
        public double m_angle;
        public String m_shape;
        public double startXVelocity;
        public double startHightVelocity;
        public double shootingTime;
        public double motorSpeed;
        /**
         * calculate the varibles and the motorSpeed
         * 
         */
        private PowerPorts(double xGoal, double hightGoal,double angle, String shape) {
            HIGHT_GOAL = hightGoal;
            X_GOAL = xGoal;
            m_shape = shape;
            m_angle = angle;
            //FIXME: check if this formula is like the orignal, if change here change
            // in PassPowerCell
            shootingTime =
           Math.sqrt((-2*Utilites.getDistanceFromPowerPort() * 
           Math.tan(m_angle) + Shooter.getInstance().SHOOTER_HIGHET -HIGHT_GOAL) /Utilites.GRAVITY_CONSTANT);
            startXVelocity = 
            (-Utilites.getDistanceFromPowerPort())/shootingTime;
            
            startHightVelocity = 
            (2*(HIGHT_GOAL - Shooter.getInstance().SHOOTER_HIGHET) - 
            Utilites.GRAVITY_CONSTANT*Math.pow(shootingTime, 2))
                                /2*shootingTime;
            motorSpeed = Math.sqrt(Math.pow(startXVelocity, 2) + Math.pow(startHightVelocity, 2));


        }

    }
}
