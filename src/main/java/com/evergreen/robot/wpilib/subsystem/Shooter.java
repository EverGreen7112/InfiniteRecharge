package com.evergreen.robot.wpilib.subsystem;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.evergreen.robot.RobotMap;
import com.evergreen.robot.wpilib.Storage;
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
 * Shooter subsystem, basically divided to two Subsystem:
 * <ul>
 * <li> <b>the aimer</b> - the part that aim the thrower</li>
 * <li> <b>the thrower</b> - the two wheels which shoot the power cell</li>
 * </ul>
 */

public class Shooter extends SubsystemBase implements RobotMap {
    private static Shooter m_shooter = new Shooter();
    ////////////////////////sub Subsystem/////////////////////////////////////////
    // TODO:put correct SpeedController Type
     /**
     * the part that aim the thrower
     */
    public Aimer m_aimer = new Aimer();
     /**
     * the two wheels which shoot the power cell
     */
    public Thrower m_thrower = new Thrower();
    
    /**
     * the part that aim the thrower
     */
    public class Aimer extends SubsystemBase{
        // TODO:put correct SpeedController Type
        public SpeedController m_motor = new WPI_TalonSRX(MotorPorts.aimer);
        public Encoder m_encoder = new Encoder(AnalogPorts.aimerA, AnalogPorts.aimerB);
    }

    /**
     * the two wheels which shoot the power cell
     */
    public class Thrower extends SubsystemBase{
        // TODO:put correct SpeedController Type
        public SpeedController m_motor = new WPI_TalonSRX(MotorPorts.thrower);
        public Encoder m_encoder = new Encoder(AnalogPorts.throwerA, AnalogPorts.throwerB);
    }
    //////////////////////////////////////////////////////////////////////////
    /**
     * shooter highet in cm
     */
    public final double SHOOTER_HEIGHT = 29;
    /**
     * if the aimer is at the upper position return true if it at the lower position
     * return false if between return last position
     */
    //TODO: find start position
    public boolean m_atUpperPosition = true;
    
    ////////////////////////tuneable values//////////////////////////////////////
    // TODO: tune the values
    public double aimingUpAngle() {
        return Preferences.getInstance().getDouble("aimingUpAngle", 30);
    }

    public double aimingDownAngle() {
        return Preferences.getInstance().getDouble("aimingDownAngle", 10);
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
        return m_aimer.m_encoder.getDistance();
    }

    public double getThrowerSpeed() {
        return m_thrower.m_encoder.getRate();
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
    
    private CommandBase m_aimUp = new PIDCommand(m_aimController, Shooter.getInstance()::getAimerAngle,
            aimingUpAngle(), Shooter.getInstance().m_aimer.m_motor::set, m_aimer) {
             @Override
            public void end(boolean interrupted){
                super.end(interrupted);;
                m_atUpperPosition = true;
            }
            };
    private CommandBase m_aimDown = new PIDCommand(m_aimController, Shooter.getInstance()::getAimerAngle,
            aimingDownAngle(), Shooter.getInstance().m_aimer.m_motor::set, m_aimer) {
            @Override
            public void end(boolean interrupted){
               super.end(interrupted);;
                m_atUpperPosition =false;
            }
            };
    private CommandBase m_aimToggle = new PIDCommand(m_aimController, Shooter.getInstance()::getAimerAngle,
           ()->{ 
               if(m_atUpperPosition){
                    return aimingDownAngle();
                }
                return aimingUpAngle();
        },Shooter.getInstance().m_aimer.m_motor::set, m_aimer) {
            public void end(boolean interrupted){
                super.end(interrupted);
                m_atUpperPosition =!m_atUpperPosition;
            }
         };
    /**
     *  stops the thrower
     */     
    private CommandBase m_stopThrower = new InstantCommand(()-> m_thrower.m_motor.set(0),m_thrower);
    /**
     * accelerate for shooting to inner/outer according to {@link Utilites#isShootingToInnerWork()} .
     * 
     */    
    private CommandBase m_accelerateUp = new PIDCommand(m_aimController, Shooter.getInstance()::getThrowerSpeed, () ->{
             if(Utilites.isShootingToInnerWork()){
                 return PowerPorts.INNER.motorSpeed;
             }
             return PowerPorts.OUTER.motorSpeed;
            },
         Shooter.getInstance().m_thrower.m_motor::set, m_thrower);
    /**
     * acccelerate for shooting to the bootom.
     */
    private CommandBase m_accelerateBottom = new PIDCommand(m_aimController, Shooter.getInstance()::getThrowerSpeed, 
         PowerPorts.BOTTOM.motorSpeed,
         Shooter.getInstance().m_thrower.m_motor::set, m_thrower);
     /**
     * if aimer at upper accelerate for shooting to inner/outer according to {@link Utilites#isShootingToInnerWork()} 
     * if aimer at the lower position acccelerate for shooting to the bootom.
     */
    private CommandBase m_accelerteTothrow = new PIDCommand(m_aimController, Shooter.getInstance()::getThrowerSpeed, () ->{
        if (m_atUpperPosition) {

                if (Utilites.isShootingToInnerWork()) {
                    return PowerPorts.INNER.motorSpeed;
                }

                return PowerPorts.OUTER.motorSpeed;
        }

        return PowerPorts.BOTTOM.motorSpeed;
    }
    ,
    m_thrower.m_motor::set, m_thrower) ;

    /**
    * fully(aim while accelerate and then passToShooter) shoot to outer or inner port according
    * to {@link Utilites#isShootingToInnerWork}
    */
    private CommandBase shootToUpper =  Utilites.toFullShootingCommand(new ParallelCommandGroup(m_aimUp,m_accelerateUp));
     /**
     * fully(aim while accelerate and then passToShooter) shoot to bottom port
     */
    private CommandBase shootToBottom = Utilites.toFullShootingCommand( new ParallelCommandGroup(m_aimDown,m_accelerateBottom));
     /**
      * fully(aim while accelerate and then passToShooter) drop the ball and leave it for our aliince members.
      */  
    //TODO: find how many time take to the robot to drop
      private CommandBase drop =Utilites.toFullShootingCommand(new InstantCommand(() -> m_thrower.m_motor.set(droppingSpeed()),m_thrower) {
        public void end(boolean interrupted){
            m_thrower.m_motor.set(0);
        }
    });
    /**
     *fully (aim while accelerate and then passToShooter) pass the power cell to the desierd distance
     */
    public PassPowerCell m_pass =new PassPowerCell(700);
    /////////////////////////////////////////////////////////////////////////
    
    /////////////////////////commands getters///////////////////////////////
    /**
     * 
     * @return {@link Shooter#m_aimUp} 
     */
    public CommandBase getAimUp(){
        return m_aimUp;
    }
    /**
     * 
     * @return {@link Shooter#m_aimDown} 
     */
    public CommandBase getAimDown(){
        return m_aimDown;
    }
    /**
     * 
     * @return {@link Shooter#m_aimToggle} 
     */
    public CommandBase getAimToggle(){
        return m_aimToggle;
    }
    /**
     * 
     * @return {@link m_stopThrower} -
     * stops the thrower
     */
    public CommandBase getStopThrower(){
        return m_stopThrower;
    }
    /**
     * 
     * @return {@link Shooter#m_accelerateUp} - 
     * accelerate for shooting to inner/outer according to {@link Utilites#isShootingToInnerWork()}
     */
    public CommandBase getAccelerateUp(){
        return m_accelerateUp;
    }
    /**
     * 
     * @return {@link Shooter#m_accelerateBottom} - 
     * acccelerate for shooting to the bootom  
     */
    public CommandBase getAccelerateBottom(){
        return m_accelerateBottom;
    }
    /**
     * 
     * @return {@link Shooter#m_accelerteTothrow} - 
     * if aimer at upper accelerate for shooting to inner/outer according to {@link Utilites#isShootingToInnerWork()} 
     * if aimer at the lower position acccelerate for shooting to the bootom.
     */
    public CommandBase getAccelerateToThrow(){
        return m_accelerteTothrow;
    }
    /**
     * 
     * @return {@link Shooter#shootToUpper} - 
     * fully(aim while accelerate and then passToShooter) shoot to outer or inner port according to Utilites.isShootingToInnerWork
     */
    public CommandBase getShootToUpper(){
        return shootToUpper;
    }
    /**
     * 
     * @return {@link Shooter#shootToBottom} - fully(aim while accelerate and then passToShooter) shoot to bottom port
     */
    public CommandBase getShootToBottom(){
        return shootToBottom;
    }
    /**
     * 
     * @return {@link Shooter#drop} -
     * fully(aim while accelerate and then passToShooter) drop the ball and leave it for our aliince members.
     */
    public CommandBase getDrop(){
        return drop;
    }
    /**
     * 
     * @return {@link Shooter#m_pass} - 
     * fully (aim while accelerate and then passToShooter) pass the power
     *  cell to the desierd distance
     */
    public CommandBase getPass(){
        return m_pass;
    }
    /////////////////////////////////////////////////////////////////
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
        m_aimer.m_encoder.setDistancePerPulse(aimerAnglePerPulse());
        m_thrower.m_encoder.setDistancePerPulse(throwerDistancePerPulse());

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
           Math.sqrt((-2*Utilites.getXDistanceFromPowerPort() * 
           Math.tan(m_angle) + Shooter.getInstance().SHOOTER_HEIGHT -HIGHT_GOAL) /Utilites.GRAVITY_CONSTANT);
            startXVelocity = 
            (-Utilites.getXDistanceFromPowerPort())/shootingTime;
            
            startHightVelocity = 
            (2*(HIGHT_GOAL - Shooter.getInstance().SHOOTER_HEIGHT) - 
            Utilites.GRAVITY_CONSTANT*Math.pow(shootingTime, 2))
                                /2*shootingTime;
            motorSpeed = Math.sqrt(Math.pow(startXVelocity, 2) + Math.pow(startHightVelocity, 2));


        }

    }
}
