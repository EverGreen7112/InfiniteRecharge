package com.evergreen.everlib.utils;

import com.evergreen.everlib.shuffleboard.constants.ConstantDouble;
import com.evergreen.everlib.shuffleboard.constants.DashboardConstants;
import com.evergreen.everlib.subsystems.SubsystemEG;
import com.evergreen.everlib.subsystems.motors.subsystems.MotorSubsystem;

import edu.wpi.first.wpilibj.controller.PIDController;

/**
 * A utillity class to easily keep constant parameters, and to avoid cluttering constructors.
 */
public class PIDSettings {

    /**Proportional constant supplier*/
    private final ConstantDouble m_P;
    /**Integral Constant supplier*/
    private final ConstantDouble m_I;
    /**Derivative constant supplier */
    private final ConstantDouble m_D;
    /**Feedforward constant supplier*/
    private final ConstantDouble m_F;
    /**Tolernace supplier */
    private final ConstantDouble m_tolerance;

    private final ConstantDouble m_period;
    /**The controller to write the information into.*/
    private final MotorSubsystem m_subsystem;


    private final PIDController m_controller;

    private static final double 
        DEFAULT_TOLEANCE = 0.0,
        DEFAULT_F  = 0.0,
        DEFAULT_PERIOD = 0.02;

    /**
     * Constructs {@link PIDSettings} according to input P, I and D and F constants and given subsystem.
     * @param subsystem - The subsystem to use the PIDF loop on.
     * @param kP - The proportional constants.
     * @param kI - The integral constant
     * @param kD - The derivative constant.
     * @param tolerance - the loop's tolerance; St what point away from the target should the loop
     * be satisfied?
     * @param kF - The feed-forward constant.
     * @param period - The time between each controller update. 
    */
    public PIDSettings(MotorSubsystem subsystem, double kP, double kI, double kD, 
        double tolerance, double kF, double period) {

        DashboardConstants.getInstance().cd("/" + subsystem.getName() + "/PID");

        m_P = new ConstantDouble("kP", kP);
        m_I = new ConstantDouble("kI", kI);
        m_D = new ConstantDouble("kD", kD);
        m_F = new ConstantDouble("kF", kF);
        
        m_tolerance = new ConstantDouble(
            subsystem.getName() + "Tolerance", tolerance);
        
        m_period = new ConstantDouble(
            subsystem.getName() + "Period", period);   

        m_controller = new PIDController(kP, kI, kD, period);
        m_controller.setTolerance(tolerance);

        m_subsystem = subsystem;
    }

    /**
     * Constructs {@link PIDSettings} according to input P, I and D and F 
     * constants and given subsystem, with a default period of 0.02.
     * @param subsystem - The subsystem to use the PIDF loop on.
     * @param kP - The proportional constants.
     * @param kI - The integral constant
     * @param kD - The derivative constant.
     * @param tolerance - the loop's tolerance; St what point away from the target should the loop
     * be satisfied?
     * @param kF - The feed-forward constant.
    */
    public PIDSettings(MotorSubsystem subsystem, double kP, double kI, double kD, 
        double tolerance, double kF) {
            this(subsystem, kP, kI, kD, tolerance, kF, DEFAULT_PERIOD);
    }


    /**
     * Constructs {@link PIDSettings} according to input P, I and D
     * constants and given subsystem, with a default period of 0.02.
     * @param subsystem - The subsystem to use the PIDF loop on.
     * @param kP - The proportional constants.
     * @param kI - The integral constant
     * @param kD - The derivative constant.
     * @param tolerance - the loop's tolerance; St what point away from the target should the loop
     * be satisfied?
    */
    public PIDSettings(MotorSubsystem subsystem, double kP, double kI, double kD, 
        double tolerance) {
            this(subsystem, kP, kI, kD, tolerance, DEFAULT_F, DEFAULT_PERIOD);
    }

    /**
     * Constructs {@link PIDSettings} according to input P, I and D 
     * constants and given subsystem, with a default period of 0.02 and a default tolerance of 0.
     * 
     * @param subsystem - The subsystem to use the PIDF loop on.
     * @param kP - The proportional constants.
     * @param kI - The integral constant
     * @param kD - The derivative constant.
    */
    public PIDSettings(MotorSubsystem subsystem, double kP, double kI, double kD) {
            this(subsystem, kP, kI, kD, DEFAULT_TOLEANCE, DEFAULT_F, DEFAULT_PERIOD);
    }


    public double kP() {
        return m_P.get();
    }

    public double kI() {
        return m_I.get();
    }

    public double kD() {
        return m_D.get();
    }

    public double kF() {
        return m_F.get();
    }

    public double getTolerance() {
        return m_tolerance.get();
    }

    public double getMeasurment() {
        return m_subsystem.getDistance();
    }

    public double getPeriod() {
        return m_period.get();
    }

    public void write(double power) {
        m_subsystem.move(power);
    }

    public PIDController getController() {
        return m_controller;
    }

    public SubsystemEG getSubsystem() {
        return m_subsystem;
    }
}