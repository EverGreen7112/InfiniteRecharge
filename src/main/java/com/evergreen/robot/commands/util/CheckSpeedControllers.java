package com.evergreen.robot.commands.util;

import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * CheckSpeedControllers
 */
public class CheckSpeedControllers extends CommandBase {
    public static enum SpeedConrollerType {
        WPI_VICTORSPX, WPI_TALONSRX;
    }

    private SpeedController m_motor;
    private Supplier<Double> m_speed;
    private Supplier<Integer> m_port;
    private Supplier<SpeedConrollerType> m_controllerType;

    /**
     * construct new Check speed controller from shuffleboard
     */
    public CheckSpeedControllers() {
        SendableChooser<SpeedConrollerType> chooser = new SendableChooser<SpeedConrollerType>();
        chooser.setDefaultOption("victor", SpeedConrollerType.WPI_VICTORSPX);
        chooser.addOption("talon", SpeedConrollerType.WPI_TALONSRX);
        SmartDashboard.putData(chooser);
        Preferences.getInstance().putDouble("SpeedControllerCheck/speed", 0);
        Preferences.getInstance().putInt("SpeedControllerCheck/port", 1);
        Preferences.getInstance().putDouble("SpeedControllerCheck/speed", 0);
        m_controllerType = () -> chooser.getSelected();
        m_speed = () -> Preferences.getInstance().getDouble("SpeedControllerCheck/speed", 0);
        m_port = () -> Preferences.getInstance().getInt("SpeedControllerCheck/port", 1);
    }

    /**
     * construct new Check speed controller
     */
    public CheckSpeedControllers(double speed, int port, SpeedConrollerType controlllerType) {
        m_speed = () -> speed;
        m_controllerType = () -> controlllerType;
        m_port = () -> port;
    }

    public void setMotor(int port, SpeedConrollerType conrollerType) {
        m_port = () -> port;
        m_controllerType = () -> conrollerType;
    }

    public void setMotor(int port) {
        setMotor(port, m_controllerType.get());
    }

    public void setMotor(SpeedConrollerType conrollerType) {
        setMotor(m_port.get(), conrollerType);
    }

    public void setSpeed(double speed) {
        m_speed = () -> speed;
    }

    /**
     * update the speed controller
     */
    private void updateControllers() {
        switch (m_controllerType.get()) {
        case WPI_VICTORSPX:
            m_motor = new WPI_VictorSPX(m_port.get());
            break;
        case WPI_TALONSRX:
            m_motor = new WPI_TalonSRX(m_port.get());
            break;
        }
    }

    @Override
    public void execute() {
        updateControllers();
        m_motor.set(m_speed.get());
    }
    @Override
    public void end(boolean interrupted) {
        m_motor.set(0);
    }
}
