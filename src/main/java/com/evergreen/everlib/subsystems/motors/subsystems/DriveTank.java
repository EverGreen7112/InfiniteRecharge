package com.evergreen.everlib.subsystems.motors.subsystems;

import java.util.Map;

import com.evergreen.everlib.oi.joysticks.JoystickEG;
import com.evergreen.everlib.subsystems.motors.commands.TankDrive;
import com.evergreen.everlib.subsystems.sensors.DistanceSensor;

import edu.wpi.first.wpilibj.Joystick.AxisType;


/**
 * DriveTank
 */
public class DriveTank extends MotorSubsystem {

    public DriveTank(String name, MotorController leftMotors, MotorController rightMotors) {
        super(name, leftMotors, rightMotors);
    }

    public DriveTank(String name, MotorController leftMotors, MotorController rightMotors, DistanceSensor sensor) {
        super(name, leftMotors, rightMotors);
    }

    public DriveTank(String name, DistanceSensor sensor, MotorController leftMotors, MotorController rightMotors) {
        super(name, leftMotors, rightMotors);
    }

    public void drive(double leftSpeed, double rightSpeed) {
        set(Map.of(0, leftSpeed, 1, rightSpeed));
    }

    public void setDefaultByJoystick(JoystickEG joystick, AxisType leftAxis, AxisType rightAxis) {
        setDefaultCommand(
            new TankDrive(
                getName() + " default command (drive tank by joystick)", 
                this,
                () -> joystick.getRawAxis(leftAxis),
                () -> joystick.getRawAxis(rightAxis)));
    }
}