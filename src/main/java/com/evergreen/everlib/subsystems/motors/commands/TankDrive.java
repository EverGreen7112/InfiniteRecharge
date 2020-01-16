package com.evergreen.everlib.subsystems.motors.commands;

import java.util.List;
import java.util.Map;
import java.util.function.Supplier;

import com.evergreen.everlib.shuffleboard.loggables.LoggableDouble;
import com.evergreen.everlib.subsystems.motors.subsystems.DriveTank;
import com.evergreen.everlib.utils.ranges.Range;

/**
 * TankDrive
 */

public class TankDrive extends SetMotorSystem {

  /**
   * Constructs a {@link SetMotorSystem MoveMotorSystem Command} according to input parameteres   
   * @param name - The command's name - will correspond to its getName() method, and be the key
   * to its shuffleboard switch.
   * @param subsystem - The subsystem to move.
   * @param speedMap - A {@link Map} which links controllers (by index) to speed suppliers.
   */
  public TankDrive(String name, DriveTank subsystem, Supplier<Double> leftSpeed, 
  Supplier<Double> rightSpeed) {
  super(name, subsystem, Map.of(0, leftSpeed, 1, rightSpeed));
  }

  /**
   * Constructs a {@link TanKDrive TanKDrive Command} according to input parameteres   
   * @param name - The command's name - will correspond to its getName() method, and be the key
   * to its shuffleboard switch.
   * @param subsystem - The subsystem to move.
   * @param leftSpeed - the speed supplier for the power to give to the left motors of the drive tank.
   * @param rightSpeed - the speed supplier for the power to give to the right motors of the drive tank.
   * @param speedModifier - the modifier for the speed suppliers for this movement.
   */
  public TankDrive(String name, DriveTank subsystem, Supplier<Double> leftSpeed, 
    Supplier<Double> rightSpeed, Supplier<Double> speedModifier) {
    super(name, subsystem, speedModifier, Map.of(0, leftSpeed, 1, rightSpeed));
  }

  /**
   * Constructs a {@link TanKDrive TanKDrive Command} according to input parameteres   
   * @param name - The command's name - will correspond to its getName() method, and be the key
   * to its shuffleboard switch.
   * @param subsystem - The subsystem to move.
   * @param speedMap - A {@link Map} which links controllers (by index) to speed suppliers.
   * @param limit - a {@link Range} which supplies the range in which to allow the subsystem to move.
   */
  public TankDrive(String name, DriveTank subsystem, Range limit, 
    Supplier<Double> leftSpeed, Supplier<Double> rightSpeed) {
    super(name, subsystem, limit, Map.of(0, leftSpeed, 1, rightSpeed));
  }

  /**
   * 
   * Constructs a {@link TanKDrive TanKDrive Command} according to input parameteres   
   * @param name - The command's name - will correspond to its getName() method, and be the key
   * to its shuffleboard switch.
   * @param subsystem - The subsystem to move.
   * @param limit - The movement's desired limit.
   * @param leftSpeed - the speed supplier for the power to give to the left motors of the drive tank.
   * @param rightSpeed - the speed supplier for the power to give to the right motors of the drive tank.
   * @param speedModifier - the modifier for the speed suppliers for this movement.
   */
  public TankDrive(String name, DriveTank subsystem, Range limit, 
    Supplier<Double> leftSpeed, Supplier<Double> rightSpeed, Supplier<Double> speedModifier) { 
    super(name, subsystem, limit, speedModifier, Map.of(0, leftSpeed, 1, rightSpeed));
  }

  @Override
  protected List<LoggableDouble> getSpeedLoggables() {
    return List.of(
      new LoggableDouble(getName() + " - Left Speed", m_speedMap.get(0)),
      new LoggableDouble(getName() + " - Right Speed", m_speedMap.get(1)),
      new LoggableDouble(
        getName() + " - Mean Power", 
        () -> ( m_speedMap.get(0).get() + m_speedMap.get(1).get() ) / 2.0 ));
  }
}