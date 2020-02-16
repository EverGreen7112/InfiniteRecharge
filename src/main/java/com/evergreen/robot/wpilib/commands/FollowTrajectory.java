package com.evergreen.robot.wpilib.commands;

import com.evergreen.robot.wpilib.Chassis;
import com.evergreen.robot.wpilib.Collector;
import com.evergreen.robot.wpilib.DoubleArgCommand;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandGroupBase;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/**
 * followTrajectory use to follow trajectory in autounumus
 */
public class FollowTrajectory extends ParallelDeadlineGroup implements DoubleArgCommand{
  private boolean m_collectWhileMoving;
  /**
   * 
   * @param trajectory the trajectory that the command follow.
   * @param collectWhileMoving if we wish to collect power cells while we moving put one else put zero, defualt 0
   */  
  public FollowTrajectory(Chassis.TrajectoryOption trajectory, double collectWhileMoving) {
    super(new RamseteCommand(
      trajectory.getTrajectory(),
      Chassis.getInstance().getOdometry()::getPoseMeters,
      Chassis.getInstance().getrRamseteController(),
      Chassis.getInstance().getFeedforward(),
      Chassis.getInstance().getKinematics(),
     Chassis.getInstance()::getVelocity,
      new PIDController(Chassis.getInstance().getDistanceKd(), Chassis.getInstance().getDistanceKi(),Chassis.getInstance().getDistanceKd()),
      new PIDController(Chassis.getInstance().getDistanceKd(), Chassis.getInstance().getDistanceKi(),Chassis.getInstance().getDistanceKd()),
      Chassis.getInstance()::setVoltage,
      Chassis.getInstance())
     ,new CommandBase[0]); 


      
      m_collectWhileMoving = collectWhileMoving ==1;
  }
  /**
   * 
   * @param trajectory the trajectory that the command follow.
   * @param collectWhileMoving are we wish to collect power cells while we moving 
   */  
  public FollowTrajectory(Chassis.TrajectoryOption trajectory, boolean collectWhileMoving) {
    super(new RamseteCommand(
      trajectory.getTrajectory(),
      Chassis.getInstance().getOdometry()::getPoseMeters,
      Chassis.getInstance().getrRamseteController(),
      Chassis.getInstance().getFeedforward(),
      Chassis.getInstance().getKinematics(),
     Chassis.getInstance()::getVelocity,
      new PIDController(Chassis.getInstance().getDistanceKd(), Chassis.getInstance().getDistanceKi(),Chassis.getInstance().getDistanceKd()),
      new PIDController(Chassis.getInstance().getDistanceKd(), Chassis.getInstance().getDistanceKi(),Chassis.getInstance().getDistanceKd()),
      Chassis.getInstance()::setVoltage,
      Chassis.getInstance())
     ,new CommandBase[0]); 
     
     m_collectWhileMoving = collectWhileMoving;
 }
  /**
   * 
   * @param collectWhileMoving if we wish to collect power cells while we moving put one else put zero, defualt 0;
   */
  public void setCollectWhileMoving(double collectWhileMoving){
    m_collectWhileMoving = collectWhileMoving == 1;
  }
  /**
   * 
   * @param collectWhileMoving are we wish to collect power cells while we moving 
   */
  public void setCollectWhileMoving(boolean collectWhileMoving){
    m_collectWhileMoving = collectWhileMoving;
  }
  /**
   * 
   * @return if we wish to collect power cells while we moving return one , defualt 0
   */
  public double getollectWhileMovingDbl(){
    if(m_collectWhileMoving){
      return 1;
    }
    return 0;
  }
  /**
   * 
   * @return re we wish to collect power cells while we moving 
   */
  public boolean getollectWhileMovingBool(){
    return m_collectWhileMoving;
  }
  @Override
  public void schedule() {
  if(m_collectWhileMoving){
      addCommands(Collector.getInstance().collect);
    }
    super.schedule();
  }
  @Override
  public void schedule(boolean interruptible) {
    if(m_collectWhileMoving){
      addCommands(Collector.getInstance().collect);
    }
    super.schedule(interruptible);
  }


  @Override
  public void setValue(double collectWhileMoving) {
    setCollectWhileMoving(collectWhileMoving);

  }

  @Override
  public double getValue() {
    return getollectWhileMovingDbl();
  }
}