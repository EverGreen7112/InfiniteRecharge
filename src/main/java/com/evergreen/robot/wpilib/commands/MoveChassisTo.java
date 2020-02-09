package com.evergreen.robot.wpilib.commands;

<<<<<<< HEAD
import com.evergreen.robot.wpilib.Chassis;

=======
>>>>>>> wpilib_chassis
import edu.wpi.first.wpilibj2.command.PIDCommand;

/**
 * MoveChassisTo
 */
public class MoveChassisTo extends PIDCommand {

<<<<<<< HEAD
    double m_distance;

    public MoveChassisTo(double distance) {
        super(Chassis.getInstance().getDistancePID(), 
        () -> Chassis.getInstance().getDistance(), () -> distance, Chassis.getInstance()::move, Chassis.getInstance());
    }

    @Override
    public boolean isFinished() {
        return ((getController().getPositionError())<=Chassis.getInstance().getPIDDistanceTolerance());
    }

=======
    public MoveChassisTo(double distance) {
        super
    }
>>>>>>> wpilib_chassis
}