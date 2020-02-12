package com.evergreen.robot.wpilib;

import java.util.function.Supplier;

import com.evergreen.robot.wpilib.commands.WaitCommandEG;
import com.evergreen.robot.wpilib.subsystem.Shooter;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

/**
 * Autonomous
 */
public class Autonomous extends SequentialCommandGroup {
    /**
     * the number of options that we wish to have.
     */
    public static final int OPTIONS_NUMBER = 15;
    private SendableChooser<CommandBase>[] m_options = new SendableChooser[OPTIONS_NUMBER];
    private Supplier<Double>[] m_arguments = new Supplier[OPTIONS_NUMBER];
   
    public Autonomous() {
        for (int i = 0; i < OPTIONS_NUMBER; i++) {
            final int j = i;
            Preferences.getInstance().putDouble("arg" + i,0);
            m_arguments[i] = () -> Preferences.getInstance().getDouble("arg" + j, 0);
            m_options[i] = new SendableChooser();
            m_options[i].setDefaultOption("wait",new WaitCommandEG(m_arguments[i].get()));
            m_options[i].addOption("shootToUpper", Shooter.getInstance().shootToUpper);
            m_options[i].addOption("shootToBottom", Shooter.getInstance().shootToBottom);
            m_options[i].addOption("drop", Shooter.getInstance().drop);
            //TODO: add:
            //be ready for shooting
            //drive inside the sector
            //drive to somwhere
            //turn
            //collect balls
    

            
        }
    }
    public void update(){
        for(int i = 0; i<OPTIONS_NUMBER;i++){
          if (m_options[i] instanceof DoubleArgCommand) {
                ((DoubleArgCommand)(m_options[i].getSelected())).setValue(m_arguments[i].get());
           
            }
        }
    }
}