package com.evergreen.everlib.subsystems.motors.subsystems;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.SpeedController;



/**
 * This is a wrapper class for {@link SpeedController}, which allowes easier and more generic
 * construction of SpeedControlelr objects.
 * 
 * @author Atai Ambus
 */
public class MotorController implements SpeedController {

    /**
     * MotorInitialize
     */
    public interface MotorInitializer {
        SpeedController generate(int channel);
    }

    /**
     * A list of all motors this objects controlls.
      */
    private final ArrayList<SpeedController> m_motors = new ArrayList<>();


    private final List<CANEncoder> m_encoders = new ArrayList<>();


    /**
     * Constructs a new {@link MotorController} which controlls one of more motors with the same 
     * type of electronic controller.
     * 
     * @param type - The {@link ControllerType} type of the electronic speed controllers that 
     * produce the voltage for the motors. (Victor, Talon, ect.)
     * 
     * @param ports - The roborio ports the voltage controllers conect to.
     */
    public MotorController(ControllerType type, int... ports) {
        
        //intializing them.
        for(int i = 1; i < ports.length; i++)
        {
            getMotors().add(type.initlize(ports[i]));

            if (type == ControllerType.SPARKMAX_BRUSHED || type == ControllerType.SPARKMAX_BRUSHLESS)
                m_encoders.add( 
                    ((CANSparkMax)getMotors().get(i))
                    .getEncoder());
        }
    }


    public List<SpeedController> getMotors() {
        return m_motors;
    }

    /**
     * @return A list of all CAN encoders this motor uses.
     */
    public List<CANEncoder> getCANEncoders() {
        return m_encoders;
    }

    /**
     * Returns the encoder of the first SparkMax this controller uses. 
     * Usefull if there is only one.
     */
    public CANEncoder getCANEncoder() {
        return m_encoders.get(0);
    }

    /**
     * Consrtucts a {@link MotorController} which combines multiple other MotorControllers.
     * Also works as a copy constructor. <p>
     * 
     * The new Controller's methods will aplly on all the given controllers.
     * 
     * @param controllers - The controllers to combine.
     */
    public MotorController(MotorController... controllers)
    {
        
        //Foreach of the motors, add its controller to the controller list.
        for(MotorController controller : controllers)
        {
            getMotors().addAll(controller.getMotors());
            m_encoders.addAll(controller.getCANEncoders());
        }
    }

    @Override
    public void set(double speed) {

        for (SpeedController controller : getMotors()) {
            controller.set(speed);   
        }
    }

    public void setAt(int index, double speed) {
        getMotors().get(index).set(speed);
    }

    @Override
    public double get() {

        double sum = 0;
        
        for (SpeedController controller : getMotors()) {
            sum += controller.get();
        }

        return sum / getMotors().size();
    }

    public double getAt(int index) {
        return getMotors().get(index).get();
    }

    @Override
    public void setInverted(boolean isInverted) {
        for (SpeedController controller : getMotors()) {
            controller.setInverted(isInverted);
        }
    }

    public void setInverted(int index, boolean isInverted) {
        getMotors().get(index).setInverted(isInverted);
    }

    @Override
    public boolean getInverted() {
        
        for (SpeedController controller : getMotors()) {
            if (!controller.getInverted()) return false;
        }

        return true;
    }

    public boolean getInverted(int index) {
        return getMotors().get(index).getInverted();
    }

    @Override
    public void disable() {
        
        for (SpeedController controller : getMotors()) {
            controller.disable();
        }
    }

    public void disableAt(int index) {
        getMotors().get(index).disable();
    }

    @Override
    public void stopMotor() {
        for (SpeedController controller : getMotors() ) {
            controller.stopMotor();
        }
    }

    /**A controller model - Victor SPX, Talon SRX, ect. */
    public enum ControllerType
    {
        VICTOR_SPX(WPI_VictorSPX::new),
        TALON_SRX(WPI_TalonSRX::new),
        SPARKMAX_BRUSHED( (port) -> new CANSparkMax(port, MotorType.kBrushed)),
        SPARKMAX_BRUSHLESS( (port) -> new CANSparkMax(port, MotorType.kBrushless));

        MotorInitializer m_init;

        SpeedController initlize(int channel) {
            return m_init.generate(channel);
        }

        ControllerType(MotorInitializer init) {
            m_init = init;
        }
    }

    @Deprecated
    @Override
    public void pidWrite(double output) {
        for (SpeedController controller : getMotors()) {
            controller.pidWrite(output);
        }
    }
}
