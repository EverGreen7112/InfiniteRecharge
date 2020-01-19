package com.evergreen.robot;

import java.util.concurrent.TimeUnit;
import java.util.function.Supplier;

import com.evergreen.everlib.CommandEG;
import com.evergreen.everlib.subsystems.motors.commands.MoveMotorSystem;
import com.evergreen.robot.SubsystemComponents.ShooterComponents;

/**
 * ShooterAimer used to aim the shooter - to raise it or to move it down or to
 * toggle it. 
 */
public class ShooterAimer extends MoveMotorSystem implements ShooterComponents, SubsystemConstants {

    AimOption m_aimOption;

    public ShooterAimer(String name, AimOption aimOption) {
        super(name, Robot.m_aimer, aimOption.m_aimingSpeed);
        m_aimOption = aimOption;

    }
    //TODO: not sure about try catch block, ask atai
    @Override
    public void initialize() {
        Robot.m_aimer.move(m_aimOption.m_aimingSpeed.get());
        try {
            TimeUnit.SECONDS.sleep(m_aimOption.m_aimingTime.get());
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        Robot.m_aimer.move(0);
    }
    /**
     * The aim option and it aiming speed and aiming time suppliers.
     * <ul>
     * <li>objects: TOGGLE, RAISE, MOVE_DOWN</li>
     * </ul>
    */
    public enum AimOption implements SubsystemConstants {
        TOGGLE(() -> {
            if (Utillities.getAimPosition()) {
                return ShooterConstant.aimingDownSpeed.get();
            }
            return ShooterConstant.aimingUpSpeed.get();
        }, () -> {
            if (Utillities.getAimPosition()) {
                return Double.valueOf(ShooterConstant.aimingDownTime.get()).longValue();
            }
            return Double.valueOf(ShooterConstant.aimingUpTime.get()).longValue();
        }), RAISE(ShooterConstant.aimingUpSpeed, () -> Double.valueOf(ShooterConstant.aimingUpTime.get()).longValue()),
        MOVE_DOWN(ShooterConstant.aimingDownSpeed,
                () -> Double.valueOf(ShooterConstant.aimingDownTime.get()).longValue());

        public Supplier<Double> m_aimingSpeed;
        public Supplier<Long> m_aimingTime;

        private AimOption(Supplier<Double> aimingSpeed, Supplier<Long> aimingTime) {
            m_aimingSpeed = aimingSpeed;
            m_aimingTime = aimingTime;
        }
    }

}