package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class IntakeRoller {

    TalonSRX mTalon = new TalonSRX(9);

    private IntakeRoller() {
        mTalon.configFactoryDefault();
        mTalon.setInverted(false);
    }

    public void setIntakeRoller(double percentOut) {
        mTalon.set(ControlMode.PercentOutput, percentOut);
    }

    public static IntakeRoller getInstance() {
        return InstanceHolder.mInstance;
    }

    private static class InstanceHolder {
        private static final IntakeRoller mInstance = new IntakeRoller();
    }
}