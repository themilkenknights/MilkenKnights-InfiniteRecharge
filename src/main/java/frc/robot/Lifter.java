package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class Lifter {
    private final double kPLift = 0.025;
    private final double kDLift = kPLift * 10;

    private final double maxAnglularVel = 30; // Deg/100ms
    private final double maxAnglularAccel = 200;
    private TalonSRX mLifter = new TalonSRX(4);

    private Lifter() {
        mLifter.configFactoryDefault();
        mLifter.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
        mLifter.setInverted(false);
        mLifter.setSensorPhase(false);
        mLifter.configMotionAcceleration(degreesToNativeUnits(maxAnglularAccel));
        mLifter.configMotionCruiseVelocity(degreesToNativeUnits(maxAnglularVel));
        mLifter.configMotionSCurveStrength(4);
        mLifter.selectProfileSlot(0, 0);
        mLifter.config_kF(0, 1023.0 / degreesToNativeUnits(maxAnglularVel));
        mLifter.config_kP(0, kPLift);
        mLifter.config_kI(0, 0);
        mLifter.config_kD(0, kDLift);
        mLifter.configClosedLoopPeakOutput(0, 0.1); /// MAKE SURE TO CHANGE, LIMITS TO 50% POWER
        mLifter.setSelectedSensorPosition(0);
    }

    public void setLifterState(LifterState state) {
        mLifter.set(ControlMode.MotionMagic, degreesToNativeUnits(state.angle));
    }

    public void resetEncoder() {
        mLifter.setSelectedSensorPosition(0);
    }

    public double getArmPosition() { // Returns Degrees
        return ((mLifter.getSelectedSensorPosition()) / 4096.0) * 360.0;
    }

    public double getArmVelocity() { // Returns Degrees Per Second
        return ((mLifter.getSelectedSensorVelocity() * 10) / 4096.0) * 360.0;
    }

    public int degreesToNativeUnits(double degrees) {
        return (int) ((degrees / 360.0) * 4096.0);
    }

    public enum LifterState {
        ENABLE(0.0), // State directly after robot is enabled (not mapped to a specific angle)
        POS1(182.0), POS2(120.0);

        public final double angle;

        LifterState(final double angle) {
            this.angle = angle;
        }
    }

    public static Lifter getInstance() {
        return InstanceHolder.mInstance;
    }

    private static class InstanceHolder {
        private static final Lifter mInstance = new Lifter();
    }

}