package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import frc.robot.Constants.LIFTER;
import frc.robot.Constants.CAN;

public class Lifter {
  private TalonSRX lifterTalon = new TalonSRX(CAN.lifterTalonId);

  private Lifter() {
    lifterTalon.configFactoryDefault();
    lifterTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    lifterTalon.setInverted(LIFTER.outputInverted);
    lifterTalon.setSensorPhase(LIFTER.sensorInverted);
    lifterTalon.configMotionAcceleration(degreesToNativeUnits(LIFTER.maxAnglularAccel));
    lifterTalon.configMotionCruiseVelocity(degreesToNativeUnits(LIFTER.maxAnglularVel));
    lifterTalon.configMotionSCurveStrength(4);
    lifterTalon.selectProfileSlot(0, 0);
    lifterTalon.config_kF(0, 1023.0 / degreesToNativeUnits(LIFTER.maxAnglularVel));
    lifterTalon.config_kP(0, LIFTER.kP);
    lifterTalon.config_kI(0, 0);
    lifterTalon.config_kD(0, LIFTER.kD);
    lifterTalon.configClosedLoopPeakOutput(0, 0.1); /// MAKE SURE TO CHANGE, LIMITS TO 10% POWER
    lifterTalon.setSelectedSensorPosition(0);
  }

  public void setLifterState(LifterState state) {
    lifterTalon.set(ControlMode.MotionMagic, degreesToNativeUnits(state.angle));
  }

  public void resetEncoder() {
    lifterTalon.setSelectedSensorPosition(0);
  }

  public double getArmPosition() { // Returns Degrees
    return ((lifterTalon.getSelectedSensorPosition()) / 4096.0) * 360.0;
  }

  public double getArmVelocity() { // Returns Degrees Per Second
    return ((lifterTalon.getSelectedSensorVelocity() * 10) / 4096.0) * 360.0;
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
