package frc.robot;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Shooter {

  CANSparkMax mShooterSparkMaxLeft = new CANSparkMax(Constants.CAN.LeftShooterId, MotorType.kBrushless);
  CANSparkMax mShooterSparkMaxRight = new CANSparkMax(Constants.CAN.RightShootId, MotorType.kBrushless);
  CANSparkMax mHoodSparkMax = new CANSparkMax(Constants.CAN.HoodId, MotorType.kBrushless);

  CANEncoder sEncoder = new CANEncoder(mShooterSparkMaxRight);
  CANEncoder hEncoder = new CANEncoder(mHoodSparkMax);

  private double kp = .1;

  private Shooter() {
    mShooterSparkMaxLeft.setInverted(Constants.CAN.LeftShooterInvered);
    mShooterSparkMaxRight.setInverted(Constants.CAN.RightShooterInvered);
    sEncoder.setVelocityConversionFactor(2 / 3);
  }

  public void setShooterOutput(double percentOut) {
    mShooterSparkMaxLeft.set(percentOut);
    mShooterSparkMaxRight.set(percentOut);
  }

  public double getShooterRPM() {
    return sEncoder.getVelocity();
  }

  public static Shooter getInstance() {
    return InstanceHolder.mInstance;
  }

  public void zeroHood() {
    hEncoder.setPosition(0);
  }

  public void setHoodPos(double Pos) {
    mHoodSparkMax.set((Pos - hEncoder.getPosition()) * kp);
  }

  public double getHoodPos() {
    return hEncoder.getPosition();
  }

  private static class InstanceHolder {
    private static final Shooter mInstance = new Shooter();
  }
}