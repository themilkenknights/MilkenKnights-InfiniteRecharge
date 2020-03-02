package frc.robot;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ControlType;

public class Shooter {
  private CANSparkMax mShooterSparkMaxLeft = new CANSparkMax(Constants.CAN.LeftShooterId, MotorType.kBrushless);
  private CANSparkMax mShooterSparkMaxRight = new CANSparkMax(Constants.CAN.RightShootId, MotorType.kBrushless);
  private CANSparkMax mHoodSparkMax = new CANSparkMax(Constants.CAN.HoodId, MotorType.kBrushless);

  private CANEncoder sEncoder = new CANEncoder(mShooterSparkMaxRight);
  private CANEncoder hEncoder = new CANEncoder(mHoodSparkMax);

  private CANPIDController mShooterPIDController;

  private double kp = .1;

  private Shooter() {
    mShooterSparkMaxLeft.restoreFactoryDefaults();
    mShooterSparkMaxRight.restoreFactoryDefaults();

    mShooterSparkMaxLeft.setInverted(Constants.CAN.LeftShooterInvered);
    mShooterSparkMaxRight.setInverted(Constants.CAN.RightShooterInvered);
    sEncoder.setVelocityConversionFactor(2.0 / 3.0); //Integer Divison Is Bad!!
    sEncoder = mShooterSparkMaxLeft.getEncoder();

    mShooterSparkMaxRight.follow(mShooterSparkMaxLeft);

    mShooterPIDController = mShooterSparkMaxLeft.getPIDController();

    mShooterPIDController.setP(0.1);
    mShooterPIDController.setI(0);
    mShooterPIDController.setD(1.0);
    mShooterPIDController.setFF(1.0 / 4000); // 1.0/MAX_RPM
    mShooterPIDController.setOutputRange(-1, 1);
  }

  public void setShooterOutput(double percentOut) {
    mShooterSparkMaxLeft.set(percentOut);
    mShooterSparkMaxRight.set(percentOut);
  }

  public void setShooterRPM(double rpm) {
    mShooterPIDController.setReference(rpm, ControlType.kVelocity);
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
