package frc.robot;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ControlType;
import frc.robot.Constants.SHOOTER;

public class Shooter {

  private final CANSparkMax mShooterSparkMaxLeft = new CANSparkMax(Constants.CAN.kLeftShooterId, MotorType.kBrushless);
  private final CANSparkMax mShooterSparkMaxRight = new CANSparkMax(Constants.CAN.RightShootId, MotorType.kBrushless);
  private final CANSparkMax mHoodSparkMax = new CANSparkMax(Constants.CAN.kHoodId, MotorType.kBrushless);

  private CANEncoder sEncoder = new CANEncoder(mShooterSparkMaxLeft);
  private CANEncoder hEncoder = new CANEncoder(mHoodSparkMax);

  private final CANPIDController mShooterPIDController;
  private double hoodSetpoint;

  private Shooter() {
    mShooterSparkMaxLeft.restoreFactoryDefaults();
    mShooterSparkMaxRight.restoreFactoryDefaults();

    mShooterSparkMaxRight.follow(mShooterSparkMaxLeft, true);
    mShooterSparkMaxLeft.setInverted(Constants.CAN.kLeftShooterInverted);
    mShooterSparkMaxRight.setInverted(Constants.CAN.kRightShooterInverted);
    mShooterSparkMaxLeft.enableVoltageCompensation(12.0);
    mShooterSparkMaxRight.enableVoltageCompensation(12.0);

    sEncoder.setVelocityConversionFactor(2.0 / 3.0);
    sEncoder = mShooterSparkMaxLeft.getEncoder();

    mShooterPIDController = mShooterSparkMaxLeft.getPIDController();
    mShooterPIDController.setP(SHOOTER.kFlywheelKp);
    mShooterPIDController.setI(SHOOTER.kFlywheelKi);
    mShooterPIDController.setD(SHOOTER.kFlywheelKd);
    mShooterPIDController.setFF(SHOOTER.kFlywheelKf);
  }

  public static Shooter getInstance() {
    return InstanceHolder.mInstance;
  }

  public void setShooterOutput(double percentOut) {
    mShooterSparkMaxLeft.set(percentOut);
  }

  public double getShooterRPM() {
    return sEncoder.getVelocity();
  }

  public void setShooterRPM(double rpm) {
    mShooterPIDController.setReference(rpm, ControlType.kVelocity);
  }

  public void zeroHood() {
    hEncoder.setPosition(0);
  }

  public double getHoodPos() {
    return hEncoder.getPosition();
  }

  public double getHoodSetpoint() {
    return hoodSetpoint;
  }

  public void setHoodPos(double pos) {
    mHoodSparkMax.set((pos - hEncoder.getPosition()) * SHOOTER.kHoodKp);
    hoodSetpoint = pos;
  }

  private static class InstanceHolder {

    private static final Shooter mInstance = new Shooter();
  }
}
