package frc.robot;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ControlType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.SHOOTER;
import frc.robot.lib.MkUtil;

public class Shooter {

  private CANSparkMax mShooterSparkMaxLeft = new CANSparkMax(Constants.CAN.kLeftShooterId, MotorType.kBrushless);
  private CANSparkMax mShooterSparkMaxRight = new CANSparkMax(Constants.CAN.RightShootId, MotorType.kBrushless);
  private CANSparkMax mHoodSparkMax = new CANSparkMax(Constants.CAN.kHoodId, MotorType.kBrushless);

  private CANEncoder sEncoder = new CANEncoder(mShooterSparkMaxRight);
  private CANEncoder hEncoder = new CANEncoder(mHoodSparkMax);

  private CANPIDController mShooterPIDController;
  private double shooterSetpoint, hoodSetpoint;

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
    mShooterPIDController.setOutputRange(-1, 1);
  }

  public static Shooter getInstance() {
    return InstanceHolder.mInstance;
  }

  public void setShooterOutput(double percentOut) {
    mShooterSparkMaxLeft.set(percentOut);
    shooterSetpoint = percentOut;
  }

  public double getShooterRPM() {
    return sEncoder.getVelocity();
  }

  public void setShooterRPM(double rpm) {
    mShooterPIDController.setReference(rpm, ControlType.kVelocity);
    shooterSetpoint = rpm;
  }

  public void zeroHood() {
    hEncoder.setPosition(0);
  }

  public double getHoodPos() {
    return hEncoder.getPosition();
  }

  public void setHoodPos(double pos) {
    pos = MkUtil.clamp(pos, SHOOTER.kMaxHoodPos);
    mHoodSparkMax.set(MkUtil.clamp((pos - hEncoder.getPosition()) * SHOOTER.kHoodKp, SHOOTER.kMaxHoodOutput));
    hoodSetpoint = pos;
  }

  public void updateDashboard() {
    SmartDashboard.putNumber("Hood Pos", getHoodPos());
    SmartDashboard.putNumber("Hood Setpoint", hoodSetpoint);
    SmartDashboard.putNumber("Shooter RPM", getShooterRPM());
    SmartDashboard.putNumber("Shooter Setpoint (RPM/PercentOut)", shooterSetpoint);
  }

  private static class InstanceHolder {

    private static final Shooter mInstance = new Shooter();
  }
}
