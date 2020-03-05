package frc.robot;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.ControlType;
import frc.robot.Constants.SHOOTER;

public class Shooter {

  private CANSparkMax mShooterSparkMaxLeft = new CANSparkMax(Constants.CAN.kLeftShooterId, MotorType.kBrushless);
  private CANSparkMax mShooterSparkMaxRight = new CANSparkMax(Constants.CAN.RightShootId, MotorType.kBrushless);
  private CANSparkMax mHoodSparkMax = new CANSparkMax(Constants.CAN.kHoodId, MotorType.kBrushless);

  private CANEncoder sEncoder = new CANEncoder(mShooterSparkMaxRight);
  private CANEncoder hEncoder = new CANEncoder(mHoodSparkMax);

  private CANPIDController mShooterPIDController;

  private double kp = .1;

  private Shooter() {
    mShooterSparkMaxLeft.restoreFactoryDefaults();
    mShooterSparkMaxRight.restoreFactoryDefaults();

    mShooterSparkMaxRight.follow(mShooterSparkMaxLeft, true);

    mShooterSparkMaxLeft.setInverted(Constants.CAN.kLeftShooterInverted);
    mShooterSparkMaxRight.setInverted(Constants.CAN.kRightShooterInverted);
    mShooterSparkMaxLeft.enableVoltageCompensation(12.0);
    mShooterSparkMaxRight.enableVoltageCompensation(12.0);

    sEncoder.setVelocityConversionFactor(2.0 / 3.0); //Integer Divison Is Bad!!
    sEncoder = mShooterSparkMaxLeft.getEncoder();


    mShooterPIDController = mShooterSparkMaxLeft.getPIDController();

    mShooterPIDController.setP(0.0022);
    mShooterPIDController.setI(0);
    mShooterPIDController.setD(0.008);//0.0075
    mShooterPIDController.setFF(1.0 / 5100); // 1.0/MAX_RPM
    mShooterPIDController.setOutputRange(-1, 1);
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

  public void setHoodPos(double Pos) {
    SmartDashboard.putNumber("desired hood pos", Pos);
    mHoodSparkMax.set(Math.abs((Pos - hEncoder.getPosition()) * kp) > 0.4 ? 0.4 : (Pos - hEncoder.getPosition()) * kp);
  }

  public void setHoodFromTargetDist(double dist) {
    /*if (dist < SHOOTER.maxHoodAdjustDist) {
      setHoodPos(SHOOTER.maxHoodPos);
    } else {
      //TODO: Modify hood based on distance here
      setHoodPos(dist * 0.5);
    }*/
  }

  private static class InstanceHolder {

    private static final Shooter mInstance = new Shooter();
  }
}