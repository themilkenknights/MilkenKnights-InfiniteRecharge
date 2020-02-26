package frc.robot;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Shooter {

    CANSparkMax mShooterSparkMaxLeft = new CANSparkMax(Constants.CAN.LeftShooterId, MotorType.kBrushless);
    CANSparkMax mShooterSparkMaxRight = new CANSparkMax(Constants.CAN.RightShootId, MotorType.kBrushless);
    CANEncoder sEncoder = new CANEncoder(mShooterSparkMaxRight);
    private Shooter(){
        mShooterSparkMaxLeft.setInverted(Constants.CAN.LeftShooterInvered);
        mShooterSparkMaxRight.setInverted(Constants.CAN.RightShooterInvered);
        sEncoder.setVelocityConversionFactor(2/3);
    }
    
  public void setShooterOutput(double percentOut) {
    mShooterSparkMaxLeft.set(percentOut);
    mShooterSparkMaxRight.set(percentOut);
  }

  public double getShooterRPM()
  {
    return sEncoder.getVelocity();
  }

  public static Shooter getInstance() {
    return InstanceHolder.mInstance;
  }

  private static class InstanceHolder {
    private static final Shooter mInstance = new Shooter();
  }
}