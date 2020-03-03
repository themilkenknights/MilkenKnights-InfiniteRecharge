package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.Solenoid;

public class Intake {

  public static Solenoid intakeSolenoid = new Solenoid(0);
  TalonSRX mTalon = new TalonSRX(Constants.CAN.intakeRollerTalonId);
  CANSparkMax hopperSparkMax = new CANSparkMax(Constants.CAN.HopperId, MotorType.kBrushless);

  private Intake() {
    mTalon.configFactoryDefault();
    mTalon.setInverted(false);
    intakeSolenoid.set(false);
  }

  public static Intake getInstance() {
    return InstanceHolder.mInstance;
  }

  public void setIntakeRoller(double percentOut) {
    mTalon.set(ControlMode.PercentOutput, percentOut);
  }

  public void setHopperRoller(double percentOut) {
    hopperSparkMax.set(percentOut);
  }

  public void setIntakeState(boolean pos) {
    intakeSolenoid.set(pos);
  }

  private static class InstanceHolder {

    private static final Intake mInstance = new Intake();
  }
}
