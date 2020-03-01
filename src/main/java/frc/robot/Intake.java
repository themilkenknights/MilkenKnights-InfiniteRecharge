package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Solenoid;

public class Intake {

  TalonSRX mTalon = new TalonSRX(Constants.CAN.intakeRollerTalonId);
  public static Solenoid intakeSolenoid = new Solenoid(0);

  private Intake() {
    mTalon.configFactoryDefault();
    mTalon.setInverted(false);
    intakeSolenoid.set(false);
  }

  public void setIntakeRoller(double percentOut) {
    mTalon.set(ControlMode.PercentOutput, percentOut);
  }

  public void setIntakeState(boolean Pos) {
    intakeSolenoid.set(Pos);
  }

  public static Intake getInstance() {
    return InstanceHolder.mInstance;
  }

  private static class InstanceHolder {
    private static final Intake mInstance = new Intake();
  }
}
