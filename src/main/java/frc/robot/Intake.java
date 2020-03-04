package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.Solenoid;

public class Intake {

  private final Solenoid intakeSolenoid = new Solenoid(0);
  private final TalonSRX mTalon = new TalonSRX(Constants.CAN.kIntakeRollerTalonId);
  private final CANSparkMax hopperSparkMax = new CANSparkMax(Constants.CAN.kHopperId, MotorType.kBrushless);
  private IntakeState mState = IntakeState.STOW;

  private Intake() {
    mTalon.configFactoryDefault();
    mTalon.setInverted(false);
    hopperSparkMax.restoreFactoryDefaults();
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

  public void setIntakeState(IntakeState pos) {
    if (pos != mState) {
      intakeSolenoid.set(pos.state);
      mState = pos;
    }
  }

  public enum IntakeState {
    INTAKE(true), STOW(false);

    public final boolean state;

    IntakeState(final boolean state) {
      this.state = state;
    }
  }

  private static class InstanceHolder {

    private static final Intake mInstance = new Intake();
  }
}
