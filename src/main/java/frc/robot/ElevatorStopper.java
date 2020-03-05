package frc.robot;

import edu.wpi.first.wpilibj.Solenoid;

public class ElevatorStopper {

  private final Solenoid stopperSolenoid = new Solenoid(2);
  private StopperState mState;

  public ElevatorStopper() {
    stopperSolenoid.set(false);
    mState = StopperState.STOP;
  }

  public static ElevatorStopper getInstance() {
    return InstanceHolder.mInstance;
  }

  public void toggleStopper() {
    if (mState == StopperState.STOP) {
      setStopper(StopperState.GO);
    } else {
      setStopper(StopperState.STOP);
    }
  }

  public void setStopper(StopperState state) {
    if (state != mState) {
      stopperSolenoid.set(state.state);
      mState = state;
    }
  }

  public enum StopperState {
    GO(true), STOP(false);

    public final boolean state;

    StopperState(final boolean state) {
      this.state = state;
    }
  }

  private static class InstanceHolder {

    private static final ElevatorStopper mInstance = new ElevatorStopper();
  }
}
