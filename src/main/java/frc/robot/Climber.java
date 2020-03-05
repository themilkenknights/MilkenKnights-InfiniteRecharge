package frc.robot;

import edu.wpi.first.wpilibj.Solenoid;

public class Climber {

  private final Solenoid climbSolenoid = new Solenoid(1);
  private ClimbState mState;

  public Climber() {
    climbSolenoid.set(false);
    mState = ClimbState.RETRACT;
  }

  public static Climber getInstance() {
    return InstanceHolder.mInstance;
  }

  public void toggleClimb() {
    if (mState == ClimbState.RETRACT) {
      setClimbState(ClimbState.CLIMB);
    } else {
      setClimbState(ClimbState.RETRACT);
    }
  }

  public void setClimbState(ClimbState state) {
    if (mState != state) {
      climbSolenoid.set(state.state);
      mState = state;
    }
  }

  public enum ClimbState {
    CLIMB(true), RETRACT(false);

    public final boolean state;

    ClimbState(final boolean state) {
      this.state = state;
    }
  }

  private static class InstanceHolder {

    private static final Climber mInstance = new Climber();
  }
}
