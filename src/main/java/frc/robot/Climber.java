package frc.robot;

import edu.wpi.first.wpilibj.Solenoid;

/**
 * Add your docs here.
 */
public class Climber {

  public static Solenoid climbSolenoid = new Solenoid(1);

  public Climber() {
    climbSolenoid.set(false);
  }

  public static Climber getInstance() {
    return InstanceHolder.mInstance;
  }

  public void toggleClimb() {
    if (climbSolenoid.get()) {
      climbSolenoid.set(false);
    } else {
      climbSolenoid.set(true);
    }
  }

  public void setClimbState(boolean state) {
    climbSolenoid.set(state);
  }

  private static class InstanceHolder {

    private static final Climber mInstance = new Climber();
  }

}
