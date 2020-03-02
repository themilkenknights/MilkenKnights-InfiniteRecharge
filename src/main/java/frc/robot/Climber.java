package frc.robot;

import edu.wpi.first.wpilibj.Solenoid;

/**
 * Add your docs here.
 */
public class Climber {
    public static Solenoid climbSolenoid = new Solenoid(1);

    public Climber()
    {
        climbSolenoid.set(false);
    }

    public static void ToggleClimb() {
        if (climbSolenoid.get())
            climbSolenoid.set(false);
        else
            climbSolenoid.set(true);

    }
    public static void setClimbState(boolean state) {
            climbSolenoid.set(state);

    }

    public static Climber getInstance() {
        return InstanceHolder.mInstance;
    }

    private static class InstanceHolder {
        private static final Climber mInstance = new Climber();
    }

}
