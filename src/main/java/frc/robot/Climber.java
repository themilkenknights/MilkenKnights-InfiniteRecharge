package frc.robot;

import edu.wpi.first.wpilibj.Solenoid;

/**
 * Add your docs here.
 */
public class Climber {
    public static Solenoid climbSolenoid = new Solenoid(1);

    public static void ToggleClimb() {
        if (climbSolenoid.get())
            climbSolenoid.set(false);
        else
            climbSolenoid.set(true);
    }

    public static Climber getInstance() {
        return InstanceHolder.mInstance;
    }

    private static class InstanceHolder {
        private static final Climber mInstance = new Climber();
    }

}
