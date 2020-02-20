package frc.robot;

import edu.wpi.first.wpilibj.Solenoid;

/**
 * Add your docs here.
 */
public class Climber {
    public Solenoid climbSolenoid = new Solenoid(0);
    private ClimbState climbState = ClimbState.RETRACTED;

    public enum ClimbState {
        RETRACTED(false), LOWERED(true);

        public final boolean state;

        ClimbState(final boolean state) {
            this.state = state;
        }
    }

    public void setClimbState(ClimbState state) {
        climbState = state;
        climbSolenoid.set(state.state);
    }


    public static Climber getInstance() {
        return InstanceHolder.mInstance;
    }

    private static class InstanceHolder {
        private static final Climber mInstance = new Climber();
    }

}
