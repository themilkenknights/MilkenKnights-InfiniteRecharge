/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Solenoid;

/**
 * Add your docs here.
 */
public class Climber {
    public Solenoid climbSolenoid = new Solenoid(Constants.CAN.PCMId, Constants.CAN.ClimbSolenoidId);
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
