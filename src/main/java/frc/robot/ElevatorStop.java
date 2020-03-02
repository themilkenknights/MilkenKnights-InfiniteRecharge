/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * Add your docs here.
 */

import edu.wpi.first.wpilibj.Solenoid;

public class ElevatorStop {
    public static Solenoid stopperSolenoid = new Solenoid(2);

    public ElevatorStop()
    {
        stopperSolenoid.set(false);
    }

    public static void ToggleStopper() {
        if (stopperSolenoid.get())
            stopperSolenoid.set(false);
        else
            stopperSolenoid.set(true);
    }

    public void setStopper(boolean state) {
        stopperSolenoid.set(state);
    }

    public static ElevatorStop getInstance() {
        return InstanceHolder.mInstance;
    }

    private static class InstanceHolder {
        private static final ElevatorStop mInstance = new ElevatorStop();
    }

}
