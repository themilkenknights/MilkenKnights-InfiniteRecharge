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
    public static Solenoid stopperSolenoid = new Solenoid(1);



    public static void ToggleStopper(){
        if(stopperSolenoid.get())
            stopperSolenoid.set(false);
        else
            stopperSolenoid.set(true);
    }


    public static ElevatorStop getInstance() {
        return InstanceHolder.mInstance;
    }

    private static class InstanceHolder {
        private static final ElevatorStop mInstance = new ElevatorStop();
    }

}
