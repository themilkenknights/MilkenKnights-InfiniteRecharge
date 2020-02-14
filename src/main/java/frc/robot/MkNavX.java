/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.kauailabs.navx.frc.AHRS;

public class MkNavX extends AHRS {
    public AHRS navX = new AHRS();
    public double oldPos = 0;
    public double yawVelocity = 0;

    public MkNavX() {
        super();
    }

    public void resetYawVelocity() {
        oldPos = navX.getYaw();
    }

    public double getYawVelocity() {
        if (navX.getYaw() - oldPos != 0) {
            yawVelocity = (navX.getYaw() - oldPos) / (Constants.time.get() - Robot.lastTime);
        } else {
            return 0;
        }
        oldPos = navX.getYaw();
        return (yawVelocity);
    }

}
