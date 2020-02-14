/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * Add your docs here.
 */
public final class Constants {

    public double maxShooterVel = 0;
    public static double kP = 1.0;
    public static double kD = 1.0;
    public static double kPElevator = 0;
    public static double kDElevator = 0;

    static MkNavX navX = new MkNavX();

    public static class Drive {
        public static final int rightMasterId = 0;
        public static final int leftMasterId = 1;
        public static final int rightSlaveId = 2;
        public static final int leftSlaveId = 3;

        public static final double wheelDiameterInches = 6; // Inches
        public static final double pi = 3.1415926535;
    }

}
