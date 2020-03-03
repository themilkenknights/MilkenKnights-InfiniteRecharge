/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import frc.robot.lib.InterpolatingDouble;
import frc.robot.lib.InterpolatingTreeMap;

/**
 * Add your docs here.
 */
public final class Constants {

  public static final double PI = 3.14159265359;

  public static class CAN {

    public static final int driveLeftMasterId = 8;
    public static final int driveLeftSlaveId = 7;
    public static final int driveRightMasterId = 6;
    public static final int driveRightSlaveId = 5;
    public static final int lifterTalonId = 2;
    public static final int intakeRollerTalonId = 1;
    public static final int RightShootId = 9;
    public static final int LeftShooterId = 12;
    public static final int ElevatorId = 4;
    public static final int HopperId = 11;
    public static final int HoodId = 10;

    public static final int PCMId = 11;
    public static final int ClimbSolenoidId = 0;

    public static final boolean LeftShooterInverted = false;
    public static final boolean RightShooterInverted = true;
    public static final boolean elevatorInverted = false;
  }

  public static class INPUT {

    public static final int attackMode = 2;
    public static final int defenceMode = 12;
    public static final int climbOn = 7;
    public static final int climbOff = 8;
    public static final int elevatorUp = 3;
    public static final int elevatorDown = 5;
    public static final int limeLight = 9;

    // public static final int button 1 is shooterControlEnable and forward and back tics for speed management.
    //hood pov
  }

  public static class DRIVE {

    public static final double wheelDiameterInches = 6; // Inches
    public static final double wheelCircumference = wheelDiameterInches * PI; // Inches

    public static final boolean leftMasterInverted = false;
    public static final boolean leftSlaveInverted = false;
    public static final boolean rightMasterInverted = true;
    public static final boolean rightSlaveInverted = true;

    public static final boolean leftSensorInverted = false;
    public static final boolean rightSensorInverted = false;

    public static final double AngleThresholdDegrees = 15;
    public static final double KAngle = 1;
  }

  public static class LIFTER {

    public static final boolean outputInverted = false;
    public static final boolean sensorInverted = false;
    public static final double kP = 0.025;
    public static final double kD = kP * 10;
    public static final double maxAnglularVel = 30; // Deg/100ms
    public static final double maxAnglularAccel = 200;
  }

  public static class SHOOTER {

    public final static double kP = 1.0;
    public final static double kD = 1.0;
    public final static double kPElevator = 0;
    public final static double kDElevator = 0;
    public final static double maxShooterVel = 0;
    public final static double maxHoodAdjustDist = 50;
    public final static double maxHoodPos = 10;

  }

  public static class VISION {

    public final static double max_auto_output = 0.5;
    public final static double kP_dist = 0.005;
    public final static double kD_dist = .0005;
    public final static double dist_tol = 0.5;
    public final static double max_dist = 75;
    public final static double min_dist = 60;
    public final static double max_linear_vel = 100;
    public final static double max_linear_accel = 140;
    public final static double kP_turn = 0.03;
    public final static double kD_turn = .005;
    public final static double angle_tol = 0.3;
    public final static double max_angular_vel = 19265.0; // Deg/Sec @TODO These are definitely way too high. Try something like 1200 Deg/Sec
    public final static double max_angular_accel = 151917.25529762334; // Deg/Sec^2 Try 2000 Deg/Sec^2

    public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> kRPMMap = new InterpolatingTreeMap<>();

    public static double[][] kDistanceRpmValues = {{90.0, 2890.0}, //{Distance(Inches), RPM},
        {95.0, 2940.0}, {100.0, 2990.0}, {105.0, 3025.0},};

    static {
      for (double[] pair : kDistanceRpmValues) {
        kRPMMap.put(new InterpolatingDouble(pair[0]), new InterpolatingDouble(pair[1]));
      }
    }
  }
}
