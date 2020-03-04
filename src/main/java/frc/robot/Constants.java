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
  public static final double kDt = 0.01; // Sec
  public static final double intakeSpeed = 0.5;

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
    public static final int defenseMode = 12;
    public static final int climbOn = 7;
    public static final int climbOff = 8;
    public static final int elevatorUp = 3;
    public static final int elevatorDown = 5;
    public static final int limeLight = 9;

    // public static final int button 1 is shooterControlEnable and forward and back
    // tics for speed
    // management.
    // hood pov
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

    public static final double kMaxNativeVel = 20000;
    public static final double kP = 1.0 * 7.0 * (0.1 * 1023.0) / (1400.0);
    public static final double kD = 3.0 * kP;
    public static final double kF = 1023.0 / DRIVE.kMaxNativeVel;
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

    public static final double kP = 1.0;
    public static final double kD = 1.0;
    public static final double kPElevator = 0;
    public static final double kDElevator = 0;
    public static final double maxShooterVel = 0;
    public static final double maxHoodAdjustDist = 50;
    public static final double maxHoodPos = 10;
  }

  public static class VISION {

    public static final double max_auto_output = 0.325;
    public static final double kP_dist = 0.0035;
    public static final double kD_dist = 0.0;
    public static final double dist_tol = 0.5;
    public static final double max_dist = 170;
    public static final double min_dist = 60;
    public static final double max_linear_vel = 100;
    public static final double max_linear_accel = 140;
    public static final double kP_turn = 0.0121;
    public static final double kI_turn = 0.023;
    public static final double kD_turn = 0.0018;
    public static final double angle_tol = 0.65;
    public static final double max_angular_vel = 400;
    public static final double max_angular_accel = 200; 

    public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> kRPMMap = new InterpolatingTreeMap<>();
    public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> kHoodMap = new InterpolatingTreeMap<>();

    public static double[][] kDistanceRpmValues = { 
      { 88.78, 2950.0 }, // {Distance(Inches), RPM},
      { 101.85, 3000.0 },
      { 113.55, 3250.0 },
      { 114.3, 3250.0 },
      { 118.0, 3320.0 },
      { 125.3, 3425.0 },
      { 135.3, 3450.0 },
      { 140.3, 3480.0 },
      { 145.3, 3505.0 },
      { 150.3, 3525 }, };

      public static double[][] kDistanceHoodValues = { 
        { 88.78, -2.14 }, // {Distance(Inches), RPM},
        { 101.85, -2.26 },
        { 113.55, -2.69 },
        { 114.3, -2.76 },
        { 120.3, -2.9 },
        { 125.3, -3.0},
        { 135.3, -3.15 }, 
        { 140.3, -3.25 },
        { 145.3, -3.25  },
        { 150.3, -3.25  }, };

    static {
      for (double[] pair : kDistanceRpmValues) {
        kRPMMap.put(new InterpolatingDouble(pair[0]), new InterpolatingDouble(pair[1]));
      }
      for (double[] pair : kDistanceHoodValues) {
        kHoodMap.put(new InterpolatingDouble(pair[0]), new InterpolatingDouble(pair[1]));
      }
    }
  }
  
}
