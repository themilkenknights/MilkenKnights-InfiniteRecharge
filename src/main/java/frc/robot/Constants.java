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

  public static final double kPi = 3.14159265359;
  public static final double kDt = 0.01; // Sec

  public static class CAN {

    public static final int kDriveLeftMasterId = 8;
    public static final int kDriveLeftSlaveId = 7;
    public static final int kDriveRightMasterId = 6;
    public static final int kDriveRightSlaveId = 5;
    public static final int kLifterTalonId = 2;
    public static final int kIntakeRollerTalonId = 1;
    public static final int RightShootId = 9;
    public static final int kLeftShooterId = 12;
    public static final int kElevatorId = 4;
    public static final int kHopperId = 11;
    public static final int kHoodId = 10;

    public static final int kPCMId = 11;
    public static final int kClimbSolenoidId = 0;

    public static final boolean kLeftShooterInverted = false;
    public static final boolean kRightShooterInverted = true;
    public static final boolean kElevatorInverted = false;
  }

  public static class INPUT {

    public static final int attackMode = 2;
    public static final int defenseMode = 12;
    public static final int climbOn = 7;
    public static final int climbOff = 8;
    public static final int elevatorUp = 3;
    public static final int elevatorDown = 5;
    public static final int limeLight = 9;
  }

  public static class DRIVE {

    public static final double kWheelDiameterInches = 6;
    public static final double kWheelCircumference = kWheelDiameterInches * kPi;

    public static final boolean kLeftMasterInverted = false;
    public static final boolean kLeftSlaveInverted = false;
    public static final boolean kRightMasterInverted = true;
    public static final boolean kRightSlaveInverted = true;

    public static final boolean kLeftSensorInverted = false;
    public static final boolean kRightSensorInverted = false;

    public static final double kAntiTipThreshold = 10;

    public static final double kMaxNativeVel = 20000;
    public static final double kDriveKp = 7.0 * (0.1 * 1023.0) / (1400.0);
    public static final double kDriveKD = 3.0 * kDriveKp;
    public static final double kDriveKf = 1023.0 / DRIVE.kMaxNativeVel;
  }

  public static class LIFTER {

    public static final boolean kOutputInverted = false;
    public static final boolean kSensorInverted = false;
  }

  public static class SHOOTER {

    public static final double kFlywheelMaxVel = 5100.0;
    public static final double kFlywheelKp = 0.0022;
    public static final double kFlywheelKi = 0.0;
    public static final double kFlywheelKd = 0.008;
    public static final double kFlywheelKf = 1.0 / kFlywheelMaxVel;

    public static final double kHoodKp = 0.1;

    public static final double kMaxHoodAdjustDist = 50;
    public static final double kMaxHoodPos = 10;
  }

  public static class VISION {

    public static final double max_auto_output = 0.11;
    public static final double kP_turn = 0.0121;
    public static final double kI_turn = 0.023;
    public static final double kD_turn = 0.0018;
    public static final double angle_tol = 0.65;
    public static final double max_angular_vel = 385;
    public static final double max_angular_accel = 120;
    public static final double elevatorSlope = .002;

    public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> kRPMMap = new InterpolatingTreeMap<>();
    public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> kHoodMap = new InterpolatingTreeMap<>();

    public static double[][] kDistanceRpmValues = {
        {88.78, 2950.0}, // {Distance(Inches), RPM},
        {101.85, 3000.0},
        {113.55, 3250.0},
        {114.3, 3250.0},
        {118.0, 3320.0},
        {125.3, 3415.0},
        {135.3, 3450.0},
        {140.3, 3480.0},
        {145.3, 3505.0},
        {150.3, 3525.0},
        {173.0, 3650.0},
        {180.0, 3650.0},
        {196.3, 3650.0},
        {200.0, 3750.0},
        {205.0, 3750.0},
        {252.0, 3800.0}};

    public static double[][] kDistanceHoodValues = {
        {88.78, -2.14}, // {Distance(Inches), Hood},
        {101.85, -2.26},
        {113.55, -2.69},
        {114.3, -2.76},
        {120.3, -2.9},
        {125.3, -3.0},
        {135.3, -3.15},
        {140.3, -3.25},
        {145.3, -3.25},
        {150.3, -3.25},
        {252.0, -3.25},
        {173, -3.25},
        {196, -3.25},
        {205, -3.25},
        {180, -3.25},
        {200, -3.25}};

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
