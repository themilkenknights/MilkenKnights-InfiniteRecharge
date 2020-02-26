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
  public static final double PI = 3.14159265359;

  public static class CAN {
    public static final int driveLeftMasterId = 8;
    public static final int driveLeftSlaveId = 7;
    public static final int driveRightMasterId = 6;
    public static final int driveRightSlaveId = 5;
    public static final int lifterTalonId = 2;
    public static final int intakeRollerTalonId = 3;
    public static final int RightShootId = 9;
    public static final int LeftShooterId = 12;
    public static final int ElevatorId = 4;
    public static final int Hopper = 11;



    public static final int PCMId = 11;
    public static final int ClimbSolenoidId = 0;

    public static final boolean LeftShooterInvered = false;
    public static final boolean RightShooterInvered = true;
    public static final boolean elevatorInverted = false;
    
  }

  public static class DRIVE {
    public static final double wheelDiameterInches = 6; // Inches
    public static final double wheelCircumference = wheelDiameterInches * PI; // Inches

    public static final boolean leftMasterInverted = false;
    public static final boolean leftSlaveInverted = false;
    public static final boolean rightMasterInverted = true;
    public static final boolean rightSlaveInverted = true;

    public static final boolean leftSensorInverted = false;
    public static final boolean rightSensorInverted = true;

    public static final double AngleThresholdDegrees = 10;
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
    public final double maxShooterVel = 0;
    public final static double kP = 1.0;
    public final static double kD = 1.0;
    public final static double kPElevator = 0;
    public final static double kDElevator = 0;
  }
}
