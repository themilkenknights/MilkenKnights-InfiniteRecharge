package frc.robot;

import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import frc.robot.lib.InterpolatingDouble;
import frc.robot.lib.InterpolatingTreeMap;

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
    public static final int limeLight = 1;
    public static final int limeAdjust = 11;

    // public static final int button 1 is shooterControlEnable and forward and back tics for speed management. hood pov
  }

  public static class DRIVE {

    public static final double kWheelDiameterInches = 5.9575;
    public static final double kWheelCircumference = kWheelDiameterInches * kPi;

    public static final boolean kLeftMasterInverted = false;
    public static final boolean kLeftSlaveInverted = false;
    public static final boolean kRightMasterInverted = true;
    public static final boolean kRightSlaveInverted = true;

    public static final double kAntiTipThreshold = 15;

    public static final double kMaxNativeVel = 1685;
    public static final double kDriveKp = 7.0 * (0.1 * 1023.0) / (1400.0);
    public static final double kDriveKi = 0;
    public static final double kDriveKD = 3.0 * kDriveKp;
    public static final double kDriveKf = 1023.0 / DRIVE.kMaxNativeVel;

    public static final int kMotionMagicStraightVel = (int) (0.65 * DRIVE.kMaxNativeVel); //Increase to drive straight faster, decrease to slow
    public static final int kMotionMagicStraightAccel = (int) (0.4 * DRIVE.kMaxNativeVel); //Increase to drive straight faster, decrease to slow

    public static final int kMotionMagicTurnInPlaceVel = (int) (0.35 * DRIVE.kMaxNativeVel); //May Need to lower this if turning overshoots
    public static final int kMotionMagicTurnInPlaceAccel = (int) (0.15 * DRIVE.kMaxNativeVel); //May Need to lower this if turning overshoots

    //False So The Limit Only Applies In Neutral (ie when stopping). 2nd param is current limit, 3rd is current limit threshold
    //4th param is limit threshold duration
    //The current limit threshold must be exceeded for the 'threshold duration' seconds to enable the current limit
    public static final StatorCurrentLimitConfiguration config = new StatorCurrentLimitConfiguration(false, 5, 20, 0.05);
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

    public static final double kHoodKp = 0.15;
    public static final double kMaxHoodPos = -3.3;
    public static final double kMaxHoodOutput = 0.25;
  }

  public static class VISION {

    public static final double max_auto_output = 0.11;
    public static final double kP_turn = 0.0121;
    public static final double kI_turn = 0.0;
    public static final double kD_turn = 0.00325;
    public static final double angle_tol = 2.5;
    public static final double angle_do_nothing_tol = 0.1;
    public static final double max_angular_vel = 300;
    public static final double max_angular_accel = 120;
    public static final double elevatorSlope = .002;
    public static final int limelight_Pipeline = 0;

    public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> kRPMMap = new InterpolatingTreeMap<>();
    public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> kHoodMap = new InterpolatingTreeMap<>();

    public static double[][] kDistanceRpmValues = {
        {88.78, 2950.0}, // {Distance(Inches), RPM},
        {101.85, 3200.0},
        {106.9, 3180.0},
        {111.55, 3200.0},
        {114.3, 3250.0},
        {118.0, 3320.0},
        {125.3, 3370.0},
        {135.3, 3425.0},
        {140.3, 3480.0},
        {145.3, 3505.0},
        {150.3, 3525.0},
        {173.0, 3565.0},
        {180.0, 3585.0},
        {196.3, 3600.0},
        {200.0, 3650.0},
        {205.0, 3700.0},
        {220.0, 3725.0},
        {252.0, 3800.0}};

    public static double[][] kDistanceHoodValues = {
        // {Distance(Inches), Hood},
        {88.78, -2.52},
        {101.85, -2.75},
        {106.85, -3.00},
        {113.55, -3.05},
        {114.3, -3.1},
        {120.3, -3.15},
        {125.3, -3.25},
        {135.3, -3.25},
        {140.3, -3.25},
        {145.3, -3.25},
        {150.3, -3.25},
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

  public static class PATHING {
    public static final double kTrackwidthMeters = 0.69;
    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackwidthMeters);

    public static final double ksVolts = 0.266;
    public static final double kvVoltSecondsPerMeter = 2.47;
    public static final double kaVoltSecondsSquaredPerMeter = 0.148;

    // Example value only - as above, this must be tuned for your drive!
    public static final double kPDriveVel = 5.88;
    public static final double kMaxSpeedMetersPerSecond = 2.54;
    public static final double kMaxAccelerationMetersPerSecondSquared = 2.54;
  }
}
