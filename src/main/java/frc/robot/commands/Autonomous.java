package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.DRIVE;
import frc.robot.Drive;
import java.util.List;

public class Autonomous extends SequentialCommandGroup {
  /**
   * Create a new autonomous command.
   */
  public Autonomous() {

    // Create a voltage constraint to ensure we don't accelerate too fast
    var autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(DRIVE.ksVolts,
                DRIVE.kvVoltSecondsPerMeter,
                DRIVE.kaVoltSecondsSquaredPerMeter),
            DRIVE.kDriveKinematics,
            10);

    // Create config for trajectory
    TrajectoryConfig config1 =
        new TrajectoryConfig(DRIVE.kMaxSpeedMetersPerSecond,
            DRIVE.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(DRIVE.kDriveKinematics)
            // Apply the voltage constraint
            .addConstraint(autoVoltageConstraint);

    // An example trajectory to follow.  All units in meters.
    Trajectory trajectory_1 = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(
            /*new Translation2d(1, 1),
            new Translation2d(2, -1)*/
        ),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(1.5, 0, new Rotation2d(Units.degreesToRadians(30))),
        // Pass config
        config1
    );

    // Create config for trajectory
    TrajectoryConfig config2 =
        new TrajectoryConfig(DRIVE.kMaxSpeedMetersPerSecond,
            DRIVE.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(DRIVE.kDriveKinematics)
            // Apply the voltage constraint
            .addConstraint(autoVoltageConstraint)
            .setReversed(true);

    Trajectory trajectory_2 = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(1.5, 0, new Rotation2d(Units.degreesToRadians(30))),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(
            /*new Translation2d(1, 1),
            new Translation2d(2, -1)*/
        ),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(1.5, 4, new Rotation2d(Units.degreesToRadians(170))),
        // Pass config
        config2
    );

    // Create config for trajectory
    TrajectoryConfig config3 =
        new TrajectoryConfig(DRIVE.kMaxSpeedMetersPerSecond,
            DRIVE.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(DRIVE.kDriveKinematics)
            // Apply the voltage constraint
            .addConstraint(autoVoltageConstraint)
            .setReversed(false);

    Trajectory trajectory_3 = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(1.5, 0, new Rotation2d(Units.degreesToRadians(30))),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(
            /*new Translation2d(1, 1),
            new Translation2d(2, -1)*/
        ),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(1.5, 0, new Rotation2d(0)),
        // Pass config
        config3
    );

    RamseteCommand ramseteCommand = new RamseteCommand(
        trajectory_1,
        Drive.getInstance()::getPose,
        new RamseteController(),
        new SimpleMotorFeedforward(DRIVE.ksVolts,
            DRIVE.kvVoltSecondsPerMeter,
            DRIVE.kaVoltSecondsSquaredPerMeter),
        DRIVE.kDriveKinematics,
        Drive.getInstance()::getWheelSpeeds,
        new PIDController(DRIVE.kPDriveVel, 0, 0, Constants.kDt),
        new PIDController(DRIVE.kPDriveVel, 0, 0, Constants.kDt),
        // RamseteCommand passes volts to the callback
        Drive.getInstance()::setVoltage,
        Drive.getInstance()
    );

    addCommands(
        deadline(ramseteCommand, new IntakeBalls()),
        deadline(new DriveStraight(70.0), new IntakeBalls()),
        deadline(new TurnInPlace(-90.0), new IntakeBalls()),
        deadline(new DriveStraight(10), new IntakeBalls()),
        deadline(new TurnInPlace(-90.0), new IntakeBalls()),
        new DriveStraight(65.5), new TurnInPlace(-40.0)
        /*new LimelightShoot(),*/
    );
  }
}
