package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
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

public class TrenchAuto extends SequentialCommandGroup {
  /**
   * Create a new autonomous command.
   */
  public TrenchAuto() {   
    addCommands(
        deadline(new DriveStraight(98.0), new IntakeBalls()),
        deadline(new TurnInPlace(-90.0), new IntakeBalls()),
        deadline(new DriveStraight(6.0), new IntakeBalls()),
        deadline(new TurnInPlace(-30.0), new IntakeBalls()),
        deadline(new DriveStraight(115), new IntakeBalls(), new SpinUp()),
        deadline(new TurnInPlace(-30.0), new IntakeBalls(), new SpinUp()),
        new LimelightShoot()
    );
  }
}
