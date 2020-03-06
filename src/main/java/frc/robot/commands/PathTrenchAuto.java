package frc.robot.commands;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import java.util.List;

public class PathTrenchAuto extends SequentialCommandGroup {
  public PathTrenchAuto() {
    addCommands(
        deadline(new FollowPath(Constants.PATHING.traj_1), new IntakeBalls()),
        deadline(new TurnInPlace(-120.0).withTimeout(2.5), new IntakeBalls()),
        deadline(new DriveStraight(115).withTimeout(3), new IntakeBalls(), new SpinUp(3600)),
        deadline(new TurnInPlace(-30.0).withTimeout(1.5), new IntakeBalls(), new SpinUp(3600)),
        new LimelightShoot()
    );
  }
}
