package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Drive;
import frc.robot.lib.MkUtil;

public class FollowPath extends CommandBase {
  private final Timer m_timer = new Timer();
  private final Trajectory m_trajectory;
  private final RamseteController m_follower;
  private final DifferentialDriveKinematics m_kinematics;

  public FollowPath(Trajectory trajectory) {
    m_trajectory = trajectory;
    m_follower = new RamseteController();
    m_kinematics = Constants.PATHING.kDriveKinematics;
  }

  @Override
  public void initialize() {
    m_timer.reset();
    m_timer.start();
  }

  @Override
  public void execute() {
    double curTime = m_timer.get();

    var targetWheelSpeeds = m_kinematics.toWheelSpeeds(m_follower.calculate(Drive.getInstance().getPose(), m_trajectory.sample(curTime)));

    var leftSpeedSetpoint = targetWheelSpeeds.leftMetersPerSecond;
    var rightSpeedSetpoint = targetWheelSpeeds.rightMetersPerSecond;

    double leftVel = MkUtil.metersPerSecondToNativeUnitsPer100Ms(leftSpeedSetpoint);

    double rightVel = MkUtil.metersPerSecondToNativeUnitsPer100Ms(rightSpeedSetpoint);

    Drive.getInstance().setOutput(leftVel, rightVel, 0, 0);
  }

  @Override
  public void end(boolean interrupted) {
    m_timer.stop();
  }

  @Override
  public boolean isFinished() {
    return m_timer.hasElapsed(m_trajectory.getTotalTimeSeconds());
  }
}
