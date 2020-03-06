package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Drive;
import frc.robot.lib.MkUtil;

public class FollowPath extends CommandBase {
  private final Timer m_timer = new Timer();
  private final Trajectory m_trajectory;
  private final RamseteController m_follower;
  private final SimpleMotorFeedforward m_feedforward;
  private final DifferentialDriveKinematics m_kinematics;
  private DifferentialDriveWheelSpeeds m_prevSpeeds;
  private double m_prevTime;

  private final PIDController m_leftController;
  private final PIDController m_rightController;

  public FollowPath(Trajectory trajectory) {
    m_trajectory = trajectory;
    m_follower = new RamseteController();
    m_feedforward = Constants.PATHING.kFeedforward;
    m_kinematics = Constants.PATHING.kDriveKinematics;
    m_leftController = new PIDController(Constants.PATHING.kPDriveVel, 0, 0);
    m_rightController = new PIDController(Constants.PATHING.kPDriveVel, 0, 0);
  }

  @Override
  public void initialize() {
    m_prevTime = 0;
    var initialState = m_trajectory.sample(0);
    m_prevSpeeds = m_kinematics.toWheelSpeeds(
        new ChassisSpeeds(initialState.velocityMetersPerSecond,
            0,
            initialState.curvatureRadPerMeter
                * initialState.velocityMetersPerSecond));
    m_timer.reset();
    m_timer.start();
    m_leftController.reset();
    m_rightController.reset();
  }

  @Override
  public void execute() {
    double curTime = m_timer.get();
    double dt = curTime - m_prevTime;

    var targetWheelSpeeds = m_kinematics.toWheelSpeeds(
        m_follower.calculate(Drive.getInstance().getPose(), m_trajectory.sample(curTime)));

    var leftSpeedSetpoint = targetWheelSpeeds.leftMetersPerSecond;
    var rightSpeedSetpoint = targetWheelSpeeds.rightMetersPerSecond;

    double leftFeedforward = m_feedforward.calculate(leftSpeedSetpoint, (leftSpeedSetpoint - m_prevSpeeds.leftMetersPerSecond) / dt);

    double rightFeedforward = m_feedforward.calculate(rightSpeedSetpoint, (rightSpeedSetpoint - m_prevSpeeds.rightMetersPerSecond) / dt);

    double leftVel = MkUtil.metersPerSecondToNativeUnitsPer100Ms(leftSpeedSetpoint);

    double rightVel = MkUtil.metersPerSecondToNativeUnitsPer100Ms(rightSpeedSetpoint);

    Drive.getInstance().setOutput(leftVel, rightVel, leftFeedforward, rightFeedforward);

    m_prevTime = curTime;
    m_prevSpeeds = targetWheelSpeeds;
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
