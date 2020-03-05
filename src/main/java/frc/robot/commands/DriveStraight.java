package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Drive;

public class DriveStraight extends CommandBase {
  private double dist;
  private Timer mTimer = new Timer();

  public DriveStraight(double dist) {
    this.dist = dist;
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    mTimer.start();
    Drive.getInstance().setDriveStraight(dist);
  }

  @Override
  public void execute() {
    Drive.getInstance().updateDriveStraight();
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return Drive.getInstance().isDriveStraightDone() || mTimer.hasElapsed(5.0);
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {

  }
}
