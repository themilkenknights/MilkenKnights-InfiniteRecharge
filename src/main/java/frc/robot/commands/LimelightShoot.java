package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Limelight;
import frc.robot.Shooter;

public class LimelightShoot extends CommandBase {

  public LimelightShoot() {

  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {

  }

  @Override
  public void execute() {
    Limelight.getInstance().autoAimShoot();
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
    Shooter.getInstance().setHoodPos(0);
  }
}
