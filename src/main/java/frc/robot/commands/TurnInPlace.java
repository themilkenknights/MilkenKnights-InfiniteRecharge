package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Drive;

public class TurnInPlace extends CommandBase {
  private double angle;
  private Timer mTimer = new Timer();

  public TurnInPlace(double angle) {
    this.angle = angle;
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    mTimer.start();
    Drive.getInstance().setMagicTurnInPlace(angle);
  }

  @Override
  public void execute() {
    Drive.getInstance().magicTurnInPlaceUpdate();
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return Drive.getInstance().isMagicOnTarget() || mTimer.hasElapsed(3.0);
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {

  }
}
