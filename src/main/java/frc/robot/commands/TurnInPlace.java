package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Drive;

public class TurnInPlace extends CommandBase {
  private double angle;

  //Currently Negative Turns Left (Front of robot is intake)
  public TurnInPlace(double angle) {
    this.angle = angle;
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    Drive.getInstance().setMagicTurnInPlace(angle);
  }

  @Override
  public void execute() {
    Drive.getInstance().magicTurnInPlaceUpdate();
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return Drive.getInstance().isMagicTurnInPlaceDone();
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {

  }
}
