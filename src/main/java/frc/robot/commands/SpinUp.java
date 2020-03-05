package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Elevator;
import frc.robot.ElevatorStopper;
import frc.robot.ElevatorStopper.StopperState;
import frc.robot.Intake;
import frc.robot.Shooter;
import frc.robot.Intake.IntakeState;

public class SpinUp extends CommandBase {

  public SpinUp() {

  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {

  }

  @Override
  public void execute() {
   Shooter.getInstance().setShooterRPM(3600);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return true;
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {

  }
}
