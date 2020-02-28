/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;


import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {

  private Joystick stick = new Joystick(0);
  private Joystick jStick = new Joystick(1);
  private Compressor mCompressor = new Compressor(0);
  private double HoodPos = 0;

  @Override
  public void robotInit() {
    //mCompressor.start();
  }

  @Override
  public void autonomousInit() {
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() 
  {
    Shooter.getInstance().zeroHood();
    HoodPos = 0;
    //mCompressor.stop();
    
  }

  @Override
  public void teleopPeriodic() {
    input();
    Shuffle.getInstance().Update();

    SmartDashboard.putNumber("Shooter RPM", Shooter.getInstance().getShooterRPM());
  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
  }

  public void input() {
    if (jStick.getRawButton(8)) {
      Drive.getInstance().setOutput(Limelight.getInstance().update());
    } else {
      double forward, turn, rightOut, leftOut;
      forward = (Math.pow(stick.getRawAxis(3) - stick.getRawAxis(2), 5) + Drive.getInstance().AntiTip()); // this gets how far forward the forward stick
                                                                        // is
      turn = stick.getRawAxis(0); // this gets out left or right the turn stick is
      rightOut = forward - turn; // This sets the turn distance for arcade drive
      leftOut = forward + turn;
      Drive.getInstance().setOutput(new Drive.DriveSignal(leftOut, rightOut));
    }
    // Run Shooter
    if (jStick.getRawButton(1))
      Shooter.getInstance().setShooterOutput(0.25);
    else
      Shooter.getInstance().setShooterOutput(0.00);

    // Run Elevator Up and Down
    if (jStick.getRawButton(6)) {
      Elevator.getInstance().setElevatorOutput(0.75, .5);
    } else if (jStick.getRawButton(4)) {
      Elevator.getInstance().setElevatorOutput(-0.5, -.3);
    } else if (jStick.getRawButton(9)) {
      Elevator.getInstance().setElevatorOutput(-0.75, -.5);

    } else
      Elevator.getInstance().setElevatorOutput(0.00, 0.00);


    if (jStick.getRawButton(2)) {
      Intake.getInstance().setIntakeRoller(1);
    } else {
      Intake.getInstance().setIntakeRoller(0.0);
    }

    if (jStick.getRawButtonPressed(7)) {
      Climber.ToggleClimb();
    }
    if (jStick.getRawButtonPressed(10)) {
      ElevatorStop.ToggleStopper();
    }
    if(jStick.getPOV() == 0)
    {
      HoodPos += .1;
    }
    else if(jStick.getPOV() == 180)
    {
      HoodPos -= .1;
    }
    
    if (jStick.getRawButton(12)) 
    {
      AttackMode();
    }
    else if(jStick.getRawButton(11))
    {
      DefenceMode();
    }

    Shooter.getInstance().setHoodPos(HoodPos);
    
  }


  public void AttackMode()
  {
    Intake.getInstance().setIntakeRoller(.75);
    Intake.getInstance().setIntakeState(true);
    Elevator.getInstance().setElevatorOutput(.25, 0.00);
    ElevatorStop.getInstance().setStopper(true);
  }

  public void DefenceMode()
  {
    Intake.getInstance().setIntakeRoller(0.5);
    Intake.getInstance().setIntakeState(false);
    Elevator.getInstance().setElevatorOutput(0.5, 0.50);
    ElevatorStop.getInstance().setStopper(true);
    Shooter.getInstance().setShooterOutput(.80);
  }

  public void Shoot(double TargetRPM, double HoodPos)
  {
    Shooter.getInstance().setShooterOutput(TargetRPM);
    Shooter.getInstance().setHoodPos(HoodPos);

    if(Shooter.getInstance().getShooterRPM() > TargetRPM)
    {
      Elevator.getInstance().setElevatorOutput(.75, .5);
      ElevatorStop.getInstance().setStopper(false);
    }
    else
    {
      Elevator.getInstance().setElevatorOutput(.25, .5);
    }
  }
}