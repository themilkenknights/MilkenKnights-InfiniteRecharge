package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import frc.robot.Constants.VISION;
import frc.robot.lib.InterpolatingDouble;
import frc.robot.lib.MkUtil.DriveSignal;

public class Limelight {

  private final TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(VISION.max_angular_vel, VISION.max_angular_accel);
  private final ProfiledPIDController m_turn_controller = new ProfiledPIDController(VISION.kP_turn, VISION.kI_turn, VISION.kD_turn, constraints);
  private final NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  private final NetworkTableEntry tx = table.getEntry("tx");
  private final NetworkTableEntry ty = table.getEntry("ty");
  private final Timer shootTimer = new Timer();
  private boolean shootOn = false;

  private Limelight() {
    m_turn_controller.setTolerance(VISION.angle_tol);
    m_turn_controller.setIntegratorRange(-100.0, 100.0);
  }

  public static Limelight getInstance() {
    return InstanceHolder.mInstance;
  }

  public void resetInt() {
    m_turn_controller.reset(tx.getDouble(0.0));
  }

  public boolean inRange() {
    return Math.abs(tx.getDouble(0.0)) < Constants.VISION.angle_tol;
  }

  public void autoAimShoot() {
    Intake.getInstance().setIntakeRoller(0.0);
    Intake.getInstance().setIntakeState(Intake.IntakeState.STOW);
    Drive.getInstance().setOutput(Limelight.getInstance().update());
    double curDist = Limelight.getInstance().getDistance();
    double RPM = Constants.VISION.kRPMMap.getInterpolated(new InterpolatingDouble(curDist)).value;
    double hoodDist = Constants.VISION.kHoodMap.getInterpolated(new InterpolatingDouble(curDist)).value;
    Shooter.getInstance().setShooterRPM(RPM);
    Shooter.getInstance().setHoodPos(hoodDist);
    Elevator.getInstance().setElevatorOutput(.79 - Constants.VISION.elevatorSlope * Limelight.getInstance().getDistance());
    //Intake.getInstance().setHopperRoller(0.05);
    SmartDashboard.putNumber("Map RPM", RPM);
    SmartDashboard.putNumber("Map Hood Pos", hoodDist);
    boolean isInRange = Limelight.getInstance().inRange();
    SmartDashboard.putBoolean("In Range", isInRange);
    if (isInRange && Math.abs(Shooter.getInstance().getShooterRPM() - RPM) < 30) {
      ElevatorStopper.getInstance().setStopper(ElevatorStopper.StopperState.GO);
      shootTimer.start();
      shootOn = true;
    } else if (shootTimer.hasElapsed(0.25) || !shootOn) {
      shootTimer.reset();
      shootOn = false;
      ElevatorStopper.getInstance().setStopper(ElevatorStopper.StopperState.STOP);
    }
  }

  public DriveSignal update() {
    // Get the horizontal and vertical angle we are offset by
    double horizontal_angle = tx.getDouble(0.0);
    double vertical_angle = ty.getDouble(0.0);
    double current_dist = getDistance();

    SmartDashboard.putNumber("Horizontal Angle", horizontal_angle);
    SmartDashboard.putNumber("Vertical Angle", vertical_angle);
    SmartDashboard.putNumber("Target Distance", current_dist);
    // Goal angle - current angle
    double turn_output = 0;

    // Have a deadband where we are close enough
    if (Math.abs(horizontal_angle) > VISION.angle_tol) {
      // Get PID controller output
      TrapezoidProfile trap = new TrapezoidProfile(constraints, new TrapezoidProfile.State(0, 0));
      turn_output = clamp(m_turn_controller.calculate(-horizontal_angle, 0) + ((1.0) / (VISION.max_angular_vel)) * trap.calculate(Constants.kDt).velocity);
    }

    return new DriveSignal(turn_output, -turn_output);
  }

  public double getDistance() {
    return getDistance(ty.getDouble(0.0));
  }

  public double getDistance(double ty) {
    double height_camera_to_target = (89.75 - 33); //inches
    double lightlight_pitch = 17.7; //Degrees from horizontal
    return (height_camera_to_target / Math.tan(Math.toRadians(lightlight_pitch + ty)));
  }

  public double clamp(double a) {
    return Math.abs(a) < VISION.max_auto_output ? a : Math.copySign(VISION.max_auto_output, a);
  }

  private static class InstanceHolder {

    private static final Limelight mInstance = new Limelight();
  }
}
