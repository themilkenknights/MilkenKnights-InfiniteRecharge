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
import frc.robot.lib.MkUtil;
import frc.robot.lib.MkUtil.DriveSignal;

public class Limelight {

  private final TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(VISION.max_angular_vel, VISION.max_angular_accel);
  private final ProfiledPIDController m_turn_controller = new ProfiledPIDController(VISION.kP_turn, VISION.kI_turn, VISION.kD_turn, constraints);
  private final NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  private final NetworkTableEntry tx = table.getEntry("tx");
  private final NetworkTableEntry ty = table.getEntry("ty");
  private final NetworkTableEntry led = table.getEntry("ledMode");
  private final NetworkTableEntry tv = table.getEntry("tv");
  private final NetworkTableEntry pipeline = table.getEntry("pipeline");
  private final Timer shootTimer = new Timer();
  private boolean hasTarget;
  private double distance, visionYaw, visionPitch;

  private Limelight() {
    table.getEntry("pipeline").setValue(Constants.VISION.limelight_Pipeline);
  }

  public static Limelight getInstance() {
    return InstanceHolder.mInstance;
  }

  public void updateSensors() {
    visionYaw = tx.getDouble(0.0);
    visionPitch = ty.getDouble(0.0);
    distance = getDistance(visionPitch);
    hasTarget = tv.getDouble(0.0) != 0.0f; //If tv returns 0, no valid target
  }

  public boolean inRange() {
    return hasTarget && Math.abs(visionYaw) < Constants.VISION.angle_tol && Math.abs(Drive.getInstance().getAvgVel()) < Constants.VISION.vel_tol;
  }

  public void autoAimShoot(boolean ignoreAim) {
    Intake.getInstance().setIntakeRoller(0.0);
    Intake.getInstance().setIntakeState(Intake.IntakeState.STOW);
    update();
    double curDist = getDistance();
    double RPM = Constants.VISION.kRPMMap.getInterpolated(new InterpolatingDouble(curDist)).value;
    double hoodDist = Constants.VISION.kHoodMap.getInterpolated(new InterpolatingDouble(curDist)).value;
    Shooter.getInstance().setShooterRPM(RPM);
    SmartDashboard.putNumber("Target RPM", RPM);
    Shooter.getInstance().setHoodPos(limit(hoodDist, -3.25, 0));
    double distance_mod = Constants.VISION.elevatorSlope * Limelight.getInstance().getDistance();
    double rpm_mod = VISION.rpmSlope * (RPM - Shooter.getInstance().getShooterRPM());
    Elevator.getInstance().setElevatorOutput(limit(0.60 - rpm_mod, 0.2, 1.0));
    if ((ignoreAim || inRange()) && Math.abs(Shooter.getInstance().getShooterRPM() - RPM) < 25) {
      ElevatorStopper.getInstance().setStopper(ElevatorStopper.StopperState.GO);
    }
  }

  public void update() {
    double horizontal_angle = tx.getDouble(0.0);
    double turn_output = 0;
    if (Math.abs(horizontal_angle) > VISION.angle_do_nothing_tol) {
      TrapezoidProfile trap = new TrapezoidProfile(constraints, new TrapezoidProfile.State(0, 0));
      double turn_controllout_out = m_turn_controller.calculate(-horizontal_angle, 0);
      double feedforward = ((1.0) / (VISION.max_angular_vel)) * trap.calculate(Constants.kDt).velocity;
      turn_output = MkUtil.clamp(turn_controllout_out + feedforward, Constants.VISION.max_auto_output);
    }
    Drive.getInstance().setOutput(new DriveSignal(turn_output, -turn_output));
  }

  public double getDistance() {
    return distance;
  }

  public void toggleLED() {
    if (led.getDouble(0.0) == 1) {
      table.getEntry("ledMode").setValue(3);
    } else {
      table.getEntry("ledMode").setValue(1);
    }
  }

  public double getDistance(double ty) {
    double height_camera_to_target = (89.75 - 33); //inches
    double lightlight_pitch = 17.7; //Degrees from horizontal
    return (height_camera_to_target / Math.tan(Math.toRadians(lightlight_pitch + ty)));
  }

  public void updateDashboard() {
    SmartDashboard.putNumber("Horizontal Angle", visionYaw);
    SmartDashboard.putNumber("Vertical Angle", visionPitch);
    SmartDashboard.putNumber("Target Distance", distance);
    SmartDashboard.putBoolean("In Range", inRange());
  }

  public double limit(double value, double min, double max) {
    if (value > max) {
      return max;
    } else if (value < min) {
      return min;
    } else {
      return value;
    }
  }

  private static class InstanceHolder {

    private static final Limelight mInstance = new Limelight();
  }
}
