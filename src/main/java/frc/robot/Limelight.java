package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import frc.robot.Constants.VISION;

public class Limelight {
  TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(VISION.max_angular_vel, VISION.max_angular_accel);
  ProfiledPIDController m_turn_controller = new ProfiledPIDController(VISION.kP_turn, VISION.kI_turn, VISION.kD_turn,
      constraints);
  ProfiledPIDController m_dist_controller = new ProfiledPIDController(VISION.kP_dist, 0.0, VISION.kD_dist,
      new TrapezoidProfile.Constraints(VISION.max_angular_vel, VISION.max_angular_accel));
  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry tx = table.getEntry("tx");
  NetworkTableEntry ty = table.getEntry("ty");
  NetworkTableEntry ta = table.getEntry("ta");

  private Limelight() {
    m_turn_controller.setTolerance(VISION.angle_tol);
    m_dist_controller.setTolerance(VISION.dist_tol);
    m_turn_controller.setIntegratorRange(-150.0, 150.0);
  }

  public static Limelight getInstance() {
    return InstanceHolder.mInstance;
  }

  public boolean inRange() {
    return /*m_dist_controller.atGoal() &&*/Math.abs(tx.getDouble(0.0)) < Constants.VISION.angle_tol;
  }

  public Drive.DriveSignal update() {
    // Get the horizontal and vertical angle we are offset by
    double horizontal_angle = tx.getDouble(0.0);
    double vertical_angle = ty.getDouble(0.0);
    double current_dist = getDistance();

    SmartDashboard.putNumber("Horizontal Angle", horizontal_angle);
    SmartDashboard.putNumber("Vertical Angle", vertical_angle);
    SmartDashboard.putNumber("Target Distance", current_dist);
    // Goal angle - current angle

    double forward_output = 0;
    double turn_output = 0;

    // Have a deadband where we are close enough
    if (Math.abs(horizontal_angle) > VISION.angle_tol) {
      // Get PID controller output
      TrapezoidProfile trap = new TrapezoidProfile(constraints, new TrapezoidProfile.State(0,0));
      turn_output = m_turn_controller.calculate(-horizontal_angle, 0) + ((1.0)/(VISION.max_angular_vel)) * trap.calculate(Constants.kDt).velocity; 
    }

    // Have a deadband where we are close enough

    /*if (current_dist > VISION.max_dist) {
      forward_output = m_dist_controller.calculate(current_dist, VISION.max_dist);
    } else if (getDistance() < VISION.min_dist) {
      forward_output = m_dist_controller.calculate(current_dist, VISION.min_dist);
    }*/

    return new Drive.DriveSignal(clamp(forward_output + turn_output), clamp(forward_output - turn_output));
  }

  public double getDistance() {
    double height_camera_to_target = (89.75 - 33); //inches
    double lightlight_pitch = 17.7; //Degrees from horizontal
    return (height_camera_to_target / Math.tan(Math.toRadians(lightlight_pitch + ty.getDouble(0.0))));
  }

  public double clamp(double a) {
    return Math.abs(a) < VISION.max_auto_output ? a : Math.copySign(VISION.max_auto_output, a);
  }

  private static class InstanceHolder {

    private static final Limelight mInstance = new Limelight();
  }
}
