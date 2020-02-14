/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;

/**
 * Add your docs here.
 */
public class Constants {

    public double maxShooterVel = 0;
    public static double kP = 1.0;
    public static double kD = 1.0;
    public static double kPElevator = 0;
    public static double kDElevator = 0;

    static Joystick stick = new Joystick(0);
    static Joystick jStick = new Joystick(1);

    static Timer time = new Timer();

    static MkNavX navX = new MkNavX();
    static MkTalonFx rightMaster = new MkTalonFx(0);
    static MkTalonFx leftMaster = new MkTalonFx(1);
    static MkTalonFx rightSlave = new MkTalonFx(2);
    static MkTalonFx leftSlave = new MkTalonFx(3);

    static TalonSRX lifter = new TalonSRX(9);
    static TalonSRX intakeRoller = new TalonSRX(18);

}
