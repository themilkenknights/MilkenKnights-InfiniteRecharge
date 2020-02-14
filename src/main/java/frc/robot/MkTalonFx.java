/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonFX;

/**
 * Add your docs here.
 */
public class MkTalonFx extends TalonFX {
    public double distancePerPulse = 0.0;
    public boolean reversed = false;
    public TalonFX m_talon;

    public MkTalonFx(int deviceNumber) {
        super(deviceNumber);
        m_talon = new TalonFX(deviceNumber);
    }

    public void reverse(boolean a) {
        reversed = a;
    }

    public void setDistancePerPulse(double x) {
        distancePerPulse = x;
    }

    public void reset() {
        m_talon.setSelectedSensorPosition(0);
    }

    public double getDistance() {
        if (reversed) {
            return -(distancePerPulse * m_talon.getSelectedSensorPosition());
        } else {
            return (distancePerPulse * m_talon.getSelectedSensorPosition());
        }
    }

}
