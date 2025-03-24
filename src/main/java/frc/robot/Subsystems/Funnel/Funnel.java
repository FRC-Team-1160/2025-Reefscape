// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Funnel;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

abstract public class Funnel extends SubsystemBase {

    public static final Funnel instance = Robot.isReal() ? new FunnelRealIO() : new FunnelSimIO();

    private int count;

    private FunnelState m_state = FunnelState.kOff;

    public enum FunnelState {
        kOn(-0.7, 0.7),
        kOff(0, 0),
        kReverse(0.7, -0.7);

        public final double left, right;
        private FunnelState(double left, double right) {
            this.left = left;
            this.right = right;
        }
    }

    /** Creates a new ServoSystem. */
    protected Funnel() {}

    public void setState(FunnelState target) {
        m_state = target;
        setLeftServo(target.left);
        setRightServo(target.right);
    }
    
    @AutoLogOutput
    public boolean getDown() {
        return count > 150;
    }

    protected abstract void setLeftServo(double setpoint);
    protected abstract void setRightServo(double setpoint);

    @Override
    public void periodic() {
        if (m_state != FunnelState.kOff) {
            count++;
        }
    }
}
