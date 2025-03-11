// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Funnel;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.FunnelConstants.ServoLeft;
import frc.robot.Constants.FunnelConstants.ServoRight;

abstract public class Funnel extends SubsystemBase {

    public static final Funnel instance = Robot.isReal() ? new FunnelRealIO() : new FunnelSimIO();

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
        setLeftServo(target.left);
        setRightServo(target.right);
    }

    protected abstract void setLeftServo(double setpoint);
    protected abstract void setRightServo(double setpoint);

    @Override
    public void periodic() {}
}
