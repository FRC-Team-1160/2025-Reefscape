// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Funnel;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FunnelConstants.ServoLeft;
import frc.robot.Constants.FunnelConstants.ServoRight;

abstract public class Funnel extends SubsystemBase {

    enum FunnelState {
        kUp(ServoLeft.UP, ServoRight.UP),
        kDown(ServoLeft.DOWN, ServoRight.DOWN);

        public final double left, right;
        private FunnelState(double left, double right) {
            this.left = left;
            this.right = right;
        }
    }

    /** Creates a new ServoSystem. */
    public Funnel() {
        
    }

    public void setState(FunnelState target) {
        setLeftServo(target.left);
        setRightServo(target.right);
    }

    protected abstract void setLeftServo(double setpoint);
    protected abstract void setRightServo(double setpoint);

    @Override
    public void periodic() {}
}
