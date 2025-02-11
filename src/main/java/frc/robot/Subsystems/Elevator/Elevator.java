// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ElevatorConstants;


abstract public class Elevator extends SubsystemBase {

    public double ele_setpoint, wrist_setpoint, claw_speed, shooter_speed;

    public Elevator() {
        
    }

    /**
     * Sets the target setpoint for internal elevator PID
     * @param setpoint The setpoint in meters. 0 is the lowest elevator height.
     */
    public void setElevatorSetpoint(double setpoint) {
        setpoint = MathUtil.clamp(setpoint, 0, ElevatorConstants.MAX_EXTENSION);
        this.ele_setpoint = setpoint;
        setElePID(setpoint);
    }

    public void changeSetpoint(double setpoint) {
        this.ele_setpoint += setpoint;
        setElevatorSetpoint(setpoint);
    }

    public void runShooter(double speed) {
        shooter_speed = speed;
        setShooterSpeed(speed);
    }

    // VoltageOut() methods
    public abstract void runElevator(double speed);
    public abstract void runWrist(double speed);
    // PID set methods
    protected abstract void setElePID(double setpoint);
    protected abstract void setWristPID(double setpoint);
    // Spark set methods
    protected abstract void setLeftClawSpeed(double speed);
    protected abstract void setRightClawSpeed(double speed);
    protected abstract void setShooterSpeed(double speed);

    @Override
    public void periodic() {

    } 

}
