// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.DriveTrain;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotState;

public abstract class SwerveModule {

    public SwerveModuleState target_state;

    public SwerveModule() {
        target_state = new SwerveModuleState();

    }

    public void setState(SwerveModuleState state) {
        target_state = state;
    }

    public void update() {
        if (!RobotState.isDisabled()) { //just in case, idk
            setSpeed(target_state.speedMetersPerSecond);
            setAngle(target_state.angle);
        }
    }

    abstract double getSpeed();
    abstract double getPosition();
    abstract Rotation2d getAngle();
    abstract SwerveModuleState getModuleState();
    abstract SwerveModulePosition getModulePosition();
    
    protected abstract void setSpeed(double speed);
    protected abstract void setAngle(Rotation2d angle);
}
