// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Elevator;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


abstract public class Elevator extends SubsystemBase {

  /** The height which the elevator travels toward, in meters. 0 is the bottom of the range of motion. */
  public double ele_setpoint;

  public double wrist_setpoint;

  public double shooter_speed;

  public Elevator() {
    
  }

  public void setElevatorSetpoint(double setpoint) {
    this.ele_setpoint = setpoint;
    (setpoint);
  }

  public void changeSetpoint(double setpoint) {
    this.ele_setpoint += setpoint;
    setElevatorSetpoint(setpoint);
  }

  public void runElevator(double volts) {
    
  }

  public void runShooter(double speed) {
    shooter_speed = speed;
    setShooterSpeed(speed);
  }

  // Direct set voltage methods
  protected abstract void setLeftEleVoltage(double volts);
  protected abstract void setRightEleVoltage(double volts);
  protected abstract void setWristVoltage(double volts);
  // PID set methods
  protected abstract void setLeftElePID(double setpoint);
  protected abstract void setRightElePID(double setpoint);
  protected abstract void setWristPID(double setpoint);
  // Spark set methods
  protected abstract void setLeftClawPID(double speed);
  protected abstract void setRightClawPID(double speed);
  protected abstract void setShooterSpeed(double speed);

  @Override
  public void periodic() {

  } 

}
