// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Elevator;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


abstract public class Elevator extends SubsystemBase {

  /** The height which the elevator travels toward, in meters. 0 is the bottom of the range of motion. */
  public double setpoint;

  public double shooter_speed;

  public Mechanism2d elevator_mech;

  public Elevator() {
    elevator_mech = new Mechanism2d(0, 0);
    var root = elevator_mech.getRoot("r_root", 0.1, 0.1);
    var lig = root.append(new MechanismLigament2d("ele_lig", 1, 90));
    var holder = lig.append(new MechanismLigament2d("holder", 0.2, -135));
    
  }

  public void setSetpoint(double setpoint) {
    this.setpoint = setpoint;
  }

  public void changeSetpoint(double setpoint) {
    this.setpoint += setpoint;
  }

  public void runShooter(double speed) {
    shooter_speed = speed;
    setShooterSpeed(speed);
  }

  protected abstract void setPIDControl();
  protected abstract void setShooterSpeed(double speed);

  @Override
  public void periodic() {
    SmartDashboard.putData("Elevator Mechanism", elevator_mech);

  } 

}
