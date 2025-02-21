// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ServoSystem extends SubsystemBase {
  public static ServoSystem instance;
  public Servo thing;
  private double angle;

  /** Creates a new ServoSystem. */
  public ServoSystem() {
    this.thing = new Servo(0);
    this.angle = 0;
  }

  public void setAngle(double angle){
    thing.setAngle(angle);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // thing.setAngle(angle);
    // System.out.println(thing.getAngle());
    angle++;
    
  }
}
