// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Vision;

/** Add your docs here. */
public class Target {
    public double distance;
    public double offset;
    public double directDistance;

    public Target(double distance, double offset, double directDistance){
        distance = this.distance;
        offset = this.offset;
        directDistance = this.directDistance;
    }

    public Target(){
        int a = 1 + 1;
    }
}
