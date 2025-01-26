// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Vision;

/** Add your docs here. */
public class Target {
    public double distance;
    public double offset;
    public double direct_distance;

    public Target(double distance, double offset, double direct_distance){
        distance = this.distance;
        offset = this.offset;
        direct_distance = this.direct_distance;
    }

    public Target(){
        this.distance = 0;
        this.offset = 0;
        this.direct_distance = 0;
    }
}
