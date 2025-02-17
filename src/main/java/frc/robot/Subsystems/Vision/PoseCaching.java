// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

import java.util.LinkedList;
import java.util.Queue;

public class PoseCaching {
    private final int capacity;
    private final Queue<Pose2d> poseCache;
    private final Queue<Double> timestampCache;
    
    
    public PoseCaching(int capacity) {
        this.capacity = capacity;
        this.poseCache = new LinkedList<>();
        this.timestampCache = new LinkedList<>();
    }

    public void addPose(Pose2d pose, double timestamp) {
        if(!timestampCache.contains(timestamp)){
            if (poseCache.size() >= capacity) {
                poseCache.poll();
                timestampCache.poll();
            }
            poseCache.add(pose);
            timestampCache.add(timestamp);
        }
    }

    // calculates and returns the standard deviations for x, y, and theta (in radians) from the cached Pose2d measurements.
    public Matrix<N3, N1> calculateStdDevs() {
        int n = poseCache.size();
        if (n == 0) {
            return VecBuilder.fill(0.0, 0.0, 0.0);
        }
        
        // means for x, y and theta 
        double sumX = 0.0;
        double sumY = 0.0;
        double sumSin = 0.0;
        double sumCos = 0.0;
        for (Pose2d pose : poseCache) {
            sumX += pose.getX();
            sumY += pose.getY();
            double theta = pose.getRotation().getRadians();
            sumSin += Math.sin(theta);
            sumCos += Math.cos(theta);
        }
        double meanX = sumX / n;
        double meanY = sumY / n;
        double meanTheta = Math.atan2(sumSin / n, sumCos / n);

        // variance for x, y, and theta
        double varX = 0.0;
        double varY = 0.0;
        double varTheta = 0.0;
        for (Pose2d pose : poseCache) {
            double dx = pose.getX() - meanX;
            double dy = pose.getY() - meanY;
            double dTheta = angleDifference(pose.getRotation().getRadians(), meanTheta);
            varX += dx * dx;
            varY += dy * dy;
            varTheta += dTheta * dTheta;
        }
        if (n > 1) {
            varX /= (n - 1);
            varY /= (n - 1);
            varTheta /= (n - 1);
        } else {
            varX = 0.0;
            varY = 0.0;
            varTheta = 0.0;
        }
        double stdX = Math.sqrt(varX);
        double stdY = Math.sqrt(varY);
        double stdTheta = Math.sqrt(varTheta);
        
        return VecBuilder.fill(stdX, stdY, stdTheta);
    }
    
    // calculate smallest difference between two angles
    private double angleDifference(double a, double b) {
        double diff = a - b;
        while(diff > Math.PI) {
            diff -= 2 * Math.PI;
        }
        while(diff < -Math.PI) {
            diff += 2 * Math.PI;
        }
        return diff;
    }
}
