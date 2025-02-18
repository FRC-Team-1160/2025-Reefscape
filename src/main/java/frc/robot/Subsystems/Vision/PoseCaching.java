// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

import java.util.ArrayDeque;
import java.util.LinkedList;
import java.util.Queue;

public class PoseCaching {
    
    private double pose_timeout;

    private final ArrayDeque<CachedPose> cache;

    private record CachedPose(double x, double y, double a, double timestamp) {}

    private double[] sums, sum_squares;

    public PoseCaching(double pose_timeout) {
        pose_timeout = pose_timeout;
        cache = new ArrayDeque<CachedPose>();
        sums = new double[3];
        sum_squares = new double[3];
    }

    public void addPose(Pose2d pose, Pose2d referencePose, double timestampSeconds) {
        if (cache.isEmpty() || Math.abs(timestampSeconds - cache.getLast().timestamp) < 1e-4) {
            Transform2d diff = pose.minus(referencePose);

            sums[0] += diff.getX();
            sums[1] += diff.getY();
            sums[2] += diff.getRotation().getRadians();

            sum_squares[0] += Math.pow(diff.getX(), 2);
            sum_squares[1] += Math.pow(diff.getY(), 2);
            sum_squares[2] += Math.pow(diff.getRotation().getRadians(), 2);

            cache.add(new CachedPose(
                diff.getX(), 
                diff.getY(), 
                diff.getRotation().getRadians(), 
                timestampSeconds));

        }
        clearOldPoses(timestampSeconds);
    }

    public void clearOldPoses(double timestampSeconds) {
        while (!cache.isEmpty()) {
            CachedPose first = cache.getFirst();
            if (timestampSeconds - first.timestamp > pose_timeout) {
                sums[0] -= first.x;
                sums[1] -= first.y;
                sums[2] -= first.a;

                sum_squares[0] -= first.x * first.x;
                sum_squares[1] -= first.y * first.y;
                sum_squares[2] -= first.a * first.a;
                
                cache.removeFirst();
            } else {
                break;
            }
        }
    }

    public double calcStdev(int i) {
        return Math.sqrt(sum_squares[i] - (sums[i] * sums[i] / cache.size()) / cache.size());
    }

    public Matrix<N3, N1> getStdevs() {
        return new Matrix<>(Nat.N3(), Nat.N1(), new double[] {
                calcStdev(0),
                calcStdev(1),
                calcStdev(2)
            });
    }
}
