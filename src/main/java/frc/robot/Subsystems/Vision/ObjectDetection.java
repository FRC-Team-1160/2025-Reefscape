// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Vision;

import java.util.ArrayList;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.networktables.StructSubscriber;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ObjectDetection extends SubsystemBase {
  private PhotonCamera detector;
  public boolean hasTarget;
  public Pose3d closestPose;
  private ArrayList<Pose3d> targetPoses = new ArrayList<>();
  private ArrayList<Target> targetDistances = new ArrayList<>();

  private Pose2d robotPose;


  StructPublisher<Pose3d> adv_closestPub;
  StructArrayPublisher<Pose3d> adv_targetsPub;
  StructSubscriber<Pose2d> adv_robotPoseSub;

  final int horizontalScreenPixel = 640;
  final static double cameraHFOV = Math.toRadians(70);
  final static double targetWidthM = 0.413;
  
  int minX;
  int maxX;
  int minY;
  int maxY;
  int midpoint;
  double closestDistance;

  boolean sim = false;

  /** Creates a new ObjectDetection. */
  public ObjectDetection() {
    if (!sim){
      detector = new PhotonCamera("OV9782");
    }
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable adv_vision = inst.getTable("adv_vision");
    NetworkTable adv_swerve = inst.getTable("adv_swerve");
    adv_targetsPub = adv_vision.getStructArrayTopic("Target", Pose3d.struct).publish();
    adv_closestPub = adv_vision.getStructTopic("Closest", Pose3d.struct).publish();

    adv_robotPoseSub = adv_swerve.getStructTopic("Pose", Pose2d.struct).subscribe(new Pose2d(), PubSubOption.keepDuplicates(true));
  }

  public Pose3d getObjectPose3D(Pose2d robotPose, double distance, double angleToTarget) {
    // Robot's global heading
    double robotTheta = robotPose.getRotation().getRadians();

    // Convert distance + angle to object coords in the field frame:
    double globalAngle = robotTheta + angleToTarget; // field heading to the target
    double x = robotPose.getX() + (distance * Math.cos(globalAngle));
    double y = robotPose.getY() + (distance * Math.sin(globalAngle));

    // If you don't know the object's actual heading, just store globalAngle or 0.
    // We'll just store the object's "facing" as globalAngle here for demonstration.
    return new Pose3d(x, y, 0.21, new Rotation3d(0, 0, globalAngle));
  }

  // public static double[] getDistance(double targetWidthPixel, double targetXPixel){
  //   double c = targetWidthPixel / (2 * Math.tanh(cameraHFOV));
  //   double distance = targetWidthM / targetWidthPixel * c;

  //   double offset = (targetXPixel - targetWidthPixel/2) * c * distance;
  //   double[] result = {distance, offset};
  //   return result;
  // }

  private double getFocalLength(int imageWidthPx, double hfovRadians) {
    return (imageWidthPx / 2.0) / Math.tan(hfovRadians / 2.0);
  }

  public double[] getDistance(double targetWidthPixel, double targetXPixel) {
    // 1) Distance: d = (focalLengthPx * realWidthMeters) / boundingBoxWidthPx
    double distance;
    double focalLengthPx = getFocalLength(horizontalScreenPixel, cameraHFOV);
    if (targetWidthPixel > 0) {
        distance = 250 / targetWidthPixel; //(focalLengthPx * targetWidthM) / targetWidthPixel;
        SmartDashboard.putNumber("Vison Distance", distance);

    } else {
        distance = Double.POSITIVE_INFINITY;
    }

    // 2) Horizontal Offset:
    //    - First compute angle from camera centerline:
    //      fractionOffCenter = (targetXPixel - imageCenter) / (imageCenter)
    //      offsetAngleRad    = fractionOffCenter * (CAMERA_HFOV_RAD / 2)
    //    - Then horizontal = distance * sin(offsetAngleRad)
    double imageCenter = horizontalScreenPixel / 2.0;
    double fractionOffCenter = (targetXPixel - imageCenter) / imageCenter;
    double offsetAngleRad = fractionOffCenter * (cameraHFOV / 2.0);
    double horizontalOffset = distance * Math.sin(offsetAngleRad);

    return new double[] { distance, horizontalOffset };
  }


  public Pose3d getClosestTarget(ArrayList<Target> targets){
    double minDis = 100000.0;
    Target result1 = new Target();
    for (Target target1 : targets){
      if(target1.directDistance < minDis){
        minDis = target1.directDistance;
        result1 = target1;
      }
    }
    double angleToTarget = Math.atan(result1.offset/result1.distance);
    return getObjectPose3D(robotPose, result1.distance, angleToTarget);
  }


  @Override
  public void periodic() {

    // This method will be called once per scheduler run
    // closestDistance = getDistance(413.0, 2.4, 413.0);
    targetDistances.clear();
    targetPoses.clear();
    if (!sim){
      robotPose = adv_robotPoseSub.get();
      hasTarget = detector.getLatestResult().hasTargets();
      SmartDashboard.putBoolean("hasTarget", hasTarget);
    }

    if (robotPose != null && !sim){
      var result = detector.getLatestResult();
      List<PhotonTrackedTarget> targets = result.getTargets();
      for (int i = 0; i < targets.size(); i++) {
        minX = (int) targets.get(i).getMinAreaRectCorners().get(0).x;
        maxX = (int) targets.get(i).getMinAreaRectCorners().get(0).x;
        minY = (int) targets.get(i).getMinAreaRectCorners().get(0).y;
        maxY = (int) targets.get(i).getMinAreaRectCorners().get(0).y;
        for (TargetCorner corner : targets.get(i).getMinAreaRectCorners()) {
            if (corner.x < minX) {
                minX = (int) corner.x;
            }
            if (corner.x > maxX) {
                maxX = (int) corner.x;
            }
            if (corner.y < minY) {
                minY = (int) corner.y;
            }
            if (corner.y > maxY) {
                maxY = (int) corner.y;
            }
        }
        int tempMidpoint = (maxY + minY) / 2;
        double tempWidthPixel = maxX - minX;
        double tempHeightPixel = maxY - minY;

        SmartDashboard.putNumber("minY", minY);
        SmartDashboard.putNumber("maxY", maxY);
        SmartDashboard.putNumber("tempHeight", tempHeightPixel);


        double[] tempDistance = getDistance(tempHeightPixel, tempMidpoint);
        double distance = tempDistance[0];// + 0.55;
        double offset = - tempDistance[1];// + 0.24;
        // double distance = 230 / tempWidthPixel;
        // double offset = -(tempMidpoint - (horizontalScreenPixel/2))*0.012 - 0.28;
        double angleToTarget = Math.atan(offset/distance);
        double directDistance = Math.sqrt(Math.pow(distance, 2) + Math.pow(offset, 2));
        SmartDashboard.putNumber("tempHeight2", tempHeightPixel);
        System.out.println(tempHeightPixel);
        
        targetPoses.add(getObjectPose3D(robotPose, distance, angleToTarget));
        targetDistances.add(new Target(distance, offset, directDistance));
      }

      if (targetPoses.size() == 1){
        // adv_targetsPub.set(targetPoses); 
        closestPose = targetPoses.get(0);
        adv_closestPub.set(targetPoses.get(0));
      }else if (targetPoses.size() > 1){
        closestPose = getClosestTarget(targetDistances);
      }
    }

    if (sim == true){
      closestPose = new Pose3d(1, 7, 0, new Rotation3d(0.0, 0.0, Math.PI));
      adv_closestPub.set(closestPose);
    }
  
  }
}
