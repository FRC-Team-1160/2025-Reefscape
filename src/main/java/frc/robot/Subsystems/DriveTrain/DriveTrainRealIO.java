// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.DriveTrain;

import com.studica.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.TransportConstants;

public class DriveTrainRealIO extends DriveTrain {

  private AHRS m_gyro;

  public DriveTrainRealIO(){

    m_gyro = new AHRS(AHRS.NavXComType.kI2C);
    m_gyro.zeroYaw();
  
  }

  public SwerveModule initializeModule(int drive_port, int steer_port, int sensor_port){
    return new SwerveModuleRealIO(drive_port, steer_port, sensor_port);
  }

  public Rotation2d getGyroAngle(){
    if (m_gyro != null){
      return Rotation2d.fromDegrees(-m_gyro.getAngle()); //gyro reports CW positive, negate to return CCW positive
    } else {
      return new Rotation2d();
    }
  }

  @Override
  public void periodic() {
    super.periodic();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
