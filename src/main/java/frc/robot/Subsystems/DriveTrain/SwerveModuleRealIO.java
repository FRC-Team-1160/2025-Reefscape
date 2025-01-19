// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.DriveTrain;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.SwerveConstants.DriveMotorConfigs;
import frc.robot.Constants.SwerveConstants.SteerMotorConfigs;


public class SwerveModuleRealIO extends SwerveModule{

  public TalonFX steer_motor, drive_motor;

  public CANcoder steer_sensor;

  public SwerveModuleRealIO(int drive_port, int steer_port, int sensor_port){
    drive_motor = new TalonFX(drive_port);
    steer_motor = new TalonFX(steer_port);
    steer_sensor = new CANcoder(sensor_port);
    
    TalonFXConfiguration driveConfigs = new TalonFXConfiguration();

    driveConfigs.Slot0 = new Slot0Configs()
      .withKP(DriveMotorConfigs.kP)
      .withKI(DriveMotorConfigs.kI)
      .withKD(DriveMotorConfigs.kD)
      .withKS(DriveMotorConfigs.kS)
      .withKV(DriveMotorConfigs.kV)
      .withKA(DriveMotorConfigs.kA)
      .withKG(DriveMotorConfigs.kG);

    driveConfigs.Feedback.SensorToMechanismRatio = 5.01;

    drive_motor.getConfigurator().apply(driveConfigs);

    TalonFXConfiguration steerConfigs = new TalonFXConfiguration();

    steerConfigs.Slot0 = new Slot0Configs()
      .withKP(SteerMotorConfigs.kP)
      .withKI(SteerMotorConfigs.kI)
      .withKD(SteerMotorConfigs.kD)
      .withKS(SteerMotorConfigs.kS)
      .withKV(SteerMotorConfigs.kV)
      .withKA(SteerMotorConfigs.kA)
      .withKG(SteerMotorConfigs.kG);

    steerConfigs.Feedback.FeedbackRemoteSensorID = sensor_port;
    steerConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    steerConfigs.Feedback.SensorToMechanismRatio = -1; //motors reversed?

    steerConfigs.Voltage.PeakForwardVoltage = 2;
    steerConfigs.Voltage.PeakReverseVoltage = -2;

    steerConfigs.ClosedLoopGeneral.ContinuousWrap = true;


    steer_motor.getConfigurator().apply(steerConfigs);

  }

  public double getSpeed(){
    //getRotorVelocity() returns StatusSignal<AngularVelocity> with base unit rps
    return drive_motor.getRotorVelocity().getValueAsDouble() * SwerveConstants.WHEEL_DIAMETER;
  }

  public double getPosition(){
    //getRotorPosition() returns StatusSignal<Angle> with base unit rotations
    return drive_motor.getRotorPosition().getValueAsDouble() * SwerveConstants.WHEEL_DIAMETER;
  }

  public Rotation2d getAngle(){
    //getPosition() returns StatusSignal<Angle> with base unit rotations
    double a = steer_sensor.getAbsolutePosition().getValueAsDouble();
    a = MathUtil.inputModulus(a, -0.5, 0.5);
    SmartDashboard.putNumber("out_angle", a);
    return Rotation2d.fromRotations(a);
  }

  public SwerveModuleState getModuleState(){
    return new SwerveModuleState(getSpeed(), getAngle());
  }

  public SwerveModulePosition getModulePosition(){
    SmartDashboard.putNumber("rotor_position", getPosition());
    return new SwerveModulePosition(getPosition(), getAngle());
  }

  public void setSpeed(double speedMetersPerSecond){
    SmartDashboard.putNumber("in_speed", speedMetersPerSecond / SwerveConstants.WHEEL_DIAMETER);
    drive_motor.setControl(new VelocityVoltage(speedMetersPerSecond / SwerveConstants.WHEEL_DIAMETER));

  }

  public void setAngle(Rotation2d angle){
    SmartDashboard.putNumber("in_angle", angle.getRotations());
    steer_motor.setControl(new PositionVoltage(-angle.getRotations())); //account for motor reversal?
  }

}
