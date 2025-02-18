// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Elevator.MotorConfigs;
import frc.robot.Constants.Port;

public class Elevator extends SubsystemBase {
  TalonFX left_motor, right_motor;
  SparkMax shooter_motor;
  /** The height which the elevator aims to, in meters. */
  public double setpoint = 0;
  public double kG = 0.2;

  public Elevator() {
    // right is negative to go up because CCW is positive
    // krishna had them flipped because the control board stick inputs are reversed
    left_motor = new TalonFX(20);
    right_motor = new TalonFX(Port.RIGHT_ELEVATOR_MOTOR);
    shooter_motor = new SparkMax(Port.SHOOTER_MOTOR, MotorType.kBrushless);

    TalonFXConfiguration configs = new TalonFXConfiguration();
    configs.Slot0 = new Slot0Configs()
        .withKP(MotorConfigs.kP)
        .withKI(MotorConfigs.kI)
        .withKD(MotorConfigs.kD)
        .withKS(MotorConfigs.kS)
        .withKV(MotorConfigs.kV)
        .withKA(MotorConfigs.kA)
        .withKG(MotorConfigs.kG);

    configs.Feedback.SensorToMechanismRatio = 25;
    configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    right_motor.getConfigurator().apply(configs);
    left_motor.getConfigurator().apply(configs);

    left_motor.setPosition(0);
    right_motor.setPosition(0);

    left_motor.setControl(new Follower(Port.RIGHT_ELEVATOR_MOTOR, true));

  }

  @Override
  public void periodic() {
    // // TODO: should this be in a setSetpoint function or is this better
    // left_motor.setControl(new PositionVoltage(-setpoint));
    // right_motor.setControl(new PositionVoltage(setpoint));
  }

  @Override
  public void simulationPeriodic() {
  
  }

  public void setShooter(double speed) {
    SmartDashboard.putNumber("Shooter speed", speed);
    shooter_motor.set(speed);
  }

  public void setVoltage(double volt) {
    SmartDashboard.putNumber("Elevator volt", volt);
    // left_motor.setControl(new VoltageOut(volt + kG));
    right_motor.setControl(new VoltageOut(-(volt + kG)));
  }

}
