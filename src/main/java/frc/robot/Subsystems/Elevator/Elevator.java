// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Elevator;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Elevator.MotorConfigs;
import frc.robot.Constants;
import frc.robot.Constants.Port;

public class Elevator extends SubsystemBase {
  TalonFX left_motor, right_motor;
  SparkMax shooter_motor;
  /** The height which the elevator aims to, in meters. */
  public double setpoint = 0;

  public Elevator() {
    // right is negative to go up, right is also 11
    left_motor = new TalonFX(Port.LEFT_ELEVATOR_MOTOR, "CANivore");
    right_motor = new TalonFX(Port.RIGHT_ELEVATOR_MOTOR, "CANivore");
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

    left_motor.getConfigurator().apply(configs);
    right_motor.getConfigurator().apply(configs);

    left_motor.setPosition(0);
    right_motor.setPosition(0);
  }

  @Override
  public void periodic() {
    // left_motor.setControl(new PositionVoltage(setpoint));
    // right_motor.setControl(new PositionVoltage(-setpoint));
    
    Joystick m_rightBoard = new Joystick(Constants.IO.RIGHT_BOARD_PORT);
    double direction = m_rightBoard.getRawAxis(0);
    SmartDashboard.putNumber("left_motor_encoder", left_motor.getPosition().getValueAsDouble());
    left_motor.setControl(new VoltageOut(-direction));
    right_motor.setControl(new VoltageOut(direction));
  }

  @Override
  public void simulationPeriodic() {
  
  }

  // public Command 

}
