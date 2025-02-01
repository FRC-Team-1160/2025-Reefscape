// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Claw;

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

public class Claw extends SubsystemBase {
  TalonFX wrist_motor;
  /** The height which the elevator aims to, in meters. */
  public double setpoint = 0;

  public Claw() {
    // right is negative to go up, right is also 11
    wrist_motor = new TalonFX(Constants.Port.WRIST_MOTOR);

    TalonFXConfiguration configs = new TalonFXConfiguration();
    // configs.Slot0 = new Slot0Configs()
    //     .withKP(MotorConfigs.kP)
    //     .withKI(MotorConfigs.kI)
    //     .withKD(MotorConfigs.kD)
    //     .withKS(MotorConfigs.kS)
    //     .withKV(MotorConfigs.kV)
    //     .withKA(MotorConfigs.kA)
    //     .withKG(MotorConfigs.kG);

    configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    wrist_motor.getConfigurator().apply(configs);

  }

  @Override
  public void periodic() {
    // left_motor.setControl(new PositionVoltage(setpoint));
    // right_motor.setControl(new PositionVoltage(-setpoint));
    
    Joystick m_leftBoard = new Joystick(Constants.IO.LEFT_BOARD_PORT);
    double direction = m_leftBoard.getRawAxis(0);
    wrist_motor.setControl(new VoltageOut(direction * 1));
  }

  @Override
  public void simulationPeriodic() {
  
  }

  // public Command 

}
