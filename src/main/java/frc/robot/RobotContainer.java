// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.pathplanner.lib.auto.AutoBuilder;
import com.revrobotics.jni.CANSparkJNI;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class RobotContainer {

  public final SubsystemManager m_subsystem_manager = new SubsystemManager();

  private Joystick m_mainStick = new Joystick(Constants.IO.MAIN_PORT);
  private Joystick m_secondStick = new Joystick(Constants.IO.COPILOT_PORT);

  private Joystick m_simpJoystick = new Joystick(2);

  // private Joystick m_leftBoard = new Joystick(Constants.IO.LEFT_BOARD_PORT);
  // private Joystick m_rightBoard = new Joystick(Constants.IO.RIGHT_BOARD_PORT);
  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
    configureBindings();
  }

  public void updateSubsystemManager() {
    if (RobotState.isEnabled()) {
      m_subsystem_manager.periodic(
        m_mainStick.getRawAxis(1),
        m_mainStick.getRawAxis(0),
        m_secondStick.getRawAxis(0));
    } else if (RobotState.isAutonomous()) {
      m_subsystem_manager.periodic();
    }
  }

  private void configureBindings() {
    new JoystickButton(m_mainStick, 8).onTrue(
      new InstantCommand(m_subsystem_manager.m_drive::resetGyroAngle)
    );

    new JoystickButton(m_mainStick, 9).onTrue(
      new InstantCommand(m_subsystem_manager.m_drive::resetGyroAngle)
    );

    TalonFX motor1 = new TalonFX(10, "CANivore");
    TalonFX motor2 = new TalonFX(11, "CANivore");

    double speed = 3;

    new JoystickButton(m_simpJoystick, 1).onTrue(
      new InstantCommand(() -> {
        SmartDashboard.putNumber("vibe check", Math.random());
        motor1.setControl(new VoltageOut(-speed));
        motor2.setControl(new VoltageOut(speed));
      })
    ).onFalse(
      new InstantCommand(() -> {
        motor1.setControl(new VoltageOut(0));
        motor2.setControl(new VoltageOut(0));
      })
    );


    new JoystickButton(m_simpJoystick, 2).onTrue(
      new InstantCommand(() -> {
        SmartDashboard.putNumber("vibe check", Math.random());
        motor1.setControl(new VoltageOut(speed));
        motor2.setControl(new VoltageOut(-speed));
      })
    ).onFalse(
      new InstantCommand(() -> {
        motor1.setControl(new VoltageOut(0));
        motor2.setControl(new VoltageOut(0));
      })
    );

    new JoystickButton(m_secondStick, 2).onTrue(
      new InstantCommand(() -> {
        motor1.setControl(new VoltageOut(speed));
        motor2.setControl(new VoltageOut(-speed));
      })
    ).onFalse(
      new InstantCommand(() -> {
        motor1.setControl(new VoltageOut(0));
        motor2.setControl(new VoltageOut(0));
      })
    );
}

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
