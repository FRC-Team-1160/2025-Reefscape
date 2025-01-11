// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static class FieldConstants {
    public static final double SPEAKER_Y = 0.0;
    public static final double SPEAKER_X = 0.0;
  }

  public static class RobotConstants {
    public static final double LOOP_TIME_SECONDS = 0.02;
  }

  
  public static final class Port {
    // CAN IDs 
    public static final int FRONT_LEFT_STEER_MOTOR = 1;
    public static final int FRONT_RIGHT_STEER_MOTOR = 3;
    public static final int BACK_LEFT_STEER_MOTOR = 5;
    public static final int BACK_RIGHT_STEER_MOTOR = 7;

    public static final int FRONT_LEFT_DRIVE_MOTOR = 2;
    public static final int FRONT_RIGHT_DRIVE_MOTOR = 4;
    public static final int BACK_LEFT_DRIVE_MOTOR = 6;
    public static final int BACK_RIGHT_DRIVE_MOTOR = 8;

    public static final int FRONT_LEFT_CODER = 1;
    public static final int FRONT_RIGHT_CODER = 3;
    public static final int BACK_LEFT_CODER = 5;
    public static final int BACK_RIGHT_CODER = 7;

    public static final int SHOOTER_TOP_MOTOR = 10;
    public static final int SHOOTER_BOTTOM_MOTOR = 9;
    public static final int SHOOTER_PITCH_MOTOR = 7;

    public static final int INTAKE_MOTOR = 4;

    public static final int TRANSPORT_LEFT_MOTOR = 10;
    public static final int TRANSPORT_RIGHT_MOTOR = 6;
    public static final int TRANSPORT_BELT_MOTOR = 5;
    
    public static final int TRANSPORT_ULTRASONIC = 0;
    
    public static final int CLIMBER_LEFT_MOTOR = 8;
    public static final int CLIMBER_RIGHT_MOTOR = 3;
    
    public static final int LEFT_CLIMB_LIMIT = 0;
    public static final int RIGHT_CLIMB_LIMIT = 1;
    public static final int TRANSPORT_LIMIT = 2;

  }

  public static class IO {
    public static final int MAIN_PORT = 0;
    public static final int COPILOT_PORT = 1;
    public static final int COPILOT_SIMP_PORT = 2;
    public static final int LEFT_BOARD_PORT = 3;
    public static final int RIGHT_BOARD_PORT = 4;

    public static final class Board {
      public static final class Left {
        public static final int SHOOT = 1;
        public static final int AIM = 2;

        public static final int AMP = 3;

        public static final int SHOOT_OVERRIDE = 5;
        public static final int REV = 6;

        public static final int LEFT_CLIMB = 0;
      }
      public static final class Right {
        public static final int UP_DOWN_INTAKE = 1;
        public static final int OVERRIDE = 4;
        public static final int OUTTAKE = 9;
        public static final int INTAKE = 8;

        public static final int INC_OR_DEC_TAR = 3;
        public static final int MOVE_TAR = 6;

        public static final int RIGHT_CLIMB = 0;
      }
    }
  }

  public static class ClimberConstants {
    public static final double MOTOR_SPEED_VOLTS = 8.0;
  }

  public static class IntakeConstants {
    public static final double INTAKE_VOLTS = 8.0;
    public static final double OUTTAKE_VOLTS = -6.0;
  }

  public static class TransportConstants {
    public static final double PROX_THRESHOLD = 200.0;
    public static final double BELT_SPEED_VOLTS = 6.0;
    public static final double WHEEL_SPEED_VOLTS = 2.0;
  }

  public static class SwerveConstants {
    public static final double WHEEL_DIAMETER = 4 * 0.0254 * Math.PI;
    public static final double GEAR_RATIO = 6.75;
    public static final double MAX_SPEED = 4.5;

    public static final double DRIVE_ROTOR_METERS = WHEEL_DIAMETER / GEAR_RATIO;

    public static class DriveMotorConfigs {
      public static final double kP = 0.05; 
      public static final double kI = 0;
      public static final double kD = 0;
      public static final double kS = 0.10;
      public static final double kV = 0.12; 
      public static final double kA = 0;
      public static final double kG = 0;
    }

    public static class SteerMotorConfigs {
      public static final double kP = 0.05;
      public static final double kI = 0.01;
      public static final double kD = 0.005;
      public static final double kS = 0;
      public static final double kV = 0;
      public static final double kA = 0;
      public static final double kG = 0;
    }
  }

}
