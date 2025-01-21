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
  public static class Robot {
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

  public static class Swerve {
    public static final double WHEEL_DIAMETER = 4 * 0.0254 * Math.PI;
    public static final double GEAR_RATIO = 5.01;
    public static final double MAX_SPEED = 3;
    public static final double OFFSET = 23.75 * 0.0254;

    public static class DriveMotorConfigs {
      public static final double kP = 0.3;
      public static final double kI = 0;
      public static final double kD = 0;
      public static final double kS = 0;
      public static final double kV = 0.5; 
      public static final double kA = 0;
      public static final double kG = 0;
    }

    public static class SteerMotorConfigs {
      public static final double kP = 10;
      public static final double kI = 0;
      public static final double kD = 0.7;
      public static final double kS = 0; //doesnt work?
      public static final double kV = 0;
      public static final double kA = 0;
      public static final double kG = 0;
    }
  }

  public static class Auto {
    public static final double translation_kP = 1;
    public static final double translation_kI = 0d;
    public static final double translation_kD = 0d;
    
    public static final double rotation_kP = 1;
    public static final double rotation_kI = 0d;
    public static final double rotation_kD = 0d;
  }

}