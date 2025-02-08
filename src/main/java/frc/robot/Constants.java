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

    public static final int LEFT_ELEVATOR_MOTOR = 10;
    public static final int RIGHT_ELEVATOR_MOTOR = 11;

    public static final int SHOOTER_MOTOR = 12;

    // NOTE: shooter angle motor number TBD
    public static final int SHOOTER_ANGLE_MOTOR = 13;
  }

  public static class IO {
    public static final int MAIN_PORT = 0;
    public static final int COPILOT_PORT = 1;
    public static final int LEFT_BOARD_PORT = 2;
    public static final int RIGHT_BOARD_PORT = 3;

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

  public static class Vision {
    /**in pixels */
    public static final int SCREEN_WIDTH = 640;
    /**in pixels */
    public static final int SCREEN_HEIGHT = 480;
    /**in radians */
    public final static double CAMERA_X_FOV = Math.toRadians(70);
    /**width of ball in meters */
    public final static double TARGET_WIDTH = 0.413;

    public static class CameraIntrinsics {
      public static final double f_x = 552.4060255333238;
      public static final double c_x = 353.4277563553747;
      public static final double f_y = 551.742860296023;
      public static final double c_y = 211.14820439067003;
    }

    public static class CameraDistortion {
      public static final double k1 = 0.01909844064431134;
      public static final double k2 = -0.06833877284448357;
      public static final double k3 = 0.004054963702231638;
      public static final double k4 = 0.00043824638804727323;
      public static final double k5 = 0.14324891388740463;
      public static final double k6 = -0.002369416441714424;
      public static final double k7 = 0.002236617101813135;
      public static final double k8 = -0.006014121063178925;
    }
  }

  public static class Swerve {
    public static final double WHEEL_DIAMETER = 4 * 0.0254 * Math.PI;
    public static final double GEAR_RATIO = 5.01;
    public static final double OFFSET = 23.75 * 0.0254;

    public static final double MAX_SPEED = 3;
    public static final double DRIVE_SPEED = 0.5;
    public static final double TURN_SPEED = 0.5;

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

  public static class Elevator {

    /**Max speed of elevator in meters per second. */
    public static final double MAX_SPEED = 0.05;

    public static class MotorConfigs {
      public static final double kP = 0;
      public static final double kI = 0;
      public static final double kD = 0;
      public static final double kS = 0;
      public static final double kV = 0;
      public static final double kA = 0;
      public static final double kG = 0;
    }
  }

  public static class Auto {
    public static final double translation_kP = 0d;
    public static final double translation_kI = 0d;
    public static final double translation_kD = 0d;
    
    public static final double rotation_kP = 0d;
    public static final double rotation_kI = 0d;
    public static final double rotation_kD = 0d;
  }

}