// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {


    public static class RobotConstants {
        public static final double LOOP_TIME_SECONDS = 0.02;

        public static final double BASE_WIDTH = Units.inchesToMeters(29.5);
    }

    public static final class PortConstants {
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

        public static final int ELEVATOR_MOTOR_LEFT = 9;
        public static final int ELEVATOR_MOTOR_RIGHT = 10;

        public static final int WRIST_MOTOR = 11;

        public static final int CLIMBER_MOTOR_LEFT = 12;
        public static final int CLIMBER_MOTOR_RIGHT = 13;

        // Krakens get CAN ID priority because theyre simply better
        public static final int SHOOTER_MOTOR = 14;

        public static final int CLAW_MOTOR_LEFT = 15;
        public static final int CLAW_MOTOR_RIGHT = 16;
    }

    public static class IOConstants {
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

    public static class VisionConstants {
        /**Width of the camera stream in pixels.*/
        public static final int SCREEN_WIDTH = 640;
        /**Height of the camera stream in pixels. */
        public static final int SCREEN_HEIGHT = 480;
        /**Horizontal FOV of the camera in radians. */
        public final static double CAMERA_X_FOV = Units.degreesToRadians(70);
        /**Range in which tracked targets are marked for deletion if not seen.*/
        public final static double EXPECTED_RANGE = Units.degreesToRadians(40);
        /**Algae width in meters. */
        public final static double TARGET_WIDTH = Units.inchesToMeters(16);
        /**Range in pixels for which bounding boxes are considered cut off*/
        public final static int EDGE_TOLERANCE = 5;
        /**The maximum range for following targets */
        public final static double MAX_TRACKING_DISTANCE = 4.0;
        /**Time until targets are no longer tracked after not being seen */
        public final static double TRACKING_TIMEOUT = 15.0;
        /**Number of undetected frames before target is no longer tracked. */
        public final static int DETECTION_LIMIT = 3;
        
        public static class CameraTransforms {
            public static class LeftCamera {
                public static final double X = Units.inchesToMeters(2.5);
                public static final double Y = Units.inchesToMeters(9.0);
                public static final double Z = Units.inchesToMeters(11.0);

                public static final double ROLL = 0;
                public static final double PITCH = 0;
                public static final double YAW = Units.degreesToRadians(-20);
            }

            public static class RightCamera {
                public static final double X = Units.inchesToMeters(2.5);
                public static final double Y = Units.inchesToMeters(-9.0);
                public static final double Z = Units.inchesToMeters(11.0);

                public static final double ROLL = 0;
                public static final double PITCH = 0;
                public static final double YAW = Units.degreesToRadians(20);
            }

            public static class BackCamera {
                public static final double X = 0; //TODO: find constants
                public static final double Y = 0;
                public static final double Z = 0;

                public static final double ROLL = 0;
                public static final double PITCH = 0;
                public static final double YAW = 0;
            }
        }

        public static class EstimationParameters {
            public static final double a = 261.3;
            public static final double b = -0.044;
        }

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

    public static class SwerveConstants {
        public static final double WHEEL_DIAMETER = Units.inchesToMeters(4) * Math.PI;
        public static final double GEAR_RATIO = 5.01;
        public static final double OFFSET = Units.inchesToMeters(23.75);

        public static final double MAX_SPEED = 3;
        public static final double MAX_STEER_SPEED = 2;

        public static final double DRIVE_SPEED = (RobotBase.isReal()) ? 1.0 : 3.0;
        public static final double TURN_SPEED = 1.0;

        public static class Tracking {
            public static final double MAX_SPEED = 2.0;
            public static final double MAX_ACCEL = 3.0;
            public static final double MAX_ANG_SPEED = 4.0;
            public static final double MAX_ANG_ACCEL = 6.0;

            public static final double DEFAULT_DISTANCE = 0.5;

            public static final double MAX_ALIGN_SEPARATION = 2.0;
            public static final double ALIGN_SEPARATION_TOLERANCE = 0.1;

            public static final double ANGLE_TOLERANCE = Units.degreesToRadians(5);
            public static final double DISTANCE_TOLERANCE = 0.1;

            public static class PIDConstants {
                public static class Distance {
                    public static final double kP = 2.0;
                    public static final double kI = 0;
                    public static final double kD = 0;
                }
                public static class Angle {
                    public static final double kP = 2.0;
                    public static final double kI = 0;
                    public static final double kD = 0;
                }
            }
        }

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

    public static class ElevatorConstants {

        /**Max speed of elevator in meters per second. */
        public static final double MAX_SPEED = 0.05;

        public static final double MAX_EXTENSION = 1;

        public static class ElevatorConfigs {
            public static final double kP = 0;
            public static final double kI = 0;
            public static final double kD = 0;
            public static final double kS = 0;
            public static final double kV = 0;
            public static final double kA = 0;
            public static final double kG = 0.2;
        }

        public static class WristConfigs {
            public static final double kP = 0;
            public static final double kI = 0;
            public static final double kD = 0;
            public static final double kS = 0;
            public static final double kV = 0;
            public static final double kA = 0;
            public static final double kG = 0;
        }
    }

    public static class AutoConstants {
        public static final double translation_kP = 0d;
        public static final double translation_kI = 0d;
        public static final double translation_kD = 0d;
        
        public static final double rotation_kP = 0d;
        public static final double rotation_kI = 0d;
        public static final double rotation_kD = 0d;
    }

    public static class FieldConstants {

        public static final double WIDTH = 8.05;
        public static final double LENGTH = 17.55;

        public static class Reef {
            public static final double INNER_RADIUS = 1.66 / 2;
            public static final double CENTER_X = Units.feetToMeters(12) + INNER_RADIUS;
            public static final double CENTER_Y = FieldConstants.WIDTH / 2;
            public static final int NUM_SIDES = 6;
        }
    }

    public static class PathConstants {

        public static final double MAX_SPEED = 2.0;
        public static final double MAX_ACCEL = 3.0;
        public static final double MAX_ANG_SPEED = 4.0;
        public static final double MAX_ANG_ACCEL = 6.0;

        public static class ReefPaths {
            public static final double START_DISTANCE = 0.7;
            public static final double END_DISTANCE = 0.2;

            public static final double H_OFFSET = 0;
            
            public static final double IDEAL_START_SPEED = 1.2;
            public static final double MAX_ACCEL = 2;
        }
    }

}