// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {

    // Global constants
    public static final double DEFAULT_COMMAND_TIMEOUT_SECONDS = 5;

    public static final class AutoConstants {

        public static enum AutoPattern {
            DO_NOTHING,
            DRIVE_FORWARD,
            DRIVE_FORWARD_PID_TIMED,
            DRIVE_FORWARD_PID_MEASURED,
            TEST_ARM_COMMANDS,
            // the below autopatterns assume that were on blue i.e. when facing the speaker the amp
            // is on the left
            OUTSIDE_ONE_SHOT,
            OUTSIDE_TWO_SHOT,
            SPEAKER_THREE_SHOT,
            SPEAKER_FOUR_SHOT,
            AMP_ONE_SHOT,
            AMP_TWO_SHOT,
            AMP_ONE_SHOT_ONE_AMP,
            AMP_ONE_AMP,
            AMP_TWO_AMP
        }

        // amp side distance variables

        public static final double AmpSideDiagStepCM = 0;
        public static final double AmpSideDiagAngle  = 0;
        public static final double AmpSideStraightCM = 0;



    }

    public static final class DriveConstants {

        public static enum DriveMode {
            TANK, SINGLE_STICK_ARCADE, DUAL_STICK_ARCADE;
        }

        // motor stuff
        public static final int     LEFT_REAR_PORT         = 21;
        public static final int     LEFT_FRONT_PORT        = 22;
        public static final int     RIGHT_REAR_PORT        = 23;
        public static final int     RIGHT_FRONT_PORT       = 24;

        public static final boolean LEFT_MOTOR_REVERSED    = false;
        public static final boolean RIGHT_MOTOR_REVERSED   = true;

        public static final boolean LEFT_ENCODER_REVERSED  = false;
        public static final boolean RIGHT_ENCODER_REVERSED = true;

        // public static final double MAX_WHEEL_SPEED_MPS;

        // encoder stuff
        public static final int    ENCODER_COUNTS_PER_REVOLUTION = 1024;
        public static final double ROBOT_WHEEL_DIAMETER_M        = 6 * 2.54 / 100d;

        public static final double CMS_PER_ENCODER_COUNT         =
            // Assumes the encoders are directly mounted on the wheel shafts
            (ROBOT_WHEEL_DIAMETER_M * Math.PI) / ENCODER_COUNTS_PER_REVOLUTION / 100d;

        // PID gains
        public static final double HEADING_PID_KP                = 0.02;
        public static final double HEADING_PID_KI                = 0;
        public static final double HEADING_PID_KD                = 0;

        public static final double TURN_TO_HEADING_PID_KP        = 0.002;
        public static final double TURN_TO_HEADING_PID_KI        = 0.1;
        public static final double TURN_TO_HEADING_PID_KD        = 0;

        public static final double HEADING_ERROR_BUFFER          = 3;

        public static enum HeadingStates {
            FAR, CLOSE
        }

    }

    public static final class LightConstants {

        public static final int LED_STRIP_PWM_PORT     = 0;
        public static final int LED_STRIP_LENGTH       = 60;
        public static final int LED_STICK_TAKEN_LENGTH = 5;
        public static final int BOOST_INDEX            = 0;


    }

    public static final class ArmConstants {

        public static final int    PIVOT_PORT                           = 20;
        public static final int    KEEPER_PORT                          = 2;
        public static final int    INTAKE_LOWER_PORT                    = 1;
        public static final int    INTAKE_HIGHER_PORT                   = 0;

        // climber motor left = 3;
        // climber motor right = 4;

        public static final double PIVOT_ARM_ENCODER_COUNT_PER_ROTATION = 37.2;

        public static final double MIN_ARM_PIVOT_ANGLE                  = -26.5;
        public static final double MAX_ARM_PIVOT_ANGLE                  = 130.0;

        public static final int    EQUILIBRIUM_ARM_ANGLE                = 61;
        public static final int    EQUILIBRIUM_ARM_ANGLE_BUFFER         = 5;

        public static final double PIVOT_DEFAULT_SPEED                  = 0.1;

        public static enum AngleStates {
            FAR, CLOSE
        }

        public static enum PivotShootStates {
            PIVOTING,
            SHOOTING
        }

        public static enum IntakeStates {
            PIVOTING,
            INTAKING
        }

        public static final double PIVOT_TO_ANGLE_PID_KP = 0;
        public static final double PIVOT_TO_ANGLE_PID_KI = 0;
        public static final double PIVOT_TO_ANGLE_PID_KD = 0;



    }

    public static final class OperatorConstants {

        public static final int    DRIVER_CONTROLLER_PORT         = 0;
        public static final double GAME_CONTROLLER_STICK_DEADBAND = 3;

    }

    public static final class CameraConstants {

        public static final int CAMERA_PORT      = 0;
        public static final int MJPEGSERVER_PORT = 0;

    }

}