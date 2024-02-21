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
            DRIVE_TO_COORDINATE_PID_MEASURED
        };
    }

    public static final class DriveConstants {

        public static enum DriveMode {
            TANK, SINGLE_STICK_ARCADE, DUAL_STICK_ARCADE;
        }

        // motor stuff
        public static final int     LEFT_MOTOR_PORT               = 10;
        public static final int     RIGHT_MOTOR_PORT              = 20;

        public static final boolean LEFT_MOTOR_REVERSED           = false;
        public static final boolean RIGHT_MOTOR_REVERSED          = true;

        public static final boolean LEFT_ENCODER_REVERSED         = false;
        public static final boolean RIGHT_ENCODER_REVERSED        = true;

        public static final double  MAX_WHEEL_SPEED_MPS;

        // encoder stuff
        public static final int     ENCODER_COUNTS_PER_REVOLUTION = 1024;
        public static final double  ROBOT_WHEEL_DIAMETER_M        = 6 * 2.54 / 100d;

        public static final double  CMS_PER_ENCODER_COUNT         =
            // Assumes the encoders are directly mounted on the wheel shafts
            (ROBOT_WHEEL_DIAMETER_CMS * Math.PI) / ENCODER_COUNTS_PER_REVOLUTION;

        // PID gains
        public static final double  HEADING_PID_KP                = 0.02;
        public static final double  HEADING_PID_KI                = 0;
        public static final double  HEADING_PID_KD                = 0;

        public static final double  TURN_TO_HEADING_PID_KP        = 0.002;
        public static final double  TURN_TO_HEADING_PID_KI        = 0.1;
        public static final double  TURN_TO_HEADING_PID_KD        = 0;

        public static final double  HEADING_ERROR_BUFFER          = 3;

        // Physical parms
        public static final double  WIDTH_WHEEL_TO_WHEEL;

        // Characterization gains
        public static final double  KS_VOLTS;
        public static final double  KV_VOLT_SECONDS_PER_METER;
        public static final double  KA_VOLT_SECONDS_SQUARED_PER_METER;

        public static final double  KP_DRIVE_VELO;

        public static final double  MAX_ROBOT_VELO;
        public static final double  MAX_ROBOT_ACCEL;

        public static final double  K_RAMSETE_B;
        public static final double  K_RAMSETE_ZETA;



    }

    public static final class lightConstants {

        public static final int LED_STRIP_PWM_PORT     = 0;
        public static final int LED_STRIP_LENGTH       = 60;
        public static final int LED_STICK_TAKEN_LENGTH = 5;


    }

    public static final class OperatorConstants {

        public static final int    DRIVER_CONTROLLER_PORT         = 0;
        public static final double GAME_CONTROLLER_STICK_DEADBAND = 3;
    }

}
