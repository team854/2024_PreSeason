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
            // is on the right
            AMP_ONE_SHOT,
            AMP_TWO_SHOT,
            AMP_THREE_SHOT,
            SPEAKER_ONE_SHOT,
            SPEAKER_TWO_SHOT,
            SPEAKER_THREE_SHOT,
            SPEAKER_FOUR_SHOT,
            OUTSIDE_ONE_SHOT,
            OUTSIDE_TWO_SHOT,
            OUTSIDE_THREE_SHOT,
        }

        // amp side distance variables

        public static final double AmpSideTimeoutMS            = 0;

        public static final double AmpSideDiagStepCM           = 0;
        public static final double AmpSideFirstStraightCM      = 0;
        public static final double AmpSideSecondStraightCM     = 0;
        public static final double AmpSideThirdStraightCM      = 0;

        public static final double AmpSideFirstAngle           = 0;
        public static final double AmpSideSecondAngle          = 0;
        public static final double AmpSideShootAngle           = 0;

        public static final double AmpSideShootSpeed           = 0;
        public static final double AmpSidePivotSpeed           = 0;
        public static final double AmpSideDriveSpeed           = 0;
        public static final double AmpSideIntakeSpeed          = 0;

        // speaker side variables

        public static final double SpeakerSideTimeoutMS        = 0;

        public static final double SpeakerSideDiagStepCM       = 0;
        public static final double SpeakerSideFirstStraightCM  = 0;
        public static final double SpeakerSideSecondStraightCM = 0;
        public static final double SpeakerSideThirdStraightCM  = 0;

        public static final double SpeakerSideShootAngle       = 0;
        public static final double SpeakerSideSwivelAngle      = 0;
        public static final double SpeakerSideFirstAngle       = 0;

        public static final double SpeakerSideDriveSpeed       = 0;
        public static final double SpeakerSidePivotSpeed       = 0;
        public static final double SpeakerSideShootSpeed       = 0;
        public static final double SpeakerSideIntakeSpeed      = 0;
        public static final double SpeakerSideSwivelSpeed      = 0;

        // outside side variables

        public static final double OutsideSideTimeoutMS        = 0;

        public static final double OutsideSideDiagStepCM       = 0;
        public static final double OutsideSideFirstStraightCM  = 0;
        public static final double OutsideSideSecondStraightCM = 0;
        public static final double OutsideSideThirdStraightCM  = 0;

        public static final double OutsideSideFirstAngle       = 0;
        public static final double OutsideSideSecondAngle      = 0;
        public static final double OutsideSideShooterAngle     = 0;

        public static final double OutsideSideShooterSpeed     = 0;
        public static final double OutsideSidePivotSpeed       = 0;
        public static final double OutsideSideDriveSpeed       = 0;
        public static final double OutsideSideIntakeSpeed      = 0;

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

        public static final double ENCODER_COUNTS_PER_CM         = 0;

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

        public static final double DEFAULT_DRIVE_SPEED       = 0;
        public static final double DEFAULT_TURN_SPEED        = 0;

        public static final double TURN_TO_HEADING_CLOSE     = 0;
        public static final double TURN_TO_HEADING_TOLERANCE = 0;

    }

    public static final class LightConstants {

        public static final int LED_STRIP_PWM_PORT     = 9;
        public static final int LED_STRIP_LENGTH       = 60;
        public static final int LED_STICK_TAKEN_LENGTH = 5;
        public static final int BOOST_INDEX            = 0;


    }

    public static final class ArmConstants {

        public static final int     PIVOT_PORT                           = 20;
        public static final int     KEEPER_PORT                          = 2;
        public static final int     INTAKE_LOWER_PORT                    = 1;
        public static final int     INTAKE_HIGHER_PORT                   = 0;

        public static final boolean PIVOT_INVERTED                       = true;

        public static final double  PIVOT_ARM_ENCODER_COUNT_PER_ROTATION = 37.2;

        public static final double  MIN_ARM_PIVOT_ANGLE                  = -26.5;
        public static final double  MAX_ARM_PIVOT_ANGLE                  = 130.0;

        public static final int     EQUILIBRIUM_ARM_ANGLE                = 68;
        public static final int     EQUILIBRIUM_ARM_ANGLE_CLOSE          = 10;
        public static final int     EQUILIBRIUM_ARM_ANGLE_TOLERANCE      = 2;

        public static final double  PIVOT_DEFAULT_SPEED                  = 0.1;

        public static final double  INTAKE_ANGLE                         = -25;
        public static final double  INTAKE_SPEED                         = 0.5;

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

        public static final double PIVOT_TO_ANGLE_PID_KP = 0.005;
        public static final double PIVOT_TO_ANGLE_PID_KI = 0;
        public static final double PIVOT_TO_ANGLE_PID_KD = 0;



    }

    public static final class ClimbConstants {

        public static final int     LEFT_CLIMB_PORT               = 3;
        public static final int     RIGHT_CLIMB_PORT              = 4;

        public static final boolean LEFT_CLIMBER_REVERSED         = false;
        public static final boolean RIGHT_CLIMBER_REVERSED        = false;

        public static final int     ENCODER_COUNTS_PER_REVOLUTION = 0;
        public static final int     REVOLUTIONS_TO_CM             = 0;

        public static final double  MAXIMUM_DISPLACEMENT          = 0;
        public static final double  LOWERED_DISP_LEVEL            = 0;
        public static final double  CLIMBER_DEFAULT_SPEED         = 0;

        public static enum ClimbStates {
            CLOSE, FAR
        }

        public static final double CLIMB_DISP_CLOSE = 0;

        public static final double CLIMB_PID_KP     = 0;
        public static final double CLIMB_PID_KI     = 0;
        public static final double CLIMB_PID_KD     = 0;

        public static final double DISP_TOLERANCE   = 0;



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