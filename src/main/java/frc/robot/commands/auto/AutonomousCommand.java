package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConstants.AutoPattern;
import frc.robot.commands.drive.MeasuredDriveAtHeadingCommand;
import frc.robot.commands.drive.TimedDriveCommand;
import frc.robot.commands.drive.TimedStraightDriveCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;

public class AutonomousCommand extends SequentialCommandGroup {

    public AutonomousCommand(DriveSubsystem driveSubsystem,
        SendableChooser<AutoPattern> autoPatternChooser, ArmSubsystem armSubsystem) {

        // Default is to do nothing.
        // If more commands are added, the instant command will end and
        // the next command will be executed.
        addCommands(new InstantCommand());

        AutoPattern   autoPattern = autoPatternChooser.getSelected();

        Alliance      alliance    = DriverStation.getAlliance().orElse(null);

        StringBuilder sb          = new StringBuilder();
        sb.append("Auto Selections");
        sb.append("\n   Auto Pattern  : ").append(autoPattern);
        sb.append("\n   Alliance      : ").append(alliance);

        System.out.println(sb.toString());

        // If any inputs are null, then there was some kind of error.
        if (autoPattern == null) {
            System.out.println("*** ERROR - null found in auto pattern builder ***");
            return;
        }

        // Print an error if the alliance is not set
        if (alliance == null) {
            System.out.println("*** ERROR **** null Alliance ");
            return;
        }

        double startHeading;
        /*
         * Compose the appropriate auto commands
         */
        switch (autoPattern) {

        case DO_NOTHING:
        default:
            return;

        case DRIVE_FORWARD:
            // Drive forward for 2 seconds
            addCommands(new TimedDriveCommand(2000, 0.5, 0.5, true, driveSubsystem));
            break;

        case DRIVE_FORWARD_PID_TIMED:
            // Drive forward for 30 seconds
            addCommands(new TimedStraightDriveCommand(1000, 1, true, 186, driveSubsystem));
            break;

        case DRIVE_FORWARD_PID_MEASURED:
            // Drive forward for 10 meters
            addCommands(new MeasuredDriveAtHeadingCommand(100, 0.1, true, 355, driveSubsystem));
            break;
        case TEST_ARM_COMMANDS:
            // program that will test the intake, pivot shoot, and pivot to angle


            /*
             * addCommands(new MeasuredStraightDriveCommand(100, 0.2, true, driveSubsystem));
             * 
             * addCommands(new PivotToAngleCommand(0.1, 30, true, 10000, armSubsystem));
             * 
             * addCommands(new IntakeCommand(0.5, 0.1, 10000, armSubsystem));
             * 
             * addCommands(new PivotToAngleCommand(0.1, 30, true, 10000, armSubsystem));
             * 
             * addCommands(new PivotShootCommand(1, 0.1, 90, 10000, armSubsystem));
             */
            break;

        /*
         * case AMP_ONE_SHOT:
         * // From outside position, backs up and makes a speaker shot
         * // Starts at the edge of the subwoofer facing away from it
         * 
         * startHeading = driveSubsystem.getHeading();
         * 
         * // moves backwards to make first shot
         * 
         * addCommands(new MeasuredStraightDriveCommand(AutoConstants.AmpSideDiagStepCM,
         * -AutoConstants.AmpSideDriveSpeed, true,
         * driveSubsystem));
         * 
         * // makes first shot
         * 
         * addCommands(new PivotShootCommand(AutoConstants.AmpSideShootSpeed,
         * AutoConstants.AmpSidePivotSpeed,
         * AutoConstants.AmpSideShootAngle, AutoConstants.AmpSideTimeoutMS, armSubsystem));
         * 
         * // turns and leaves starting zone to get mobility points
         * 
         * addCommands(new TurnToHeadingCommand(AutoConstants.AmpSideDriveSpeed,
         * startHeading + AutoConstants.AmpSideFirstAngle, true, AutoConstants.AmpSideTimeoutMS,
         * driveSubsystem));
         * 
         * addCommands(
         * new MeasuredStraightDriveCommand(AutoConstants.AmpSideFirstStraightCM,
         * -AutoConstants.AmpSideDriveSpeed, true,
         * driveSubsystem));
         * 
         * break;
         * 
         * case AMP_TWO_SHOT:
         * // From outside position, backs up and makes a speaker shot
         * // then it turns goes backwards to get a note, goes forward and turns back to make
         * // another shot. starts at the edge of the subwoofer facing away from it
         * 
         * startHeading = driveSubsystem.getHeading();
         * 
         * // backing up for first shot
         * 
         * addCommands(new MeasuredStraightDriveCommand(AutoConstants.AmpSideDiagStepCM,
         * AutoConstants.AmpSideDriveSpeed, true,
         * driveSubsystem));
         * 
         * // first shot
         * 
         * addCommands(new PivotShootCommand(AutoConstants.AmpSideShootSpeed,
         * AutoConstants.AmpSidePivotSpeed,
         * AutoConstants.AmpSideShootAngle, AutoConstants.AmpSideTimeoutMS, armSubsystem));
         * 
         * // turns and drives towards first note while intaking
         * 
         * addCommands(new TurnToHeadingCommand(AutoConstants.AmpSideDriveSpeed,
         * startHeading + AutoConstants.AmpSideFirstAngle, true, AutoConstants.AmpSideTimeoutMS,
         * driveSubsystem));
         * 
         * addCommands(new IntakeCommand(AutoConstants.AmpSideIntakeSpeed,
         * AutoConstants.AmpSidePivotSpeed,
         * AutoConstants.AmpSideTimeoutMS, armSubsystem));
         * 
         * addCommands(new MeasuredStraightDriveCommand(AutoConstants.AmpSideSecondStraightCM,
         * AutoConstants.AmpSideDriveSpeed,
         * true, driveSubsystem));
         * 
         * // goes back to shooting place
         * 
         * addCommands(new MeasuredStraightDriveCommand(AutoConstants.AmpSideSecondStraightCM,
         * -AutoConstants.AmpSideDriveSpeed,
         * true, driveSubsystem));
         * 
         * // second shot
         * 
         * addCommands(new TurnToHeadingCommand(AutoConstants.AmpSideDriveSpeed,
         * startHeading + AutoConstants.AmpSideFirstAngle, true, AutoConstants.AmpSideTimeoutMS,
         * driveSubsystem));
         * 
         * addCommands(new PivotShootCommand(AutoConstants.AmpSideShootSpeed,
         * AutoConstants.AmpSidePivotSpeed,
         * AutoConstants.AmpSideShootAngle, AutoConstants.AmpSideTimeoutMS, armSubsystem));
         * 
         * // turns and leaves starting zone to get mobility points
         * 
         * addCommands(new TurnToHeadingCommand(AutoConstants.AmpSideDriveSpeed,
         * startHeading + AutoConstants.AmpSideFirstAngle, true, AutoConstants.AmpSideTimeoutMS,
         * driveSubsystem));
         * 
         * addCommands(
         * new MeasuredStraightDriveCommand(AutoConstants.AmpSideFirstStraightCM,
         * -AutoConstants.AmpSideDriveSpeed, true,
         * driveSubsystem));
         * 
         * break;
         * 
         * case AMP_THREE_SHOT:
         * /*
         * From amp position, robot starts at the edge of the subwoofer in amp position,
         * backs up, makes first shot, turns to go get second note, returns for second shot,
         * then backs up more to get the third ring and returns finally to get the third sho
         */
        /*
         * startHeading = driveSubsystem.getHeading();
         * 
         * // backing up for first shot
         * 
         * addCommands(new MeasuredStraightDriveCommand(AutoConstants.AmpSideDiagStepCM,
         * AutoConstants.AmpSideDriveSpeed, true,
         * driveSubsystem));
         * 
         * // first shot
         * 
         * addCommands(new PivotShootCommand(AutoConstants.AmpSideShootSpeed,
         * AutoConstants.AmpSidePivotSpeed,
         * AutoConstants.AmpSideShootAngle, AutoConstants.AmpSideTimeoutMS, armSubsystem));
         * 
         * // turns and drives towards first note while intaking
         * 
         * addCommands(new TurnToHeadingCommand(AutoConstants.AmpSideDriveSpeed,
         * startHeading + AutoConstants.AmpSideFirstAngle, true, AutoConstants.AmpSideTimeoutMS,
         * driveSubsystem));
         * 
         * addCommands(new IntakeCommand(AutoConstants.AmpSideIntakeSpeed,
         * AutoConstants.AmpSidePivotSpeed,
         * AutoConstants.AmpSideTimeoutMS, armSubsystem));
         * 
         * addCommands(new MeasuredStraightDriveCommand(AutoConstants.AmpSideSecondStraightCM,
         * AutoConstants.AmpSideDriveSpeed,
         * true, driveSubsystem));
         * 
         * // goes back to shooting place
         * 
         * addCommands(new MeasuredStraightDriveCommand(AutoConstants.AmpSideSecondStraightCM,
         * -AutoConstants.AmpSideDriveSpeed,
         * true, driveSubsystem));
         * 
         * // second shot
         * 
         * addCommands(new TurnToHeadingCommand(AutoConstants.AmpSideDriveSpeed,
         * startHeading + AutoConstants.AmpSideFirstAngle, true, AutoConstants.AmpSideTimeoutMS,
         * driveSubsystem));
         * 
         * addCommands(new PivotShootCommand(AutoConstants.AmpSideShootSpeed,
         * AutoConstants.AmpSidePivotSpeed,
         * AutoConstants.AmpSideShootAngle, AutoConstants.AmpSideTimeoutMS, armSubsystem));
         * 
         * // goes for the third note while intaking
         * 
         * addCommands(new TurnToHeadingCommand(AutoConstants.AmpSideDriveSpeed,
         * startHeading + AutoConstants.AmpSideFirstAngle, true, AutoConstants.AmpSideTimeoutMS,
         * driveSubsystem));
         * 
         * addCommands(
         * new MeasuredStraightDriveCommand(AutoConstants.AmpSideFirstStraightCM,
         * AutoConstants.AmpSideDriveSpeed, true,
         * driveSubsystem));
         * 
         * addCommands(
         * new MeasuredStraightDriveCommand(AutoConstants.AmpSideSecondStraightCM,
         * AutoConstants.AmpSideDriveSpeed, true,
         * driveSubsystem));
         * 
         * addCommands(new TurnToHeadingCommand(AutoConstants.AmpSideDriveSpeed,
         * startHeading + AutoConstants.AmpSideSecondAngle, true, AutoConstants.AmpSideTimeoutMS,
         * driveSubsystem));
         * 
         * addCommands(new IntakeCommand(AutoConstants.AmpSideIntakeSpeed,
         * AutoConstants.AmpSidePivotSpeed,
         * AutoConstants.AmpSideTimeoutMS, armSubsystem));
         * 
         * addCommands(
         * new MeasuredStraightDriveCommand(AutoConstants.AmpSideThirdStraightCM,
         * AutoConstants.AmpSideDriveSpeed, true,
         * driveSubsystem));
         * 
         * addCommands(
         * new MeasuredStraightDriveCommand(AutoConstants.AmpSideThirdStraightCM,
         * -AutoConstants.AmpSideDriveSpeed, true,
         * driveSubsystem));
         * 
         * addCommands(new TurnToHeadingCommand(AutoConstants.AmpSideDriveSpeed,
         * startHeading + AutoConstants.AmpSideFirstAngle, true, AutoConstants.AmpSideTimeoutMS,
         * driveSubsystem));
         * 
         * addCommands(
         * new MeasuredStraightDriveCommand(AutoConstants.AmpSideSecondStraightCM,
         * -AutoConstants.AmpSideDriveSpeed, true,
         * driveSubsystem));
         * 
         * addCommands(
         * new MeasuredStraightDriveCommand(AutoConstants.AmpSideFirstStraightCM,
         * -AutoConstants.AmpSideDriveSpeed, true,
         * driveSubsystem));
         * 
         * // third shot
         * 
         * addCommands(new TurnToHeadingCommand(AutoConstants.AmpSideDriveSpeed,
         * startHeading + AutoConstants.AmpSideFirstAngle, true, AutoConstants.AmpSideTimeoutMS,
         * driveSubsystem));
         * 
         * addCommands(new PivotShootCommand(AutoConstants.AmpSideShootSpeed,
         * AutoConstants.AmpSidePivotSpeed,
         * AutoConstants.AmpSideShootAngle, AutoConstants.AmpSideTimeoutMS, armSubsystem));
         * 
         * // turns and leaves starting zone to get mobility points
         * 
         * addCommands(new TurnToHeadingCommand(AutoConstants.AmpSideDriveSpeed,
         * startHeading + AutoConstants.AmpSideFirstAngle, true, AutoConstants.AmpSideTimeoutMS,
         * driveSubsystem));
         * 
         * addCommands(
         * new MeasuredStraightDriveCommand(AutoConstants.AmpSideFirstStraightCM,
         * -AutoConstants.AmpSideDriveSpeed, true,
         * driveSubsystem));
         * 
         * break;
         * 
         * case SPEAKER_ONE_SHOT:
         * /*
         * From speaker position, robot starts in the center facing away from speaker
         * The robot backs up and makes one shot, then backs up further to get mobility points
         */

        // backs up
        /*
         * addCommands(
         * new MeasuredStraightDriveCommand(AutoConstants.SpeakerSideFirstStraightCM,
         * AutoConstants.SpeakerSideDriveSpeed,
         * true, driveSubsystem));
         * 
         * // first shot
         * 
         * addCommands(new PivotShootCommand(AutoConstants.SpeakerSideShootSpeed,
         * AutoConstants.SpeakerSidePivotSpeed,
         * AutoConstants.SpeakerSideShootAngle, AutoConstants.SpeakerSideTimeoutMS, armSubsystem));
         * 
         * // leaves starting zone to get mobility points
         * 
         * addCommands(
         * new MeasuredStraightDriveCommand(AutoConstants.SpeakerSideSecondStraightCM,
         * AutoConstants.SpeakerSideDriveSpeed,
         * true,
         * driveSubsystem));
         * 
         * 
         * case SPEAKER_TWO_SHOT:
         * /*
         * From speaker position, robot starts in the center facing away from speaker
         * The robot backs up
         */

        // backs up
        /*
         * addCommands(
         * new MeasuredStraightDriveCommand(AutoConstants.SpeakerSideFirstStraightCM,
         * AutoConstants.SpeakerSideDriveSpeed,
         * true, driveSubsystem));
         * 
         * // first shot
         * 
         * addCommands(new PivotShootCommand(AutoConstants.SpeakerSideShootSpeed,
         * AutoConstants.SpeakerSidePivotSpeed,
         * AutoConstants.SpeakerSideShootAngle, AutoConstants.SpeakerSideTimeoutMS, armSubsystem));
         * 
         * // goes for second shot while intaking
         * 
         * addCommands(
         * new MeasuredStraightDriveCommand(AutoConstants.SpeakerSideSecondStraightCM,
         * AutoConstants.SpeakerSideDriveSpeed,
         * true, driveSubsystem));
         * 
         * addCommands(new IntakeCommand(AutoConstants.SpeakerSideIntakeSpeed,
         * AutoConstants.SpeakerSidePivotSpeed,
         * AutoConstants.AmpSideTimeoutMS, armSubsystem));
         * 
         * // goes back to make new shot
         * 
         * addCommands(
         * new MeasuredStraightDriveCommand(AutoConstants.SpeakerSideSecondStraightCM,
         * -AutoConstants.SpeakerSideDriveSpeed,
         * true, driveSubsystem));
         * 
         * // second shot
         * 
         * addCommands(new PivotShootCommand(AutoConstants.SpeakerSideShootSpeed,
         * AutoConstants.SpeakerSidePivotSpeed,
         * AutoConstants.SpeakerSideShootAngle, AutoConstants.SpeakerSideTimeoutMS, armSubsystem));
         * 
         * // goes outside again to make mobility points
         * 
         * addCommands(
         * new MeasuredStraightDriveCommand(AutoConstants.SpeakerSideSecondStraightCM,
         * AutoConstants.SpeakerSideDriveSpeed,
         * true, driveSubsystem));
         * 
         * break;
         * 
         * case SPEAKER_THREE_SHOT:
         * /*
         * From speaker position, robot backs up to make first shot,
         * Grabs last note, goes back to make second shot,
         * then turns left to grab the last note, turns to the speaker
         * and makes the last shot
         */
        /*
         * startHeading = driveSubsystem.getHeading();
         * 
         * // backs up
         * 
         * addCommands(
         * new MeasuredStraightDriveCommand(AutoConstants.SpeakerSideDiagStepCM,
         * AutoConstants.SpeakerSideDriveSpeed,
         * true, driveSubsystem));
         * 
         * // first shot
         * 
         * addCommands(new PivotShootCommand(AutoConstants.SpeakerSideShootSpeed,
         * AutoConstants.SpeakerSidePivotSpeed,
         * AutoConstants.SpeakerSideShootAngle, AutoConstants.SpeakerSideTimeoutMS, armSubsystem));
         * 
         * // goes for second shot while intaking
         * 
         * addCommands(new IntakeCommand(AutoConstants.SpeakerSideIntakeSpeed,
         * AutoConstants.SpeakerSidePivotSpeed,
         * AutoConstants.AmpSideTimeoutMS, armSubsystem));
         * 
         * addCommands(
         * new MeasuredStraightDriveCommand(AutoConstants.SpeakerSideFirstStraightCM,
         * AutoConstants.SpeakerSideDriveSpeed,
         * true, driveSubsystem));
         * 
         * // goes back to make new shot
         * 
         * addCommands(
         * new MeasuredStraightDriveCommand(AutoConstants.SpeakerSideFirstStraightCM,
         * -AutoConstants.SpeakerSideDriveSpeed,
         * true, driveSubsystem));
         * 
         * // second shot
         * 
         * addCommands(new PivotShootCommand(AutoConstants.SpeakerSideShootSpeed,
         * AutoConstants.SpeakerSidePivotSpeed,
         * AutoConstants.SpeakerSideShootAngle, AutoConstants.SpeakerSideTimeoutMS, armSubsystem));
         * 
         * // goes for the third note
         * 
         * addCommands(
         * new MeasuredStraightDriveCommand(AutoConstants.SpeakerSideFirstStraightCM,
         * AutoConstants.SpeakerSideDriveSpeed,
         * true, driveSubsystem));
         * 
         * addCommands(new SwivelOnLeftWheelCommand(AutoConstants.SpeakerSideSwivelSpeed,
         * startHeading + AutoConstants.SpeakerSideSwivelAngle,
         * true, AutoConstants.SpeakerSideTimeoutMS, driveSubsystem));
         * 
         * addCommands(new SwivelOnRightWheelCommand(AutoConstants.SpeakerSideSwivelSpeed,
         * startHeading,
         * true, AutoConstants.SpeakerSideTimeoutMS, driveSubsystem));
         * 
         * addCommands(
         * new MeasuredStraightDriveCommand(AutoConstants.SpeakerSideSecondStraightCM,
         * AutoConstants.SpeakerSideDriveSpeed,
         * true, driveSubsystem));
         * 
         * addCommands(new TurnToHeadingCommand(AutoConstants.SpeakerSideDriveSpeed,
         * startHeading + AutoConstants.SpeakerSideFirstAngle, true,
         * AutoConstants.SpeakerSideTimeoutMS,
         * driveSubsystem));
         * 
         * addCommands(new IntakeCommand(AutoConstants.SpeakerSideIntakeSpeed,
         * AutoConstants.AmpSidePivotSpeed,
         * AutoConstants.SpeakerSideTimeoutMS, armSubsystem));
         * 
         * addCommands(
         * new MeasuredStraightDriveCommand(AutoConstants.SpeakerSideThirdStraightCM,
         * AutoConstants.SpeakerSideDriveSpeed,
         * true, driveSubsystem));
         * 
         * // comes back
         * 
         * addCommands(
         * new MeasuredStraightDriveCommand(AutoConstants.SpeakerSideThirdStraightCM,
         * -AutoConstants.SpeakerSideDriveSpeed,
         * true, driveSubsystem));
         * 
         * addCommands(new TurnToHeadingCommand(AutoConstants.SpeakerSideDriveSpeed,
         * startHeading, true, AutoConstants.SpeakerSideTimeoutMS, driveSubsystem));
         * 
         * addCommands(
         * new MeasuredStraightDriveCommand(AutoConstants.SpeakerSideSecondStraightCM,
         * -AutoConstants.SpeakerSideDriveSpeed,
         * true, driveSubsystem));
         * 
         * addCommands(new SwivelOnRightWheelCommand(-AutoConstants.SpeakerSideSwivelSpeed,
         * startHeading + AutoConstants.SpeakerSideSwivelAngle, true,
         * AutoConstants.SpeakerSideTimeoutMS, driveSubsystem));
         * 
         * addCommands(new SwivelOnLeftWheelCommand(-AutoConstants.SpeakerSideSwivelSpeed,
         * startHeading,
         * true, AutoConstants.SpeakerSideTimeoutMS, driveSubsystem));
         * 
         * addCommands(
         * new MeasuredStraightDriveCommand(AutoConstants.SpeakerSideSecondStraightCM,
         * -AutoConstants.SpeakerSideDriveSpeed,
         * true, driveSubsystem));
         * 
         * addCommands(
         * new MeasuredStraightDriveCommand(AutoConstants.SpeakerSideFirstStraightCM,
         * -AutoConstants.SpeakerSideDriveSpeed,
         * true, driveSubsystem));
         * 
         * // third shot
         * 
         * addCommands(new PivotShootCommand(AutoConstants.SpeakerSideShootSpeed,
         * AutoConstants.SpeakerSidePivotSpeed,
         * AutoConstants.SpeakerSideShootAngle, AutoConstants.SpeakerSideTimeoutMS, armSubsystem));
         * 
         * // moves back out to get mobility points
         * 
         * addCommands(
         * new MeasuredStraightDriveCommand(AutoConstants.SpeakerSideFirstStraightCM,
         * AutoConstants.SpeakerSideDriveSpeed,
         * true, driveSubsystem));
         * 
         * break;
         * 
         */
        }
    }
}

