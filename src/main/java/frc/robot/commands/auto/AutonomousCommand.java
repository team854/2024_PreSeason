package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.AutoConstants.AutoPattern;
import frc.robot.commands.arm.IntakeCommand;
import frc.robot.commands.arm.PivotShootCommand;
import frc.robot.commands.arm.PivotToAngleCommand;
import frc.robot.commands.drive.MeasuredDriveAtHeadingCommand;
import frc.robot.commands.drive.MeasuredStraightDriveCommand;
import frc.robot.commands.drive.TimedStraightDriveCommand;
import frc.robot.commands.drive.TurnToHeadingCommand;
import frc.robot.operator.OperatorInput;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;

public class AutonomousCommand extends SequentialCommandGroup {

    public AutonomousCommand(DriveSubsystem driveSubsystem,
        SendableChooser<AutoPattern> autoPatternChooser, ArmSubsystem armSubsystem, OperatorInput operatorInput) {

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


            addCommands(new IntakeCommand(operatorInput, armSubsystem, true)
                .alongWith(new MeasuredStraightDriveCommand(AutoConstants.AmpSideFirstStraightCM,
                    AutoConstants.AmpSideDriveSpeed,
                    true, driveSubsystem)));

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

            addCommands(new MeasuredStraightDriveCommand(100, 0.2, true, driveSubsystem));

            addCommands(new PivotToAngleCommand(0.3, 30, 5000, armSubsystem));

            addCommands(new IntakeCommand(operatorInput, armSubsystem, true));

            addCommands(new IntakeCommand(operatorInput, armSubsystem, true));

            // addCommands(new MeasuredStraightDriveCommand(100, 0.2, true, driveSubsystem));

            // addCommands(new PivotToAngleCommand(0.3, 30, true, 10000, armSubsystem));

            // addCommands(new IntakeCommand(0.5, 0.3, 10000, armSubsystem));

            // addCommands(new PivotToAngleCommand(0.3, 30, true, 10000, armSubsystem));

            // addCommands(new PivotShootCommand(1, 0.3, 90, 10000, armSubsystem));

            break;



        case BLUE_AMP_ONE_SHOT:
            // From amp position, makes a speaker shot then leaves mobility zone

            startHeading = driveSubsystem.getHeading();

            // makes first shot

            addCommands(new PivotShootCommand(AutoConstants.AmpSideShootSpeed,
                AutoConstants.AmpSideShootAngle, AutoConstants.AmpSideTimeoutMS, armSubsystem));

            // moves backwards

            addCommands(new MeasuredStraightDriveCommand(AutoConstants.AmpSideDiagStepCM,
                AutoConstants.AmpSideDriveSpeed, true,
                driveSubsystem).alongWith(new IntakeCommand(operatorInput, armSubsystem, true)));

            // turns and leaves starting zone to get mobility points

            addCommands(new TurnToHeadingCommand(AutoConstants.AmpSideDriveSpeed,
                startHeading + AutoConstants.AmpSideFirstAngle, true, AutoConstants.AmpSideTimeoutMS,
                driveSubsystem));

            addCommands(new IntakeCommand(operatorInput, armSubsystem, true)
                .alongWith(new MeasuredStraightDriveCommand(AutoConstants.AmpSideFirstStraightCM,
                    -AutoConstants.AmpSideDriveSpeed,
                    true, driveSubsystem)));

            break;



        case RED_AMP_ONE_SHOT:
            // From amp outside position, makes a speaker shot then leaves mobility zone

            startHeading = driveSubsystem.getHeading();

            // first shot

            addCommands(new PivotShootCommand(AutoConstants.AmpSideShootSpeed,
                AutoConstants.AmpSideShootAngle, AutoConstants.AmpSideTimeoutMS, armSubsystem));

            // backing up

            addCommands(new MeasuredStraightDriveCommand(AutoConstants.AmpSideDiagStepCM,
                AutoConstants.AmpSideDriveSpeed, true,
                driveSubsystem).alongWith(new IntakeCommand(operatorInput, armSubsystem, true)));

            // turns and drives towards first note while intaking

            addCommands(new TurnToHeadingCommand(AutoConstants.AmpSideDriveSpeed,
                startHeading - AutoConstants.AmpSideFirstAngle, true, AutoConstants.AmpSideTimeoutMS,
                driveSubsystem));

            addCommands(new IntakeCommand(operatorInput, armSubsystem, true)
                .alongWith(new MeasuredStraightDriveCommand(AutoConstants.AmpSideFirstStraightCM,
                    -AutoConstants.AmpSideDriveSpeed,
                    true, driveSubsystem)));

            break;



        case BLUE_OUTSIDE_ONE_SHOT:

            // backs up to make a shot,
            // then backs into mobility zone

            addCommands(
                new PivotShootCommand(AutoConstants.OutsideSideShooterSpeed, AutoConstants.OutsideSideShooterAngle,
                    AutoConstants.OutsideSideTimeoutMS, armSubsystem));

            addCommands(
                new MeasuredStraightDriveCommand(AutoConstants.OutsideSideDiagStepCM,
                    AutoConstants.OutsideSideDriveSpeed, true,
                    driveSubsystem));

            break;



        case RED_OUTSIDE_ONE_SHOT:

            // backs up to make a shot,
            // then backs into mobility zone

            addCommands(
                new PivotShootCommand(AutoConstants.OutsideSideShooterSpeed, AutoConstants.OutsideSideShooterAngle,
                    AutoConstants.OutsideSideTimeoutMS, armSubsystem));

            addCommands(
                new MeasuredStraightDriveCommand(AutoConstants.OutsideSideDiagStepCM,
                    AutoConstants.OutsideSideDriveSpeed, true,
                    driveSubsystem));

            break;



        case RED_SPEAKER_ONE_SHOT:

            addCommands(
                new PivotShootCommand(AutoConstants.SpeakerSideShootSpeed, AutoConstants.SpeakerSideShootAngle,
                    AutoConstants.SpeakerSideTimeoutMS, armSubsystem));

            addCommands(
                new MeasuredStraightDriveCommand(AutoConstants.SpeakerSideDiagStepCM,
                    AutoConstants.SpeakerSideDriveSpeed, true,
                    driveSubsystem));

            break;



        case BLUE_SPEAKER_ONE_SHOT:

            addCommands(
                new PivotShootCommand(AutoConstants.SpeakerSideShootSpeed, AutoConstants.SpeakerSideShootAngle,
                    AutoConstants.SpeakerSideTimeoutMS, armSubsystem));

            addCommands(
                new MeasuredStraightDriveCommand(AutoConstants.SpeakerSideDiagStepCM,
                    AutoConstants.SpeakerSideDriveSpeed, true,
                    driveSubsystem));

            break;



        case BLUE_AMP_TWO_SHOT:
            // From outside position, makes a speaker and backs up
            // then it turns goes backwards to get a note, goes forward and turns back to make
            // another shot. starts at the edge of the subwoofer facing away from it

            startHeading = driveSubsystem.getHeading();

            // first shot

            addCommands(new PivotShootCommand(AutoConstants.AmpSideShootSpeed,
                AutoConstants.AmpSideShootAngle, AutoConstants.AmpSideTimeoutMS, armSubsystem));

            // backing up

            addCommands(new MeasuredStraightDriveCommand(AutoConstants.AmpSideDiagStepCM,
                AutoConstants.AmpSideDriveSpeed, true,
                driveSubsystem).alongWith(new IntakeCommand(operatorInput, armSubsystem, true)));

            // turns and drives towards first note while intaking

            addCommands(new TurnToHeadingCommand(AutoConstants.AmpSideDriveSpeed,
                startHeading + AutoConstants.AmpSideFirstAngle, true, AutoConstants.AmpSideTimeoutMS,
                driveSubsystem));

            addCommands(new MeasuredStraightDriveCommand(AutoConstants.AmpSideFirstStraightCM,
                AutoConstants.AmpSideDriveSpeed,
                true, driveSubsystem).alongWith(new IntakeCommand(operatorInput, armSubsystem, true)));

            // goes back to shooting place

            // Add 10cm to the return distance to ensure that it is flush with the subwoofer
            addCommands(new MeasuredStraightDriveCommand(AutoConstants.AmpSideFirstStraightCM + 10,
                -AutoConstants.AmpSideDriveSpeed,
                true, driveSubsystem));

            addCommands(new TurnToHeadingCommand(AutoConstants.AmpSideDriveSpeed,
                startHeading, true, AutoConstants.AmpSideTimeoutMS,
                driveSubsystem));

            addCommands(new MeasuredStraightDriveCommand(AutoConstants.AmpSideDiagStepCM,
                -AutoConstants.AmpSideDriveSpeed, true,
                driveSubsystem));

            // second shot

            addCommands(new PivotShootCommand(AutoConstants.AmpSideShootSpeed,
                AutoConstants.AmpSideShootAngle, AutoConstants.AmpSideTimeoutMS, armSubsystem));

            // turns and leaves starting zone to get mobility points

            addCommands(new MeasuredStraightDriveCommand(AutoConstants.AmpSideDiagStepCM,
                -AutoConstants.AmpSideDriveSpeed, true,
                driveSubsystem));

            addCommands(new TurnToHeadingCommand(AutoConstants.AmpSideDriveSpeed,
                startHeading + AutoConstants.AmpSideFirstAngle, true, AutoConstants.AmpSideTimeoutMS,
                driveSubsystem));

            addCommands(
                new MeasuredStraightDriveCommand(AutoConstants.AmpSideFirstStraightCM,
                    AutoConstants.AmpSideDriveSpeed, true,
                    driveSubsystem));

            break;


        case RED_AMP_TWO_SHOT:
            // From outside position, backs up and makes a speaker shot
            // then it turns goes backwards to get a note, goes forward and turns back to make
            // another shot. starts at the edge of the subwoofer facing away from it

            startHeading = driveSubsystem.getHeading();

            // first shot

            addCommands(new PivotShootCommand(AutoConstants.AmpSideShootSpeed,
                AutoConstants.AmpSideShootAngle, AutoConstants.AmpSideTimeoutMS, armSubsystem));

            // backing up

            addCommands(new MeasuredStraightDriveCommand(AutoConstants.AmpSideDiagStepCM,
                AutoConstants.AmpSideDriveSpeed, true,
                driveSubsystem).alongWith(new IntakeCommand(operatorInput, armSubsystem, true)));
            // turns and drives towards first note while intaking

            addCommands(new TurnToHeadingCommand(AutoConstants.AmpSideDriveSpeed,
                startHeading - AutoConstants.AmpSideFirstAngle, true, AutoConstants.AmpSideTimeoutMS,
                driveSubsystem));


            addCommands(new MeasuredStraightDriveCommand(AutoConstants.AmpSideFirstStraightCM,
                AutoConstants.AmpSideDriveSpeed,
                true, driveSubsystem).alongWith(new IntakeCommand(operatorInput, armSubsystem, true)));

            // goes back to shooting place

            // Add 10cm to the return distance to ensure that it is flush with the subwoofer
            addCommands(new MeasuredStraightDriveCommand(AutoConstants.AmpSideFirstStraightCM + 10,
                -AutoConstants.AmpSideDriveSpeed,
                true, driveSubsystem));

            addCommands(new TurnToHeadingCommand(AutoConstants.AmpSideDriveSpeed,
                startHeading, true, AutoConstants.AmpSideTimeoutMS,
                driveSubsystem));

            addCommands(new MeasuredStraightDriveCommand(AutoConstants.AmpSideDiagStepCM,
                -AutoConstants.AmpSideDriveSpeed, true,
                driveSubsystem));

            // second shot

            addCommands(new PivotShootCommand(AutoConstants.AmpSideShootSpeed,
                AutoConstants.AmpSideShootAngle, AutoConstants.AmpSideTimeoutMS, armSubsystem));

            // turns and leaves starting zone to get mobility points

            addCommands(new MeasuredStraightDriveCommand(AutoConstants.AmpSideDiagStepCM,
                -AutoConstants.AmpSideDriveSpeed, true,
                driveSubsystem));

            addCommands(new TurnToHeadingCommand(AutoConstants.AmpSideDriveSpeed,
                startHeading - AutoConstants.AmpSideFirstAngle, true, AutoConstants.AmpSideTimeoutMS,
                driveSubsystem));

            addCommands(
                new MeasuredStraightDriveCommand(AutoConstants.AmpSideFirstStraightCM,
                    AutoConstants.AmpSideDriveSpeed, true,
                    driveSubsystem));

            break;



        case BLUE_SPEAKER_TWO_SHOT:

            // first shot

            addCommands(
                new PivotShootCommand(AutoConstants.SpeakerSideShootSpeed, AutoConstants.SpeakerSideShootAngle,
                    AutoConstants.SpeakerSideTimeoutMS, armSubsystem));

            // backs up while intaking

            addCommands(new IntakeCommand(operatorInput, armSubsystem, true));

            addCommands(
                new MeasuredStraightDriveCommand(AutoConstants.SpeakerSideDiagStepCM,
                    AutoConstants.SpeakerSideDriveSpeed, true,
                    driveSubsystem).alongWith(new IntakeCommand(operatorInput, armSubsystem, true)));

            // goes back

            addCommands(
                new MeasuredStraightDriveCommand(AutoConstants.SpeakerSideDiagStepCM,
                    -AutoConstants.SpeakerSideDriveSpeed, true,
                    driveSubsystem));

            // second shot

            addCommands(
                new PivotShootCommand(AutoConstants.SpeakerSideShootSpeed, AutoConstants.SpeakerSideShootAngle,
                    AutoConstants.SpeakerSideTimeoutMS, armSubsystem));

            // goes to get mobility points

            addCommands(
                new MeasuredStraightDriveCommand(AutoConstants.SpeakerSideDiagStepCM,
                    AutoConstants.SpeakerSideDriveSpeed, true,
                    driveSubsystem));


            break;



        case RED_SPEAKER_TWO_SHOT:

            // first shot

            addCommands(
                new PivotShootCommand(AutoConstants.SpeakerSideShootSpeed, AutoConstants.SpeakerSideShootAngle,
                    AutoConstants.SpeakerSideTimeoutMS, armSubsystem));

            // backs up while intaking

            addCommands(new IntakeCommand(operatorInput, armSubsystem, true));

            addCommands(
                new MeasuredStraightDriveCommand(AutoConstants.SpeakerSideDiagStepCM,
                    AutoConstants.SpeakerSideDriveSpeed, true,
                    driveSubsystem).alongWith(new IntakeCommand(operatorInput, armSubsystem, true)));

            // goes back

            addCommands(
                new MeasuredStraightDriveCommand(AutoConstants.SpeakerSideDiagStepCM,
                    -AutoConstants.SpeakerSideDriveSpeed, true,
                    driveSubsystem));

            // second shot

            addCommands(
                new PivotShootCommand(AutoConstants.SpeakerSideShootSpeed, AutoConstants.SpeakerSideShootAngle,
                    AutoConstants.SpeakerSideTimeoutMS, armSubsystem));

            // goes to get mobility points

            addCommands(
                new MeasuredStraightDriveCommand(AutoConstants.SpeakerSideDiagStepCM,
                    AutoConstants.SpeakerSideDriveSpeed, true,
                    driveSubsystem));


            break;



        }
    }
}

