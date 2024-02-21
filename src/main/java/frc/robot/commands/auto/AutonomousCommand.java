package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConstants.AutoPattern;
import frc.robot.commands.drive.MeasuredDriveAtHeadingCommand;
import frc.robot.commands.drive.MeasuredStraightDriveCommand;
import frc.robot.commands.drive.TimedDriveCommand;
import frc.robot.commands.drive.TimedStraightDriveCommand;
import frc.robot.commands.drive.TurnToHeadingCommand;
import frc.robot.subsystems.DriveSubsystem;

public class AutonomousCommand extends SequentialCommandGroup {

    public AutonomousCommand(DriveSubsystem driveSubsystem,
        SendableChooser<AutoPattern> autoPatternChooser) {

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

        case OUTSIDE_ONE_SHOT:
            // From outside position, backs up and makes a speaker shot

            addCommands(new MeasuredStraightDriveCommand(100, 0.2, true, driveSubsystem));

            // Shoot arm, no command made yet

            break;

        case OUTSIDE_TWO_SHOT:
            // From outside position, backs up and makes a speaker shot
            // then it turns goes backwards to get a note, goes forward and turns back to make
            // another shot

            startHeading = driveSubsystem.getHeading();

            addCommands(new MeasuredStraightDriveCommand(100, -0.2, true, driveSubsystem));

            // Shoot arm, no command made yet

            addCommands(new TurnToHeadingCommand(0.2, startHeading - 60, true, 3, driveSubsystem));

            addCommands(new MeasuredStraightDriveCommand(30, -0.2, true, driveSubsystem));

            addCommands(new MeasuredStraightDriveCommand(30, 0.2, true, driveSubsystem));

            addCommands(new TurnToHeadingCommand(0.2, startHeading, true, 3, driveSubsystem));

            // Shoot arm, no command made yet

            break;

        case SPEAKER_THREE_SHOT:

            startHeading = driveSubsystem.getHeading();

            addCommands(new MeasuredStraightDriveCommand(100, -0.2, true, driveSubsystem));

            // Shoot arm, no command made yet

            addCommands(new MeasuredStraightDriveCommand(100, -0.2, true, driveSubsystem));

            // Shoot arm, no command made yet

            addCommands(new TurnToHeadingCommand(0.2, startHeading + 90, true, 3000, driveSubsystem));

            addCommands(new MeasuredStraightDriveCommand(100, -0.2, true, driveSubsystem));

            addCommands(new TurnToHeadingCommand(0.2, startHeading + 30, true, 3000, driveSubsystem));

            // Shoot arm, no command made yet

            break;

        case SPEAKER_FOUR_SHOT:

            startHeading = driveSubsystem.getHeading();

            addCommands(new MeasuredStraightDriveCommand(100, -0.2, true, driveSubsystem));

            // Shoot arm, no command made yet

            addCommands(new MeasuredStraightDriveCommand(100, -0.2, true, driveSubsystem));

            // Shoot arm, no command made yet

            addCommands(new TurnToHeadingCommand(0.2, startHeading + 90, true, 3, driveSubsystem));

            addCommands(new MeasuredStraightDriveCommand(100, -0.2, true, driveSubsystem));

            addCommands(new TurnToHeadingCommand(0.2, startHeading + 30, true, 3, driveSubsystem));

            // Shoot arm, no command made yet

            addCommands(new TurnToHeadingCommand(0.2, startHeading - 90, true, 3, driveSubsystem));

            addCommands(new MeasuredStraightDriveCommand(200, -0.2, true, driveSubsystem));

            addCommands(new TurnToHeadingCommand(0.2, startHeading - 30, true, 3, driveSubsystem));

            // Shoot arm, no command made yet

            break;

        case AMP_ONE_SHOT:

            startHeading = driveSubsystem.getHeading();

            addCommands(new MeasuredStraightDriveCommand(100, -0.2, true, driveSubsystem));

            // Shoot arm, no command made yet

            addCommands(new TurnToHeadingCommand(0.2, startHeading + 45, true, 3, driveSubsystem));

            addCommands(new MeasuredStraightDriveCommand(100, -0.2, true, driveSubsystem));

            break;

        case AMP_TWO_SHOT:

            startHeading = driveSubsystem.getHeading();

            addCommands(new MeasuredStraightDriveCommand(100, -0.2, true, driveSubsystem));

            // Shoot arm, no command made yet

            addCommands(new TurnToHeadingCommand(0.2, startHeading + 45, true, 3, driveSubsystem));

            addCommands(new MeasuredStraightDriveCommand(200, -0.2, true, driveSubsystem));

            addCommands(new TurnToHeadingCommand(0.2, startHeading + 22.5, true, 3, driveSubsystem));

            // Shoot arm, no command made yet

            break;

        case AMP_ONE_SHOT_ONE_AMP:

            startHeading = driveSubsystem.getHeading();

            addCommands(new MeasuredStraightDriveCommand(100, -0.2, true, driveSubsystem));

            // Shoot arm, no command made yet

            addCommands(new TurnToHeadingCommand(0.2, startHeading + 45, true, 3, driveSubsystem));

            addCommands(new MeasuredStraightDriveCommand(200, -0.2, true, driveSubsystem));

            addCommands(new TurnToHeadingCommand(0.2, startHeading + 135, true, 3, driveSubsystem));

            addCommands(new MeasuredStraightDriveCommand(100, 0.2, true, driveSubsystem));

            // Shoot arm into amp, no command made yet

            break;

        case AMP_ONE_AMP:

            startHeading = driveSubsystem.getHeading();

            addCommands(new MeasuredStraightDriveCommand(100, -0.2, true, driveSubsystem));

            addCommands(new TurnToHeadingCommand(0.2, startHeading + 31, true, 3, driveSubsystem));

            addCommands(new MeasuredStraightDriveCommand(206, -0.2, true, driveSubsystem));

            addCommands(new TurnToHeadingCommand(0.2, startHeading + 135, true, 3, driveSubsystem));

            addCommands(new MeasuredStraightDriveCommand(50, 0.2, true, driveSubsystem));

            // Shoot arm into amp, no command made yet

            break;

        case AMP_TWO_AMP:

            startHeading = driveSubsystem.getHeading();

            addCommands(new MeasuredStraightDriveCommand(100, -0.2, true, driveSubsystem));

            addCommands(new TurnToHeadingCommand(0.2, startHeading + 31, true, 3, driveSubsystem));

            addCommands(new MeasuredStraightDriveCommand(206, -0.2, true, driveSubsystem));

            addCommands(new TurnToHeadingCommand(0.2, startHeading + 135, true, 3, driveSubsystem));

            addCommands(new MeasuredStraightDriveCommand(50, 0.2, true, driveSubsystem));

            // Shoot arm into amp, no command made yet

            addCommands(new MeasuredStraightDriveCommand(100, -0.2, true, driveSubsystem));

            // Shoot arm into amp, no command made yet

            break;

        }
    }

}

