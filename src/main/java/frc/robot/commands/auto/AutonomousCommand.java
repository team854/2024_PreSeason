package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConstants.AutoPattern;
import frc.robot.commands.drive.MeasuredStraightDriveCommand;
import frc.robot.commands.drive.TimedDriveCommand;
import frc.robot.commands.drive.TimedStraightDriveCommand;
import frc.robot.subsystems.DriveSubsystem;

public class AutonomousCommand extends SequentialCommandGroup {

    public AutonomousCommand(DriveSubsystem driveSubsystem,
        SendableChooser<AutoPattern> autoPatternChooser) {

        // Default is to do nothing.
        // If more commands are added, the instant command will end and
        // the next command will be executed.
        addCommands(new InstantCommand());

        AutoPattern   autoPattern = autoPatternChooser.getSelected();

        Alliance      alliance    = DriverStation.getAlliance();

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
        else if (alliance == Alliance.Invalid) {
            System.out.println("*** ERROR *** Invalid alliance");
            return;
        }

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
            addCommands(new TimedStraightDriveCommand(30000, 0.5, true, 0, driveSubsystem));
            break;

        case DRIVE_FORWARD_PID_MEASURED:
            // Drive forward for 10 meters
            addCommands(new MeasuredStraightDriveCommand(1000, 0.5, true, driveSubsystem));
        }
    }

}

