package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConstants.AutoPattern;
import frc.robot.commands.arm.IntakeCommand;
import frc.robot.commands.arm.PivotShootCommand;
import frc.robot.commands.arm.PivotToAngleCommand;
import frc.robot.commands.drive.MeasuredDriveAtHeadingCommand;
import frc.robot.commands.drive.MeasuredStraightDriveCommand;
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

            addCommands(new MeasuredStraightDriveCommand(100, 0.2, true, driveSubsystem));

            addCommands(new PivotToAngleCommand(0.3, 30, true, 10000, armSubsystem));

            addCommands(new IntakeCommand(0.5, 0.3, 10000, armSubsystem));

            addCommands(new PivotToAngleCommand(0.3, 30, true, 10000, armSubsystem));

            addCommands(new PivotShootCommand(1, 0.3, 90, 10000, armSubsystem));
        case OUTSIDE_ONE_SHOT:
            // From outside position, backs up and makes a speaker shot

            // moves backwards to make first shot

            /*
             * addCommands(new MeasuredStraightDriveCommand(100, -0.2, true, driveSubsystem));
             * 
             * addCommands(new PivotToAngleCommand(0.3, 90, true, 3000, armSubsystem));
             * 
             * // first shot
             * 
             * addCommands(new ShooterCommand(1, 2000, armSubsystem));
             */
            break;

        case OUTSIDE_TWO_SHOT:
            // From outside position, backs up and makes a speaker shot
            // then it turns goes backwards to get a note, goes forward and turns back to make
            // another shot
            /*
             * startHeading = driveSubsystem.getHeading();
             * 
             * // moving arm while backign up for first shot
             * 
             * addCommands(new PivotToAngleCommand(0.2, 90, true, 3000, armSubsystem));
             * 
             * addCommands(new MeasuredStraightDriveCommand(100, -0.2, true, driveSubsystem));
             * 
             * // first shot
             * 
             * addCommands(new ShooterCommand(0.5, 2000, armSubsystem));
             * 
             * // turns and drives towards first note while intaking
             * 
             * addCommands(new TurnToHeadingCommand(0.2, startHeading + 120, true, 3000,
             * driveSubsystem));
             * 
             * addCommands(new PivotToAngleCommand(0.2, 0, true, 3000, armSubsystem));
             * 
             * addCommands(new IntakeCommand(0.2, 10000, armSubsystem));
             * 
             * addCommands(new MeasuredStraightDriveCommand(30, 0.2, true, driveSubsystem));
             * 
             * 
             * // getting ready to shoot while turning to correct angle
             * 
             * addCommands(new PivotToAngleCommand(0.2, 90, true, 3000, armSubsystem));
             * 
             * addCommands(new TurnToHeadingCommand(0.2, startHeading - 102, true, 3,
             * driveSubsystem));
             * 
             * // second shot
             * 
             * addCommands(new ShooterCommand(0.5, 2000, armSubsystem));
             */

            break;

        case SPEAKER_THREE_SHOT:
            /*
             * Facing Directly towards the speaker, robot backs up to make first shot,
             * Makes a full turn and goes forward to get the last note,
             * full turns again and makes the second shot,
             * then turns left to grab the last note, turns to the speaker
             * and makes the last shot
             */

            /*
             * 
             * startHeading = driveSubsystem.getHeading();
             * 
             * // backs up for first shot (needs space)
             * 
             * addCommands(new MeasuredStraightDriveCommand(100, -0.2, true, driveSubsystem));
             * 
             * addCommands(new PivotToAngleCommand(0.3, 90, true, 5000, armSubsystem));
             * 
             * // first shot
             * 
             * addCommands(new ShooterCommand(0.5, 2000, armSubsystem));
             * 
             * // full turns and goes for the second note
             * 
             * addCommands(new PivotToAngleCommand(0.3, 0, true, 5000, armSubsystem));
             * 
             * addCommands(new TurnToHeadingCommand(0.2, startHeading + 180, true, 3000,
             * driveSubsystem));
             * 
             * addCommands(new IntakeCommand(0.2, 10000, armSubsystem));
             * 
             * addCommands(new MeasuredStraightDriveCommand(100, 0.2, true, driveSubsystem));
             * 
             * // Second shotting.
             * 
             * addCommands(new PivotToAngleCommand(0.3, 90, true, 5000, armSubsystem));
             * 
             * addCommands(new TurnToHeadingCommand(0.2, startHeading + 180, true, 3000,
             * driveSubsystem));
             * 
             * addCommands(new ShooterCommand(0.5, 2000, armSubsystem));
             * 
             * // Third Sucking \(O-o)/
             * addCommands(new PivotToAngleCommand(0.3, 0, true, 5000, armSubsystem));
             * 
             * addCommands(new TurnToHeadingCommand(0.2, startHeading - 90, true, 3000,
             * driveSubsystem));
             * 
             * addCommands(new IntakeCommand(0.2, 10000, armSubsystem));
             * 
             * addCommands(new MeasuredStraightDriveCommand(100, 0.2, true, driveSubsystem));
             * 
             * // Third Shooting :p
             * 
             * addCommands(new TurnToHeadingCommand(0.2, startHeading + 185, true, 3000,
             * driveSubsystem));
             * 
             * addCommands(new PivotToAngleCommand(0.3, 90, true, 5000, armSubsystem));
             * 
             * addCommands(new ShooterCommand(0.5, 2000, armSubsystem));
             * 
             * break;
             */

        case SPEAKER_FOUR_SHOT:
            /*
             * Facing Directly towards the speaker, robot backs up to make first shot,
             * Makes a full turn and goes forward to get the last note,
             * full turns again and makes the second shot,
             * then turns left to grab the 2nd last note, turns to the speaker and shoots
             * then turns again then goes towards last note
             * sucks last note (uwu)
             * and makes the last shot
             */

            startHeading = driveSubsystem.getHeading();

            // backs up for first shot (needs space)

            /*
             * 
             * addCommands(new MeasuredStraightDriveCommand(100, -0.2, true, driveSubsystem));
             * 
             * addCommands(new PivotToAngleCommand(0.3, 90, true, 5000, armSubsystem));
             * 
             * // first shot
             * 
             * addCommands(new ShooterCommand(0.5, 2000, armSubsystem));
             * 
             * // full turns and goes for the second note
             * 
             * addCommands(new PivotToAngleCommand(0.3, 0, true, 5000, armSubsystem));
             * 
             * addCommands(new TurnToHeadingCommand(0.2, startHeading + 180, true, 3000,
             * driveSubsystem));
             * 
             * addCommands(new IntakeCommand(0.2, 10000, armSubsystem));
             * 
             * addCommands(new MeasuredStraightDriveCommand(100, 0.2, true, driveSubsystem));
             * 
             * // Second shotting.
             * 
             * addCommands(new PivotToAngleCommand(0.3, 90, true, 5000, armSubsystem));
             * 
             * addCommands(new TurnToHeadingCommand(0.2, startHeading + 180, true, 3000,
             * driveSubsystem));
             * 
             * addCommands(new ShooterCommand(0.5, 2000, armSubsystem));
             * 
             * // Third Sucking \(O-o)/
             * addCommands(new PivotToAngleCommand(0.3, 0, true, 5000, armSubsystem));
             * 
             * addCommands(new TurnToHeadingCommand(0.2, startHeading - 90, true, 3000,
             * driveSubsystem));
             * 
             * addCommands(new IntakeCommand(0.2, 10000, armSubsystem));
             * 
             * addCommands(new MeasuredStraightDriveCommand(100, 0.2, true, driveSubsystem));
             * 
             * // Third Shooting :p
             * 
             * addCommands(new TurnToHeadingCommand(0.2, startHeading + 185, true, 3000,
             * driveSubsystem));
             * 
             * addCommands(new PivotToAngleCommand(0.3, 90, true, 5000, armSubsystem));
             * 
             * addCommands(new ShooterCommand(0.5, 2000, armSubsystem));
             * 
             * 
             * // Forth Sucking \(>w<)/
             * addCommands(new PivotToAngleCommand(0.3, 0, true, 5000, armSubsystem));
             * 
             * addCommands(new TurnToHeadingCommand(0.2, startHeading + 55, true, 3000,
             * driveSubsystem));
             * 
             * addCommands(new IntakeCommand(0.2, 10000, armSubsystem));
             * 
             * addCommands(new MeasuredStraightDriveCommand(200, 0.2, true, driveSubsystem));
             * 
             * // Fourth shooting powo :
             * 
             * addCommands(new TurnToHeadingCommand(0.2, startHeading - 185, true, 3000,
             * driveSubsystem));
             * 
             * addCommands(new PivotToAngleCommand(0.3, 90, true, 5000, armSubsystem));
             * 
             * addCommands(new ShooterCommand(0.5, 2000, armSubsystem));
             * 
             */

            break;

        case AMP_ONE_SHOT:

            /*
             * 
             * startHeading = driveSubsystem.getHeading();
             * 
             * addCommands(new MeasuredStraightDriveCommand(100, -0.2, true, driveSubsystem));
             * 
             * // Shoot arm, no command made yet
             * 
             * addCommands(new TurnToHeadingCommand(0.2, startHeading + 45, true, 3,
             * driveSubsystem));
             * 
             * addCommands(new MeasuredStraightDriveCommand(100, -0.2, true, driveSubsystem));
             * 
             * break;
             */

        case AMP_TWO_SHOT:
            /*
             * startHeading = driveSubsystem.getHeading();
             * 
             * addCommands(new MeasuredStraightDriveCommand(100, -0.2, true, driveSubsystem));
             * 
             * // Shoot arm, no command made yet
             * 
             * addCommands(new TurnToHeadingCommand(0.2, startHeading + 45, true, 3,
             * driveSubsystem));
             * 
             * addCommands(new MeasuredStraightDriveCommand(200, -0.2, true, driveSubsystem));
             * 
             * addCommands(new TurnToHeadingCommand(0.2, startHeading + 22.5, true, 3,
             * driveSubsystem));
             * 
             * // Shoot arm, no command made yet
             */

            break;

        case AMP_ONE_SHOT_ONE_AMP:
            /*
             * startHeading = driveSubsystem.getHeading();
             * 
             * addCommands(new MeasuredStraightDriveCommand(100, -0.2, true, driveSubsystem));
             * 
             * // Shoot arm, no command made yet
             * 
             * addCommands(new TurnToHeadingCommand(0.2, startHeading + 45, true, 3,
             * driveSubsystem));
             * 
             * addCommands(new MeasuredStraightDriveCommand(200, -0.2, true, driveSubsystem));
             * 
             * addCommands(new TurnToHeadingCommand(0.2, startHeading + 135, true, 3,
             * driveSubsystem));
             * 
             * addCommands(new MeasuredStraightDriveCommand(100, 0.2, true, driveSubsystem));
             * 
             * // Shoot arm into amp, no command made yet
             */

            break;

        case AMP_ONE_AMP:
            /*
             * startHeading = driveSubsystem.getHeading();
             * 
             * addCommands(new MeasuredStraightDriveCommand(100, -0.2, true, driveSubsystem));
             * 
             * addCommands(new TurnToHeadingCommand(0.2, startHeading + 31, true, 3,
             * driveSubsystem));
             * 
             * addCommands(new MeasuredStraightDriveCommand(206, -0.2, true, driveSubsystem));
             * 
             * addCommands(new TurnToHeadingCommand(0.2, startHeading + 135, true, 3,
             * driveSubsystem));
             * 
             * addCommands(new MeasuredStraightDriveCommand(50, 0.2, true, driveSubsystem));
             * 
             * // Shoot arm into amp, no command made yet
             */

            break;

        case AMP_TWO_AMP:

            /*
             * /
             * startHeading = driveSubsystem.getHeading();
             * 
             * addCommands(new MeasuredStraightDriveCommand(100, -0.2, true, driveSubsystem));
             * 
             * addCommands(new TurnToHeadingCommand(0.2, startHeading + 31, true, 3,
             * driveSubsystem));
             * 
             * addCommands(new MeasuredStraightDriveCommand(206, -0.2, true, driveSubsystem));
             * 
             * addCommands(new TurnToHeadingCommand(0.2, startHeading + 135, true, 3,
             * driveSubsystem));
             * 
             * addCommands(new MeasuredStraightDriveCommand(50, 0.2, true, driveSubsystem));
             * 
             * // Shoot arm into amp, no command made yet
             * 
             * addCommands(new MeasuredStraightDriveCommand(100, -0.2, true, driveSubsystem));
             * 
             * // Shoot arm into amp, no command made yet
             */

            break;

        }
    }
}

