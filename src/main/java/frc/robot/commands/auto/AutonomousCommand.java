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
import frc.robot.subsystems.LightsSubsystem; // Import LightsSubsystem

public class AutonomousCommand extends SequentialCommandGroup {

    // Updated constructor with LightsSubsystem
    public AutonomousCommand(DriveSubsystem driveSubsystem,
        SendableChooser<AutoPattern> autoPatternChooser,
        ArmSubsystem armSubsystem,
        OperatorInput operatorInput,
        LightsSubsystem lightsSubsystem) { // Add LightsSubsystem

        // Default is to do nothing.
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

        if (alliance == null) {
            System.out.println("*** ERROR **** null Alliance ");
            return;
        }

        double startHeading;

        /*
         * Compose the appropriate auto commands based on selected auto pattern
         */
        switch (autoPattern) {

        case DO_NOTHING:
        default:
            return;

        case DRIVE_FORWARD:
            addCommands(new IntakeCommand(operatorInput, armSubsystem, lightsSubsystem, true) // Pass
                                                                                              // LightsSubsystem
                .alongWith(new MeasuredStraightDriveCommand(AutoConstants.AmpSideFirstStraightCM,
                    AutoConstants.AmpSideDriveSpeed, true, driveSubsystem)));
            break;

        case DRIVE_FORWARD_PID_TIMED:
            addCommands(new TimedStraightDriveCommand(1000, 1, true, 186, driveSubsystem));
            break;

        case DRIVE_FORWARD_PID_MEASURED:
            addCommands(new MeasuredDriveAtHeadingCommand(100, 0.1, true, 355, driveSubsystem));
            break;

        case TEST_ARM_COMMANDS:
            addCommands(new MeasuredStraightDriveCommand(100, 0.2, true, driveSubsystem));
            addCommands(new PivotToAngleCommand(0.3, 30, 5000, armSubsystem));
            addCommands(new IntakeCommand(operatorInput, armSubsystem, lightsSubsystem, true)); // Pass
                                                                                                // LightsSubsystem
            addCommands(new IntakeCommand(operatorInput, armSubsystem, lightsSubsystem, true)); // Pass
                                                                                                // LightsSubsystem
            break;

        case BLUE_AMP_ONE_SHOT:
            startHeading = driveSubsystem.getHeading();
            addCommands(new PivotShootCommand(AutoConstants.AmpSideShootSpeed,
                AutoConstants.AmpSideShootAngle, AutoConstants.AmpSideTimeoutMS, armSubsystem));
            addCommands(new MeasuredStraightDriveCommand(AutoConstants.AmpSideDiagStepCM,
                AutoConstants.AmpSideDriveSpeed, true, driveSubsystem)
                .alongWith(new IntakeCommand(operatorInput, armSubsystem, lightsSubsystem, true))); // Pass
                                                                                                    // LightsSubsystem
            addCommands(new TurnToHeadingCommand(AutoConstants.AmpSideDriveSpeed,
                startHeading + AutoConstants.AmpSideFirstAngle, true, AutoConstants.AmpSideTimeoutMS, driveSubsystem));
            addCommands(new IntakeCommand(operatorInput, armSubsystem, lightsSubsystem, true)
                .alongWith(new MeasuredStraightDriveCommand(AutoConstants.AmpSideFirstStraightCM,
                    -AutoConstants.AmpSideDriveSpeed, true, driveSubsystem)));
            break;

        case RED_AMP_ONE_SHOT:
            startHeading = driveSubsystem.getHeading();
            addCommands(new PivotShootCommand(AutoConstants.AmpSideShootSpeed,
                AutoConstants.AmpSideShootAngle, AutoConstants.AmpSideTimeoutMS, armSubsystem));
            addCommands(new MeasuredStraightDriveCommand(AutoConstants.AmpSideDiagStepCM,
                AutoConstants.AmpSideDriveSpeed, true, driveSubsystem)
                .alongWith(new IntakeCommand(operatorInput, armSubsystem, lightsSubsystem, true))); // Pass
                                                                                                    // LightsSubsystem
            addCommands(new TurnToHeadingCommand(AutoConstants.AmpSideDriveSpeed,
                startHeading - AutoConstants.AmpSideFirstAngle, true, AutoConstants.AmpSideTimeoutMS, driveSubsystem));
            addCommands(new IntakeCommand(operatorInput, armSubsystem, lightsSubsystem, true)
                .alongWith(new MeasuredStraightDriveCommand(AutoConstants.AmpSideFirstStraightCM,
                    -AutoConstants.AmpSideDriveSpeed, true, driveSubsystem)));
            break;

        case BLUE_OUTSIDE_ONE_SHOT:
            addCommands(new PivotShootCommand(AutoConstants.OutsideSideShooterSpeed,
                AutoConstants.OutsideSideShooterAngle, AutoConstants.OutsideSideTimeoutMS, armSubsystem));
            addCommands(new MeasuredStraightDriveCommand(AutoConstants.OutsideSideDiagStepCM,
                AutoConstants.OutsideSideDriveSpeed, true, driveSubsystem));
            break;

        case RED_OUTSIDE_ONE_SHOT:
            addCommands(new PivotShootCommand(AutoConstants.OutsideSideShooterSpeed,
                AutoConstants.OutsideSideShooterAngle, AutoConstants.OutsideSideTimeoutMS, armSubsystem));
            addCommands(new MeasuredStraightDriveCommand(AutoConstants.OutsideSideDiagStepCM,
                AutoConstants.OutsideSideDriveSpeed, true, driveSubsystem));
            break;

        case RED_SPEAKER_ONE_SHOT:
            addCommands(new PivotShootCommand(AutoConstants.SpeakerSideShootSpeed,
                AutoConstants.SpeakerSideShootAngle, AutoConstants.SpeakerSideTimeoutMS, armSubsystem));
            addCommands(new MeasuredStraightDriveCommand(AutoConstants.SpeakerSideDiagStepCM,
                AutoConstants.SpeakerSideDriveSpeed, true, driveSubsystem));
            break;

        case BLUE_SPEAKER_ONE_SHOT:
            addCommands(new PivotShootCommand(AutoConstants.SpeakerSideShootSpeed,
                AutoConstants.SpeakerSideShootAngle, AutoConstants.SpeakerSideTimeoutMS, armSubsystem));
            addCommands(new MeasuredStraightDriveCommand(AutoConstants.SpeakerSideDiagStepCM,
                AutoConstants.SpeakerSideDriveSpeed, true, driveSubsystem));
            break;

        case BLUE_AMP_TWO_SHOT:
            startHeading = driveSubsystem.getHeading();
            addCommands(new PivotShootCommand(AutoConstants.AmpSideShootSpeed,
                AutoConstants.AmpSideShootAngle, AutoConstants.AmpSideTimeoutMS, armSubsystem));
            addCommands(new MeasuredStraightDriveCommand(AutoConstants.AmpSideDiagStepCM,
                AutoConstants.AmpSideDriveSpeed, true, driveSubsystem)
                .alongWith(new IntakeCommand(operatorInput, armSubsystem, lightsSubsystem, true))); // Pass
                                                                                                    // LightsSubsystem
            addCommands(new TurnToHeadingCommand(AutoConstants.AmpSideDriveSpeed,
                startHeading + AutoConstants.AmpSideFirstAngle, true, AutoConstants.AmpSideTimeoutMS, driveSubsystem));
            addCommands(new MeasuredStraightDriveCommand(AutoConstants.AmpSideFirstStraightCM - 15,
                AutoConstants.AmpSideDriveSpeed, true, driveSubsystem)
                .alongWith(new IntakeCommand(operatorInput, armSubsystem, lightsSubsystem, true))); // Pass
                                                                                                    // LightsSubsystem
            addCommands(new MeasuredStraightDriveCommand(AutoConstants.AmpSideFirstStraightCM + 10,
                -AutoConstants.AmpSideDriveSpeed, true, driveSubsystem));
            addCommands(new TurnToHeadingCommand(AutoConstants.AmpSideDriveSpeed,
                startHeading, true, AutoConstants.AmpSideTimeoutMS, driveSubsystem));
            addCommands(new MeasuredStraightDriveCommand(AutoConstants.AmpSideDiagStepCM,
                -AutoConstants.AmpSideDriveSpeed, true, driveSubsystem));
            addCommands(new PivotShootCommand(AutoConstants.AmpSideShootSpeed,
                AutoConstants.AmpSideShootAngle, AutoConstants.AmpSideTimeoutMS, armSubsystem));
            addCommands(new MeasuredStraightDriveCommand(AutoConstants.AmpSideDiagStepCM,
                -AutoConstants.AmpSideDriveSpeed, true, driveSubsystem));
            addCommands(new TurnToHeadingCommand(AutoConstants.AmpSideDriveSpeed,
                startHeading + AutoConstants.AmpSideFirstAngle, true, AutoConstants.AmpSideTimeoutMS, driveSubsystem));
            addCommands(new MeasuredStraightDriveCommand(AutoConstants.AmpSideFirstStraightCM,
                AutoConstants.AmpSideDriveSpeed, true, driveSubsystem));
            break;

        case RED_AMP_TWO_SHOT:
            startHeading = driveSubsystem.getHeading();
            addCommands(new PivotShootCommand(AutoConstants.AmpSideShootSpeed,
                AutoConstants.AmpSideShootAngle, AutoConstants.AmpSideTimeoutMS, armSubsystem));
            addCommands(new MeasuredStraightDriveCommand(AutoConstants.AmpSideDiagStepCM + 10,
                AutoConstants.AmpSideDriveSpeed, true, driveSubsystem)
                .alongWith(new IntakeCommand(operatorInput, armSubsystem, lightsSubsystem, true))); // Pass
                                                                                                    // LightsSubsystem
            addCommands(new TurnToHeadingCommand(AutoConstants.AmpSideDriveSpeed,
                startHeading - AutoConstants.AmpSideFirstAngle, true, AutoConstants.AmpSideTimeoutMS, driveSubsystem));
            addCommands(new MeasuredStraightDriveCommand(AutoConstants.AmpSideFirstStraightCM,
                AutoConstants.AmpSideDriveSpeed, true, driveSubsystem)
                .alongWith(new IntakeCommand(operatorInput, armSubsystem, lightsSubsystem, true))); // Pass
                                                                                                    // LightsSubsystem
            addCommands(new MeasuredStraightDriveCommand(AutoConstants.AmpSideFirstStraightCM - 15,
                -AutoConstants.AmpSideDriveSpeed, true, driveSubsystem));
            addCommands(new TurnToHeadingCommand(AutoConstants.AmpSideDriveSpeed,
                startHeading - 10, true, AutoConstants.AmpSideTimeoutMS, driveSubsystem));
            addCommands(new MeasuredStraightDriveCommand(AutoConstants.AmpSideDiagStepCM + 50,
                -AutoConstants.AmpSideDriveSpeed, true, driveSubsystem));
            addCommands(new PivotShootCommand(AutoConstants.AmpSideShootSpeed,
                AutoConstants.AmpSideShootAngle, AutoConstants.AmpSideTimeoutMS, armSubsystem));
            addCommands(new MeasuredStraightDriveCommand(AutoConstants.AmpSideDiagStepCM,
                -AutoConstants.AmpSideDriveSpeed, true, driveSubsystem));
            addCommands(new TurnToHeadingCommand(AutoConstants.AmpSideDriveSpeed,
                startHeading - AutoConstants.AmpSideFirstAngle, true, AutoConstants.AmpSideTimeoutMS, driveSubsystem));
            addCommands(new MeasuredStraightDriveCommand(AutoConstants.AmpSideFirstStraightCM,
                AutoConstants.AmpSideDriveSpeed, true, driveSubsystem));
            break;

        case BLUE_SPEAKER_TWO_SHOT:
            addCommands(new PivotShootCommand(AutoConstants.SpeakerSideShootSpeed,
                AutoConstants.SpeakerSideShootAngle, AutoConstants.SpeakerSideTimeoutMS, armSubsystem));
            addCommands(new IntakeCommand(operatorInput, armSubsystem, lightsSubsystem, true)); // Pass
                                                                                                // LightsSubsystem
            addCommands(new MeasuredStraightDriveCommand(AutoConstants.SpeakerSideDiagStepCM,
                AutoConstants.SpeakerSideDriveSpeed, true, driveSubsystem)
                .alongWith(new IntakeCommand(operatorInput, armSubsystem, lightsSubsystem, true))); // Pass
                                                                                                    // LightsSubsystem
            addCommands(new MeasuredStraightDriveCommand(AutoConstants.SpeakerSideDiagStepCM,
                -AutoConstants.SpeakerSideDriveSpeed, true, driveSubsystem));
            addCommands(new PivotShootCommand(AutoConstants.SpeakerSideShootSpeed,
                AutoConstants.SpeakerSideShootAngle, AutoConstants.SpeakerSideTimeoutMS, armSubsystem));
            addCommands(new MeasuredStraightDriveCommand(AutoConstants.SpeakerSideDiagStepCM,
                AutoConstants.SpeakerSideDriveSpeed, true, driveSubsystem));
            break;

        case RED_SPEAKER_TWO_SHOT:
            addCommands(new PivotShootCommand(AutoConstants.SpeakerSideShootSpeed,
                AutoConstants.SpeakerSideShootAngle, AutoConstants.SpeakerSideTimeoutMS, armSubsystem));
            addCommands(new IntakeCommand(operatorInput, armSubsystem, lightsSubsystem, true)); // Pass
                                                                                                // LightsSubsystem
            addCommands(new MeasuredStraightDriveCommand(AutoConstants.SpeakerSideDiagStepCM,
                AutoConstants.SpeakerSideDriveSpeed, true, driveSubsystem)
                .alongWith(new IntakeCommand(operatorInput, armSubsystem, lightsSubsystem, true))); // Pass
                                                                                                    // LightsSubsystem
            addCommands(new MeasuredStraightDriveCommand(AutoConstants.SpeakerSideDiagStepCM,
                -AutoConstants.SpeakerSideDriveSpeed, true, driveSubsystem));
            addCommands(new PivotShootCommand(AutoConstants.SpeakerSideShootSpeed,
                AutoConstants.SpeakerSideShootAngle, AutoConstants.SpeakerSideTimeoutMS, armSubsystem));
            addCommands(new MeasuredStraightDriveCommand(AutoConstants.SpeakerSideDiagStepCM,
                AutoConstants.SpeakerSideDriveSpeed, true, driveSubsystem));
            break;
        }
    }
}
