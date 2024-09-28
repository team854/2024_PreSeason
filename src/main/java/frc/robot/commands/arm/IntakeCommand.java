package frc.robot.commands.arm;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.ArmConstants;
import frc.robot.operator.OperatorInput;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.LightsSubsystem;

public class IntakeCommand extends BaseArmCommand {

    final OperatorInput operatorInput;
    long                start;
    long                finish;
    boolean             isAuto;
    boolean             hasFlashed = false; // To track if the flash has occurred
    LightsSubsystem     lightsSubsystem;    // Reference to LightsSubsystem
    Timer               flashTimer;         // Timer for white flashing

    public IntakeCommand(OperatorInput operatorInput, ArmSubsystem armSubsystem, LightsSubsystem lightsSubsystem,
        boolean isAuto) {
        super(armSubsystem);
        this.operatorInput   = operatorInput;
        this.isAuto          = isAuto;
        this.lightsSubsystem = lightsSubsystem; // Initialize LightsSubsystem
        this.flashTimer      = new Timer();     // Initialize Timer
    }

    @Override
    public void initialize() {
        logCommandStart();
        super.initialize();
        start      = 0;
        hasFlashed = false;
        flashTimer.stop();
        flashTimer.reset();

        // Set lights to orange when starting to intake
        lightsSubsystem.setIntakingColor();
    }

    @Override
    public void execute() {
        boolean atTarget = moveToTargetAngle(ArmConstants.INTAKE_ANGLE);

        if (atTarget) {
            armSubsystem.intakeSetSpeed(ArmConstants.INTAKE_SPEED);
        }

        // Check if the robot has intaken a note
        if (armSubsystem.isLoaded()) {
            if (!hasFlashed) {
                // Start the white flashing effect for 3 seconds
                lightsSubsystem.flashWhite();
                flashTimer.start();
                hasFlashed = true;
            }

            // Flashing effect for 3 seconds
            if (flashTimer.get() > 3.0) {
                // After 3 seconds, set LEDs to green (indicating the robot holds a note)
                lightsSubsystem.setPossessionColor();
                flashTimer.stop();
                flashTimer.reset();
            }
        }
        else {
            // Ensure the LEDs stay orange during the intake process
            lightsSubsystem.setIntakingColor();
        }
    }

    @Override
    public boolean isFinished() {
        if (isAuto) {
            if (isTimeoutExceeded(3) || armSubsystem.isLoaded()) {
                setFinishReason("timeout exceeded");
                return true;
            }
        }
        else {
            if (!operatorInput.isIntake() && isTimeoutExceeded(0.25)) {
                setFinishReason("let go of intake button");
                return true;
            }
            else {
                if (armSubsystem.isLoaded()) {
                    // Trigger vibration when a note is loaded
                    operatorInput.driverController.pulseRumble(1.0, 0.1);
                    finish = System.currentTimeMillis();

                    if ((finish - start > 500) && (start != 0)) {
                        setFinishReason("sensor intook");
                        return true;
                    }
                    if (start == 0) {
                        start = System.currentTimeMillis();
                    }
                }
            }
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        armSubsystem.intakeSetSpeed(0);
        logCommandEnd(interrupted);

        // Optionally, reset lights after intake is done (set to neutral or turn off)
        lightsSubsystem.turnOffLights();
    }
}
