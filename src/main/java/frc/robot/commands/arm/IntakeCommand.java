package frc.robot.commands.arm;

import frc.robot.Constants.ArmConstants;
import frc.robot.operator.OperatorInput;
import frc.robot.subsystems.ArmSubsystem;

public class IntakeCommand extends BaseArmCommand {

    final OperatorInput operatorInput;
    long                start;
    long                finish;
    boolean             isAuto;

    public IntakeCommand(OperatorInput operatorInput, ArmSubsystem armSubsystem, boolean isAuto) {
        super(armSubsystem);
        this.operatorInput = operatorInput;
        this.isAuto        = isAuto;
    }

    @Override
    public void initialize() {
        logCommandStart();
        super.initialize();
        start = 0;
    }

    @Override
    public void execute() {
        boolean atTarget = moveToTargetAngle(ArmConstants.INTAKE_ANGLE);

        if (atTarget) {
            armSubsystem.intakeSetSpeed(ArmConstants.INTAKE_SPEED);
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
                    // Added code to trigger vibration when loaded
                    operatorInput.driverController.pulseRumble(1.0, 0.2); // Full intensity, 0.2
                                                                          // seconds duration per
                                                                          // pulse
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
    }
}