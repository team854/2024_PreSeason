package frc.robot.commands.arm;

import frc.robot.subsystems.ArmSubsystem;

/**
 * Command for shooting from a specific angle defined for long shots.
 */
public class LongShotCommand extends BaseArmCommand {

    private final double shootSpeed;
    private final double targetAngle;

    public LongShotCommand(double shootSpeed, double targetAngle, ArmSubsystem armSubsystem) {
        super(armSubsystem);
        this.shootSpeed  = shootSpeed;
        this.targetAngle = targetAngle;
    }

    @Override
    public void initialize() {
        String commandParams = "shoot speed: " + shootSpeed + ", target angle: " + targetAngle;
        logCommandStart(commandParams);
        super.initialize();
    }

    @Override
    public void execute() {
        boolean atAngle = moveToTargetAngle(targetAngle);
        armSubsystem.setShooterSpeed(shootSpeed);

        if (atAngle) {
            if (isTimeoutExceeded(1.5))
                armSubsystem.setKeeperSpeed(shootSpeed);
        }
    }

    @Override
    public boolean isFinished() {
        if (isTimeoutExceeded(2)) {
            return true;
        }
        else {
            return false;
        }
    }

    @Override
    public void end(boolean interrupted) {
        armSubsystem.intakeSetSpeed(0);
        armSubsystem.pivotRotSetSpeed(0);
        logCommandEnd(interrupted);
    }
}
