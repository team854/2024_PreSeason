package frc.robot.commands.arm;

import frc.robot.subsystems.ArmSubsystem;

public class PivotShootCommand extends BaseArmCommand {

    // Speeds
    private double shootSpeed;
    private double pivotSpeed;

    // Logging skibidi
    private double targetAngle;


    public PivotShootCommand(double shootSpeed, double targetAngle, double timeoutMS,
        ArmSubsystem armSubsystem) {

        super(armSubsystem);

        this.shootSpeed  = shootSpeed;
        this.targetAngle = targetAngle;
    }

    @Override
    public void initialize() {

        String commandParms = "shoot speed: " + shootSpeed + "pivot speed: " + pivotSpeed + "target angle: " + targetAngle;
        logCommandStart(commandParms);

        super.initialize();

    }

    @Override
    public void execute() {

        boolean atAngle = moveToTargetAngle(targetAngle);

        armSubsystem.setShooterSpeed(shootSpeed);

        if (atAngle) {
            armSubsystem.setKeeperSpeed(shootSpeed);
        }

    }

    @Override
    public boolean isFinished() {

        if (!armSubsystem.isLoaded()) {
            setFinishReason("Shot note");
            return true;
        }

        return false;

    }

    @Override
    public void end(boolean interrupted) {

        armSubsystem.intakeSetSpeed(0);
        armSubsystem.pivotRotSetSpeed(0);

        logCommandEnd(interrupted);

    }



}
