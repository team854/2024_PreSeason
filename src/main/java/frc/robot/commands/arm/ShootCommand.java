package frc.robot.commands.arm;

import frc.robot.subsystems.ArmSubsystem;

public class ShootCommand extends BaseArmCommand {

    // Speeds
    private double shootSpeed;
    private double pivotSpeed;

    // Logging skibidi
    private double targetAngle;
    private int    counter;

    private long   start = 0;


    public ShootCommand(double shootSpeed, ArmSubsystem armSubsystem) {

        super(armSubsystem);

        this.shootSpeed = shootSpeed;
    }

    @Override
    public void initialize() {

        String commandParms = "shoot speed: " + shootSpeed;
        logCommandStart(commandParms);

        counter = 0;

        super.initialize();

        targetAngle = armSubsystem.getAnglePivot();

    }

    @Override
    public void execute() {

        moveToTargetAngle(targetAngle);

        armSubsystem.setShooterSpeed(shootSpeed);
        counter++;

        if (counter >= 20) {

            armSubsystem.setKeeperSpeed(shootSpeed);
            counter = 0;
        }
    }

    @Override
    public boolean isFinished() {

        if (!armSubsystem.isLoaded()) {
            double finish = System.currentTimeMillis();

            if ((finish - start > 500) && (start != 0)) {
                setFinishReason("sensor intook");
                return true;
            }
            if (start == 0) {

                start = System.currentTimeMillis();
            }


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
