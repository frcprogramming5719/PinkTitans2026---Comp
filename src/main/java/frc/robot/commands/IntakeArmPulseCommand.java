package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeArmPulseCommand extends Command {
    private final IntakeArmSubsystem intakeArm;
    private final IntakeSubsystem intake;
    private final double switchIntervalSec;
    private final double slowIntakeSpeed;

    private boolean goDown = true;
    private double lastSwitchTime = 0.0;

    public IntakeArmPulseCommand(
        IntakeArmSubsystem intakeArm,
        IntakeSubsystem intake,
        double switchIntervalSec,
        double slowIntakeSpeed
    ) {
        this.intakeArm = intakeArm;
        this.intake = intake;
        this.switchIntervalSec = switchIntervalSec;
        this.slowIntakeSpeed = slowIntakeSpeed;

        addRequirements(intakeArm, intake);
    }

    @Override
    public void initialize() {
        lastSwitchTime = Timer.getFPGATimestamp();
        goDown = true;

        intakeArm.moveFeedDown();
        intake.stop();
    }

    @Override
    public void execute() {
        double now = Timer.getFPGATimestamp();

        if (now - lastSwitchTime >= switchIntervalSec) {
            goDown = !goDown;
            lastSwitchTime = now;

        }

        if (goDown) {
            // intakeArm.moveFeedDown();
            // intake.stop();
            
            intakeArm.moveFeedUp();
            intake.RunIntake(slowIntakeSpeed);
        } else {
            intakeArm.moveFeedUp();
            intake.RunIntake(slowIntakeSpeed);
        }
    }

    @Override
    public void end(boolean interrupted) {
        intakeArm.stop();
        intake.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}