package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.IntakeArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeUpCommand extends ParallelDeadlineGroup {
    public IntakeUpCommand(IntakeSubsystem intake, IntakeArmSubsystem intakeArm) {
        super(
            // End when the arm reports it has reached deployed position
            new WaitUntilCommand(intakeArm::isUp),

            // Keep commanding the arm toward deployed until deadline is met
            intakeArm.goUp(),

            // Spin intake slowly while deploying, then stop automatically
            new StartEndCommand(
                () -> intake.RunIntake(-1),
                intake::stop,
                intake
            )
        );
    }
}