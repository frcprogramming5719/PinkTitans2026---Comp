package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.IntakeArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends SequentialCommandGroup {
    public IntakeCommand(IntakeSubsystem intake, IntakeArmSubsystem intakeArm) {
        addCommands(
            new ConditionalCommand(
                // If already down -> just run intake fast
                new StartEndCommand(
                    () -> intake.RunIntake(1.0),
                    intake::stop,
                    intake
                ),

                // If not down -> lower arm + slow intake, then fast intake
                new SequentialCommandGroup(
                    new ParallelDeadlineGroup(
                        new WaitUntilCommand(intakeArm::isDown),
                        intakeArm.goDown(),
                        new StartEndCommand(
                            () -> intake.RunIntake(0.1),
                            intake::stop,
                            intake
                        )
                    ),
                    new StartEndCommand(
                        () -> intake.RunIntake(1),
                        intake::stop,
                        intake
                    )
                ),

                intakeArm::isDown
            )
        );
    }
}