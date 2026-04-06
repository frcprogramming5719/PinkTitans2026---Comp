package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.subsystems.IntakeArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeFeedCommand extends ConditionalCommand {
    public IntakeFeedCommand(IntakeSubsystem intake, IntakeArmSubsystem intakeArm) {
        super(
            // If already deployed/down -> just run intake
            new StartEndCommand(
                () -> intake.RunIntake(-5.0),   // change to your correct feed speed
                intake::stop,
                intake
            ),

            // If not deployed -> deploy first, then run intake
            new SequentialCommandGroup(
                new IntakeDeployCommand(intake, intakeArm),
                new StartEndCommand(
                    () -> intake.RunIntake(-5.0),   // change to your correct feed speed
                    intake::stop,
                    intake
                )
            ),

            // Condition: true means already down
            intakeArm::isDown
        );
    }
}