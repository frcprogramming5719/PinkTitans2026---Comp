package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.commands.IntakeArmPulseCommand;


public class ShootCommand extends ParallelCommandGroup {
    public ShootCommand(
        ShooterSubsystem shooter,
        HopperSubsystem hopper,
        IntakeArmSubsystem intakeArm,
        IntakeSubsystem intake,

        double shooterSpeed,
        double hopperVolts,
        double kickerVolts
    ) {
        addCommands(
            // Keep shooter at target speed until interrupted
            shooter.runAtSpeedCommand(shooterSpeed),

            // Once shooter is ready, start hopper immediately.
            // Then 3 seconds later, start the intake arm pulsing.
            new SequentialCommandGroup(
                new WaitUntilCommand(() -> shooter.isAtSpeed(shooterSpeed)),

                new ParallelCommandGroup(
                    new StartEndCommand(
                        () -> hopper.activate(hopperVolts, kickerVolts),
                        hopper::deactivate,
                        hopper
                    ),

                    new SequentialCommandGroup(
                        new WaitCommand(1.0),
                        new IntakeUpCommand(intake, intakeArm)
                        //new IntakeArmPulseCommand(intakeArm, intake, 10, 0.4)
                    )
                )
                
            )
        );
    }
}