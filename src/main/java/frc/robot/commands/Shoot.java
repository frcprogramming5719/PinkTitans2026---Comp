package frc.robot.commands;

import edu.wpi.first.units.measure.Angle;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ShooterSubsystem;



public class Shoot extends SequentialCommandGroup {
    public Shoot(HopperSubsystem Hopper, IntakeSubsystem Intake, ShooterSubsystem shoot, HopperSubsystem hopper){
    addCommands(
            new ParallelRaceGroup(

            // new InstantCommand(() -> {
            //     Hopper.deactivateCommand(); 
            //     Intake.stop();
            // }));        
                // Hopper.activateCommand();

                new WaitCommand(8.14)
            ),
            new InstantCommand(() -> {
                Hopper.deactivateCommand(); 
                Intake.stop();
            }));
    }
}
