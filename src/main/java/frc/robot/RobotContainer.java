// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
// If you are using Command-Based programming, you will also need:
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.limelight;
import frc.robot.subsystems.ShooterSubsystem;
//import frc.robot.subsystems.NewTurretSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.IntakeArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

import frc.robot.commands.IntakeCommand;
import frc.robot.commands.IntakeDeployCommand;
import frc.robot.commands.IntakeFeedCommand;
import frc.robot.commands.IntakeUpCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.RotateToTag;
import frc.robot.commands.IntakeArmPulseCommand;



public class RobotContainer {
  // near zero velocity
    public double cur = 0;
    public double curAngle = 0;
    public boolean oneHanded = false;
    public boolean shootingL = false;
    public boolean shootingR = false;
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    public boolean shooting = false;
    public boolean intaking = false;

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final HopperSubsystem hopper = new HopperSubsystem();
    private final Telemetry logger = new Telemetry(MaxSpeed);
    public final ShooterCalculator shooterCalc = new ShooterCalculator();
    private final CommandXboxController joystick = new CommandXboxController(0);
    private final CommandXboxController joystick_debug = new CommandXboxController(1);
    private final ShooterSubsystem shooter = new ShooterSubsystem();
    public final HoodSubsystem hood = new HoodSubsystem(31);
    public final HoodSubsystem hood2 = new HoodSubsystem(33);
    public final IntakeSubsystem intake = new IntakeSubsystem();
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
   //    public final NewTurretSubsystem newturret = new NewTurretSubsystem();
    //     public final ShooterSubsystem Shooter = new ShooterSubsystem(23);
    //  public final ShooterSubsystem Shooter2 = new ShooterSubsystem(27);
    public boolean autoaim = false;
    double min = -179;
    int cycleCount = 0;
      //public double degrees = hood.getAngleDegrees();



        public final ClimbSubsystem Climber = new ClimbSubsystem();
        public final IntakeArmSubsystem IntakeArm = new IntakeArmSubsystem();

    private final limelight limelightback = new limelight("limelight-back");

    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        CommandXboxController joystick = new CommandXboxController(0);

        XboxController driverJoystick = new XboxController(0);
        autoChooser = AutoBuilder.buildAutoChooser("ShootAndClimb");
        SmartDashboard.putData("Auto Mode", autoChooser);

        configureBindings();

        // Warmup PathPlanner to avoid Java pauses
        FollowPathCommand.warmupCommand().schedule();

        // NamedCommands.registerCommand("ShootCommand", new ShootCommand(newturret, Shooter, hood, IntakeArm, drivetrain, hopper, intake));
        // NamedCommands.registerCommand("IntakeCommand", new ShootCommand(newturret, Shooter, hood, IntakeArm, drivetrain, hopper, intake));

    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.




        drivetrain.setDefaultCommand(
        //     // Drivetrain will execute this command periodically

             drivetrain.applyRequest(() ->
                 drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                     .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                     .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
             )
            
        );
        

      //shooter.setDefaultCommand(shooter.runAtDuty(() -> joystick_debug.getLeftY()));
      joystick_debug.button(2).onTrue( new InstantCommand( ()-> shooter.runShooter(30)));
      joystick_debug.button(2).onFalse( new InstantCommand( ()-> shooter.stopShooter()));
        joystick_debug.button(1).onTrue(IntakeArm.rezeroToStowedCommand());
              joystick_debug.button(3).onTrue( new InstantCommand( ()-> shooter.runShooter(50)));
      joystick_debug.button(3).onFalse( new InstantCommand( ()-> shooter.stopShooter()));
        //  joystick_debug.button(2).onTrue(IntakeArm.goUp());
        //  joystick_debug.button(3).onTrue(IntakeArm.goDown());


   //   joystick_debug.button(2).onTrue(hopper.activateCommand(-12, -8));
    //  joystick_debug.button(2).onFalse(hopper.deactivateCommand());


      // joystick_debug.button(2).onFalse(new IntakeCommand(intake, IntakeArm));

   //   joystick_debug.button(5).onTrue(new IntakeDeployCommand(intake, IntakeArm));
    //  joystick_debug.button(6).onTrue(new IntakeUpCommand(intake, IntakeArm));

          joystick_debug.button(5).onTrue(new InstantCommand(() -> IntakeArm.zeroToDeployed())); 

            joystick_debug.button(4).onTrue(new IntakeUpCommand(intake, IntakeArm));


      joystick.button(6).whileTrue(new IntakeFeedCommand(intake, IntakeArm));



            joystick.button(5).whileTrue(new ShootCommand(shooter, hopper, IntakeArm, intake, 50, -13, -8));
      joystick.button(1).whileTrue(new RotateToTag(drivetrain, limelightback));

      // joystick.button(6).onFalse(new InstantCommand(() -> intake.RunIntake(0)));


   //     turret.setDefaultCommand(turret.moveAtVoltage(() -> 2*joystick.getLeftTriggerAxis()) );

   //     joystick.button(4).onFalse(new InstantCommand(() -> Shooter.stopOutTake()));
     //  joystick.button(5).onTrue(new InstantCommand(() -> Shooter.runOutTake(90, 90) ));

    //joystick.button(4).onTrue(hood.setAngleCommand(1)); 



 
 


    //   joystick.button(2).onTrue(new InstantCommand(() -> Shooter.runOutTake(100, 100)));
    //   joystick.button(3).onTrue(new InstantCommand(() -> Shooter.runOutTake(250, 250)));
    //joystick.button(2).onTrue(new ShootCommand(newturret, Shooter, hood, IntakeArm, drivetrain, hopper, intake));


            // joystick.button(2).onTrue(new InstantCommand(() -> {
     //intake.RunIntake(-4);
//IntakeArm.RunIntake(-0.2);}));


  // joystick.button(2).negate().and(joystick.rightTrigger().negate().and(joystick.button(3).negate())).onTrue(new InstantCommand(() -> {intake.RunIntake(0); hopper.activate(0, 0);}));
   // joystick.rightTrigger().onTrue(new InstantCommand(() -> {intake.RunIntake(4); hopper.activate(8, 6);}));
    


     //   joystick.button(3).whileTrue(new RunCommand(() -> {
       //     if (autoaim) {
            //newturret.aimToDynamicAngleUntilNear(drivetrain::getTurretTargetAngle);
            //hood.autoAngle(drivetrain.getHoodTargetDeg());
            //Shooter.autorunOutTake(drivetrain.getHoodTargetDeg());
         //   }
           // else {
               // Shooter.runOutTake(-50,-50);
            //}
            //new WaitCommand(0.5);
            //intake.RunIntake(-4);
          //  hopper.activate(0, -6);
           // cycleCount++;
          //  if (cycleCount >= 25 && cycleCount <= 26) {
         //       hopper.activate(0, 6);
        //    }
       //     if (cycleCount > 26) {
     //           cycleCount = 0;
   //         }
 //   }));
   // joystick.button(3).onFalse(new InstantCommand(() -> {
    //    Shooter.stopOutTake();
     //   intake.stop();
      //  hopper.deactivate();
       // cycleCount = 0;
//    }));


  //  joystick.button(3).onFalse(new InstantCommand(() -> intake.RunIntake(0)));

   //     joystick.button(2).onTrue(new InstantCommand(() -> (2)));

    // joystick.button(1).onTrue(hood.setAngleCommand(0.1)); 
    // joystick.button(2).onTrue(hood.setAngleCommand(0.33));
    // joystick.button(3).onTrue(hood.setAngleCommand(0.66));
    // joystick.button(4).onTrue(hood.setAngleCommand(0));
    //joystick.povUp().onTrue(hood.inc());
    //joystick.povDown().onTrue(hood.dec());
     // joystick.povUp().onTrue(hood.incDist());
    //joystick.povDown().onTrue(hood.decDist());
    //joystick.rightBumper().onTrue(hood.setZero());
    




    
  //  joystick.button(2).onTrue(new InstantCommand (() -> {Shooter.input += 5;}));
  //  joystick.button(3).onTrue(new InstantCommand (() -> {Shooter.input -= 5;}));
 //   joystick.button(1).onTrue(new InstantCommand (() -> {IntakeArm.RunIntake(1.5);}));
//     joystick.button(4).onTrue(new InstantCommand (() -> {IntakeArm.RunIntake(-1.5);}));
//     joystick.povRight().onTrue(new InstantCommand (() -> {Climber.RunIntake(joystick.getLeftTriggerAxis()*12);}));
//     joystick.povLeft().onTrue(new InstantCommand (() -> {Climber.RunIntake(-12);;}));
//     joystick.povRight().negate().and(joystick.povLeft().negate()).whileTrue(new InstantCommand (() -> {Climber.RunIntake(0);}));
//     joystick.button(1).negate().and(joystick.button(4).negate()).whileTrue(new InstantCommand(() -> {
//          IntakeArm.stop();
//      }));
//    // joystick.leftTrigger().whileTrue(new InstantCommand(() -> {
    //    hopper.activateCommand(-12);
     //   intake.RunIntake(4);
    //}));
    //joystick.leftTrigger().whileFalse(new InstantCommand(() -> {
    //    hopper.deactivateCommand();
    //    intake.stop();
    //}));
    /*joystick.rightTrigger().whileTrue(new RunCommand(() -> {
        Shooter.runOutTakeCommand();
        //newturret.aimToDynamicAngleUntilNear(drivetrain::getTurretTargetAngle);
        hopper.activateCommand(-12);
        intake.RunIntake(4);
        hopper.hopperFull = false;
    }));*/
    /* 
    joystick.rightTrigger().whileFalse(new InstantCommand(() -> {
        Shooter.stopOutTake();
        intake.stop();
        hopper.deactivateCommand();
    })); */
    //joystick.button(1).onTrue(hood.setAngleDegrees((hood.MAX_ANGLE)-(hood.MAX_ANGLE/2)-(hood.MAX_ANGLE/4)));
    //new JoystickButton(joystick, 8).onTrue((new InstantCommand(drivetrain::resetGyro)));
        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        joystick.back().and(joystick.start()).onTrue(new InstantCommand(() -> {
            oneHanded = !oneHanded;
        }));

        //joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        //joystick.b().whileTrue(drivetrain.applyRequest(() ->
        //    point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        //));
        //joystick.leftBumper().onTrue(new InstantCommand(() -> { hood.zero(hood.getAngleDegrees());}));
        /*joystick.povUp().whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(0.5).withVelocityY(0))
        );
        joystick.povDown().whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(-0.5).withVelocityY(0))
        );*/


        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset the field-centric heading on left bumper press.
        joystick.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));
        joystick.povDown().onTrue(new InstantCommand(() -> {autoaim = !autoaim;
        SmartDashboard.putBoolean("autoaim", autoaim);

        hood.setAngleDegrees(0);}));
        
      //  Shooter.runOutTakeCommand(20, 20); //not sure what values to put here so bleh
      //  
        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }
}
