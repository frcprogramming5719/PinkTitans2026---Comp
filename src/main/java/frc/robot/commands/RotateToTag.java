package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.limelight;



import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.hardware.*;
import com.ctre.phoenix6.signals.*;
import com.ctre.phoenix6.swerve.*;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.*;

import frc.robot.subsystems.CommandSwerveDrivetrain;

public class RotateToTag extends Command {
  int targetTag;
  Double tx, ty, ta;
  CommandSwerveDrivetrain swerve;
  limelight limelight;

  private double targetx;
  private double targety;
  // PIDController forwardController;
  // PIDController strafeController;
  private PIDController thetaController = new PIDController(0.05, 0, 0.001);


  private long startTime;

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric();

  //private final SwerveRequest.ApplyRobotSpeeds drive = new SwerveRequest.ApplyRobotSpeeds();
  private static final double LIMELIGHT_OFFSET = 0;

  public RotateToTag(CommandSwerveDrivetrain swerve, limelight limelight) {
    this.swerve = swerve;
    this.limelight = limelight;

    addRequirements(swerve);
  }

  @Override
  public void initialize() {
    startTime = System.currentTimeMillis();
    thetaController.reset();
    thetaController.setTolerance(0.5);


    getLimelightValues();
  }

  @Override
  public void execute() {
    getLimelightValues();
    printLimelightVal();


    double rotation = thetaController.calculate(limelight.getLimelightTX(), 0);


    if (Math.abs(tx - LIMELIGHT_OFFSET) < 0.5) {
      rotation *= 0.3;
    }
    if (Math.abs(tx - LIMELIGHT_OFFSET) < 0.5) {
      rotation = 0;
    }

    swerve.setControl(

    drive.withVelocityX(0 ) 
    .withVelocityY(0 ) 
    .withRotationalRate(rotation) );
    
  }

 

  @Override
  public void end(boolean interrupted) {
    swerve.setControl(
        drive.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
  }

  @Override
  public boolean isFinished() {
    getLimelightValues(); 
 
    double rotationError = thetaController.calculate(limelight.getLimelightTX(), 0); 
  
    return 
           Math.abs(rotationError) < 0.2;
  }
  
  

  public void getLimelightValues() {
    tx = limelight.getLimelightTX();
    ty = limelight.getLimelightTY();
    ta = limelight.getLimelightTA();
  }

  public void printLimelightVal() {
    getLimelightValues();
    SmartDashboard.putNumber("Limelight tx", tx);
    SmartDashboard.putNumber("Limelight ty", ty);
    SmartDashboard.putNumber("Limelight ta", ta);
  }
}