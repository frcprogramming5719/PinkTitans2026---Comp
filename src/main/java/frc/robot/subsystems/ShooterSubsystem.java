package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
    private TalonFX Shooter1;
    private TalonFX Shooter2;
    private TalonFX ShooterB1;
    private TalonFX ShooterB2;

    private TalonFXS Hood1;
    private TalonFXS Hood2;

    private CANcoder HoodCANCoder1;
    private CANcoder HoodCANCoder2;

    private VelocityVoltage velocityVoltage;
    private DutyCycleOut dutyCycleOut;

    private double targetShooterSpeed = 0.0;

    // tune this
    private static final double SHOOTER_SPEED_TOLERANCE = 15.0;


    public ShooterSubsystem() {
        Shooter1 = new TalonFX(Constants.Shooter1_id);
        Shooter2 = new TalonFX(Constants.Shooter2_id);

        Hood1 = new TalonFXS(Constants.Hood1_id);
        HoodCANCoder1 = new CANcoder(Constants.Hood1CANCoder_id);

        ShooterB1 = new TalonFX(Constants.ShooterB1_id);
        ShooterB2 = new TalonFX(Constants.ShooterB2_id);

        Hood2 = new TalonFXS(Constants.Hood2_id);
        HoodCANCoder2 = new CANcoder(Constants.Hood2CANCoder_id);

        velocityVoltage = new VelocityVoltage(0).withSlot(0);
        dutyCycleOut = new DutyCycleOut(0);

        TalonFXConfiguration shooter_configLeft = new TalonFXConfiguration();
        TalonFXConfiguration shooter_configRight = new TalonFXConfiguration();

        shooter_configLeft.Slot0.kP = Constants.ShooterLeft_kP;
        shooter_configLeft.Slot0.kI = Constants.ShooterLeft_kI;
        shooter_configLeft.Slot0.kD = Constants.ShooterLeft_kD;
        shooter_configLeft.Slot0.kS = 0.0;
        shooter_configLeft.Slot0.kV = 0.0;
        shooter_configLeft.Slot0.kA = 0.0;
        shooter_configLeft.Slot0.kG = 0.0;

        shooter_configLeft.Voltage.PeakForwardVoltage = Constants.Shooter_PeakForwardVoltage;
        shooter_configLeft.Voltage.PeakReverseVoltage = Constants.Shooter_PeakReverseVoltage;

        shooter_configLeft.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        Shooter1.getConfigurator().apply(shooter_configLeft);
        Shooter2.getConfigurator().apply(shooter_configLeft);

        shooter_configRight.Slot0.kP = Constants.ShooterRight_kP;
        shooter_configRight.Slot0.kI = Constants.ShooterRight_kI;
        shooter_configRight.Slot0.kD = Constants.ShooterRight_kD;
        shooter_configRight.Slot0.kS = 0.0;
        shooter_configRight.Slot0.kV = 0.0;
        shooter_configRight.Slot0.kA = 0.0;
        shooter_configRight.Slot0.kG = 0.0;

        shooter_configRight.Voltage.PeakForwardVoltage = Constants.Shooter_PeakForwardVoltage;
        shooter_configRight.Voltage.PeakReverseVoltage = Constants.Shooter_PeakReverseVoltage;

        shooter_configRight.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        ShooterB1.getConfigurator().apply(shooter_configRight);
        ShooterB2.getConfigurator().apply(shooter_configRight);

        Shooter1.setControl(new Follower(Shooter2.getDeviceID(), MotorAlignmentValue.Aligned));
        ShooterB2.setControl(new Follower(ShooterB1.getDeviceID(), MotorAlignmentValue.Aligned));
    }

    public void runShooter(double speed) {
        targetShooterSpeed = speed;
        Shooter2.setControl(velocityVoltage.withVelocity(speed));
        ShooterB1.setControl(velocityVoltage.withVelocity(-speed));
        
    }

    public void runShooterDuty(double duty) {
        Shooter2.setControl(dutyCycleOut.withOutput(duty));
        ShooterB1.setControl(dutyCycleOut.withOutput(-duty));
    }

    public void stopShooter() {
        targetShooterSpeed = 0.0;
        Shooter2.stopMotor();
        ShooterB1.stopMotor();
    }

    public Command runAtDuty(DoubleSupplier duty) {
        return run(() -> runShooterDuty(duty.getAsDouble()));
    }

    public Command runAtSpeedCommand(double speed) {
        return runEnd(
            () -> runShooter(speed),
            this::stopShooter
        );
    }

    private boolean motorAtSpeed(TalonFX motor, double targetSpeed) {
        return Math.abs(Math.abs(motor.getVelocity().getValueAsDouble()) - Math.abs(targetSpeed))
            <= SHOOTER_SPEED_TOLERANCE;
    }

    public boolean isAtSpeed(double targetSpeed) {
        return motorAtSpeed(Shooter2, targetSpeed)
            && motorAtSpeed(ShooterB1, targetSpeed);
    }

    public boolean isAtTargetSpeed() {
        return isAtSpeed(targetShooterSpeed);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter/Shooter1/Velocity", Shooter1.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Shooter/Shooter2/Velocity", Shooter2.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Shooter/ShooterB1/Velocity", ShooterB1.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Shooter/ShooterB2/Velocity", ShooterB2.getVelocity().getValueAsDouble());

        SmartDashboard.putNumber("Shooter/Shooter1/Voltage", Shooter1.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Shooter/ShooterB1/Voltage", ShooterB1.getMotorVoltage().getValueAsDouble());


        SmartDashboard.putNumber("Shooter/TargetSpeed", targetShooterSpeed);
        SmartDashboard.putBoolean("Shooter/AtTargetSpeed", isAtTargetSpeed());

        
    }
}