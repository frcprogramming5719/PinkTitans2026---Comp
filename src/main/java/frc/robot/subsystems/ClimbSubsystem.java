package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbSubsystem extends SubsystemBase {
    private final TalonFXS intakeMotor;

    // Gear ratio: motor rotations per mechanism rotation
    private static final double INTAKE_GEAR_RATIO = 8.1481;

    // Two intake positions in DEGREES of the mechanism
    // Change these to your real values after testing
    private static final double STOWED_ANGLE_DEG = 0.0;
    private static final double DEPLOYED_ANGLE_DEG = 85.0;

    // Safety limits in mechanism degrees
    private static final double MIN_ANGLE_DEG = 0.0;
    private static final double MAX_ANGLE_DEG = 100.0;

    private final PositionVoltage positionRequest = new PositionVoltage(0).withSlot(0);

    public ClimbSubsystem() {
        this.intakeMotor = new TalonFXS(16);

        TalonFXSConfiguration config = new TalonFXSConfiguration();

        // Motor sensor -> mechanism scaling
        // This makes Position requests and getPosition() use mechanism rotations
        config.ExternalFeedback.SensorToMechanismRatio = 1.0;
        config.ExternalFeedback.RotorToSensorRatio = INTAKE_GEAR_RATIO;

        // PID
        config.Slot0.kP = 25.0;
        config.Slot0.kI = 0.0;
        config.Slot0.kD = 0.5;
        config.Slot0.kS = 0.0;
        config.Slot0.kV = 0.0;
        config.Slot0.kA = 0.0;
        config.Slot0.kG = 0.0;

        // Soft limits in MECHANISM ROTATIONS

        // Current limits
        config.CurrentLimits.StatorCurrentLimit = 40;
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        // Voltage limits
        config.Voltage.PeakForwardVoltage = 32;
        config.Voltage.PeakReverseVoltage = -32;

        // Ramps
        config.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.2;
        config.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.2;

        // Brake
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        intakeMotor.getConfigurator().apply(config);

        // IMPORTANT:
        // Only do this if the intake is physically in the STOWED position when robot boots.
        zeroToStowed();
    }

    /**
     * Sets the current physical intake position to STOWED_ANGLE_DEG.
     * Use this only when the arm is actually stowed.
     */
    public void zeroToStowed() {
        intakeMotor.setPosition(degreesToRotations(STOWED_ANGLE_DEG));
        SmartDashboard.putString("IntakeArm/ZeroStatus", "Zeroed to stowed");
    }

    /**
     * If you want to zero to any arbitrary known mechanism angle.
     */
    public void seedToKnownAngle(double knownAngleDeg) {
        intakeMotor.setPosition(degreesToRotations(knownAngleDeg));
        SmartDashboard.putString("IntakeArm/ZeroStatus", "Seeded to known angle");
    }

    public void setAngleDegrees(double angleDegrees) {
        double clampedDeg = clamp(angleDegrees, MIN_ANGLE_DEG, MAX_ANGLE_DEG);
        intakeMotor.setControl(positionRequest.withPosition(degreesToRotations(clampedDeg)));
    }

    public Command setAngleCommand(double angleDegrees) {
        return run(() -> setAngleDegrees(angleDegrees))
            .until(() -> Math.abs(getMotorAngleDegrees() - angleDegrees) < 2.0);
    }

    public Command goToStowed() {
        return setAngleCommand(STOWED_ANGLE_DEG);
    }

    public Command goToDeployed() {
        return setAngleCommand(DEPLOYED_ANGLE_DEG);
    }

    public void setManualPercent(double percent) {
        intakeMotor.set(percent);
    }

    public void stop() {
        intakeMotor.stopMotor();
    }

    public double getMotorAngleDegrees() {
        return rotationsToDegrees(intakeMotor.getPosition().getValueAsDouble());
    }

    public double getMotorPositionRotations() {
        return intakeMotor.getPosition().getValueAsDouble();
    }

    public double getVelocity() {
        return intakeMotor.getVelocity().getValueAsDouble();
    }

    public Command rezeroToStowedCommand() {
        return runOnce(this::zeroToStowed);
    }

    private double degreesToRotations(double degrees) {
        return degrees / 360.0;
    }

    private double rotationsToDegrees(double rotations) {
        return rotations * 360.0;
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }


    public void RunIntake(double curVolts){
        intakeMotor.setControl(new VoltageOut(curVolts));
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("IntakeArm/AngleDeg", getMotorAngleDegrees());
        SmartDashboard.putNumber("IntakeArm/PositionRot", getMotorPositionRotations());
        SmartDashboard.putNumber("IntakeArm/Velocity", getVelocity());

        SmartDashboard.putNumber("IntakeArm/StowedAngleDeg", STOWED_ANGLE_DEG);
        SmartDashboard.putNumber("IntakeArm/DeployedAngleDeg", DEPLOYED_ANGLE_DEG);
    }
}