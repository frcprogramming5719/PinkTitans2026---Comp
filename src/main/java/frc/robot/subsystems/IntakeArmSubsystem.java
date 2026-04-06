package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeArmSubsystem extends SubsystemBase {
    private final TalonFX intakeMotor;
    private final int id;

    // Gear ratio: motor rotations per mechanism rotation
    private static final double INTAKE_GEAR_RATIO = 8.1481;

    // Two intake positions in DEGREES of the mechanism
    // Change these to your real values after testing
    private static final double STOWED_ANGLE_DEG = 0.0;
    private static final double MID_ANGLE_DEG = 0.6;

    private static final double DEPLOYED_ANGLE_DEG = 3.3;

    // Safety limits in mechanism degrees
    private static final double MIN_ANGLE_DEG = 0.0;
    private static final double MAX_ANGLE_DEG = 3.5;

    private final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0).withSlot(0);

    public IntakeArmSubsystem() {
        this.id = 19;
        this.intakeMotor = new TalonFX(19);


        TalonFXConfiguration config = new TalonFXConfiguration();

        // Motor sensor -> mechanism scaling
        // This makes Position requests and getPosition() use mechanism rotations


        // PID
        config.Slot0.kP = 40.0;
        config.Slot0.kI = 0;
        config.Slot0.kD = 0;
        config.Slot0.kS = 0.0;
        config.Slot0.kV = 0.0;
        config.Slot0.kA = 0.0;
        config.Slot0.kG = 0.0;

config.Voltage.PeakForwardVoltage = 10;
config.Voltage.PeakReverseVoltage = -10;

config.MotionMagic.MotionMagicCruiseVelocity = 1.5;
config.MotionMagic.MotionMagicAcceleration = 20.0;

        // Soft limits in MECHANISM ROTATIONS


        // Voltage limits


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

    /*
     * Sets the current physical intake position to STOWED_ANGLE_DEG.
     * Use this only when the arm is actually stowed.
     */
    public void zeroToStowed() {
        intakeMotor.setPosition(STOWED_ANGLE_DEG);
        SmartDashboard.putString("IntakeArm/ZeroStatus", "Zeroed to stowed");
    }

        public void zeroToDeployed() {
        intakeMotor.setPosition(DEPLOYED_ANGLE_DEG);
        SmartDashboard.putString("IntakeArm/ZeroStatus", "Zeroed to stowed");
    }

    /**
     * If you want to zero to any arbitrary known mechanism angle.
     */
 
private static final double POSITION_TOLERANCE = 0.5;

public boolean isAtStowed() {
    return Math.abs(getMotorPositionRotations() - STOWED_ANGLE_DEG) <= POSITION_TOLERANCE;
}

public boolean isDown() {
    return Math.abs(getMotorPositionRotations() - DEPLOYED_ANGLE_DEG) <= POSITION_TOLERANCE;
}
public boolean isUp() {
    return Math.abs(getMotorPositionRotations() - MID_ANGLE_DEG) <= POSITION_TOLERANCE;
}

public void moveFeedUp() {
    setAngleDegrees(0.5);   // MID_ANGLE_DEG
}

public void moveFeedDown() {
    setAngleDegrees(3.3);   // DEPLOYED_ANGLE_DEG
}


    public void setAngleDegrees(double angleDegrees) {
        double clampedDeg = clamp(angleDegrees, MIN_ANGLE_DEG, MAX_ANGLE_DEG);
        intakeMotor.setControl(positionRequest.withPosition(clampedDeg));
    }

    public Command setAngleCommand(double angleDegrees) {
        return run(() -> setAngleDegrees(angleDegrees));
            
    }

    public Command goDown() {
        return setAngleCommand(DEPLOYED_ANGLE_DEG);
    }

    public Command goUp() {
        return setAngleCommand(MID_ANGLE_DEG);
    }

    public void setManualPercent(double percent) {
        intakeMotor.set(percent);
    }

    public void stop() {
        intakeMotor.setControl(new VoltageOut(0));
    }

    public double getMotorAngleDegrees() {
        return intakeMotor.getPosition().getValueAsDouble();
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
        SmartDashboard.putNumber("IntakeArm/" + id + "/AngleDeg", getMotorAngleDegrees());
        SmartDashboard.putNumber("IntakeArm/" + id + "/PositionRot", getMotorPositionRotations());
        SmartDashboard.putNumber("IntakeArm/" + id + "/Velocity", getVelocity());

        SmartDashboard.putNumber("IntakeArm/StowedAngleDeg", STOWED_ANGLE_DEG);
        SmartDashboard.putNumber("IntakeArm/DeployedAngleDeg", DEPLOYED_ANGLE_DEG);

        SmartDashboard.putBoolean("IntakeArm/IsStowed", isAtStowed());
SmartDashboard.putBoolean("IntakeArm/IsDown", isDown());
SmartDashboard.putBoolean("IntakeArm/IsUp", isUp());


    }
}