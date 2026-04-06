package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;

import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.*;
import frc.robot.ShooterCalculator;
import frc.robot.ShooterCalculator.ShooterState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;

public class HoodSubsystem extends SubsystemBase {
    public double reqAngle;
    private int id;
    // ================= Hardware =================
    private TalonFXS hoodMotor;
    private final CANcoder hoodAbsoluteEncoder = new CANcoder(43);
    public double cur = 0;
    // ================= Control Request =================
    private final MotionMagicVoltage motionMagicRequest =
            new MotionMagicVoltage(0).withSlot(0);

    // ================= Constants =================
    private static final double HOOD_GEAR_RATIO = 8.1481;

    // Hood range (degrees)
    private double MIN_ANGLE = 0;
    private double MAX_ANGLE = 1;
    public static double inches = 0;

    private final PositionVoltage request = new PositionVoltage(0).withSlot(0);
    public HoodSubsystem(int mid) {
        hoodMotor = new TalonFXS(mid);
        id = mid;
        // ================= TalonFX Config =================
        TalonFXSConfiguration config = new TalonFXSConfiguration();

        // ---- Feedback ----
        config.ExternalFeedback.FeedbackRemoteSensorID = hoodAbsoluteEncoder.getDeviceID();
        config.ExternalFeedback.ExternalFeedbackSensorSource = ExternalFeedbackSensorSourceValue.RemoteCANcoder;
        config.ExternalFeedback.SensorToMechanismRatio = 1;
        config.ExternalFeedback.RotorToSensorRatio = HOOD_GEAR_RATIO;

        // ---- PID ----
        config.Slot0.kP = 9;
        config.Slot0.kI = 1.3;
        config.Slot0.kD = 0;
        config.Slot0.kS = 0;
        config.Slot0.kV = 0;
        config.Slot0.kA = 0;

        // Gravity compensation
        config.Slot0.kG = 0;

        // ---- Motion Magic ----

        // ---- Soft Limits (mechanism rotations) ----
        double minRot = MIN_ANGLE;
        double maxRot = MAX_ANGLE;

        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = maxRot;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = minRot;

        // ---- Current Limits ----
        config.CurrentLimits.StatorCurrentLimit = 40;
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        // ---- Voltage Limits ----
        config.Voltage.PeakForwardVoltage = 4;
        config.Voltage.PeakReverseVoltage = -4;

        // ---- Ramps ----
        config.OpenLoopRamps.VoltageOpenLoopRampPeriod = 1;
        config.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 1;

        // ---- Output ----
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        if (id == 31)
            config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        else
            config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;

        SmartDashboard.putString(id + " HOOD CONFIG",hoodMotor.getConfigurator().apply(config).toString());
        //seedMotorFromAbsolute();
    }

    // =========================================================
    // ================== Seeding Function =====================
    // =========================================================
    private void seedMotorFromAbsolute() {

        double absoluteRot = hoodAbsoluteEncoder.getAbsolutePosition().getValueAsDouble();
        hoodMotor.setPosition(absoluteRot);

        SmartDashboard.putNumber(id + " Hood Seeded Rotations", absoluteRot);
    }

    // =========================================================
    // ================== Control Methods ======================
    // =========================================================

    public void setAngleDegrees(double angleDegrees) {
        angleDegrees = clamp(angleDegrees, MIN_ANGLE, MAX_ANGLE);
        hoodMotor.setControl(request.withPosition(angleDegrees));
    }

    public Command setAngleCommand(double angleDegrees) {
        return runOnce(() -> setAngleDegrees(angleDegrees));
    }
    
    // Disable manual unless testing
    public void setManualPercent(double percent) {
        hoodMotor.set(percent);
    }

    public void stop() {
        hoodMotor.stopMotor();
    }

    // =========================================================
    // ================== Getters ==============================
    // =========================================================

    public double getMotorAngleDegrees() {
        return rotationsToDegrees(
                hoodMotor.getPosition().getValueAsDouble()
        );
    }

    public double getAbsoluteAngleDegrees() {
        return rotationsToDegrees(
                hoodAbsoluteEncoder.getAbsolutePosition().getValueAsDouble()
        );
    }

    public double getMinAngleDegrees() { return MIN_ANGLE; }
    public double getMaxAngleDegrees() { return MAX_ANGLE; }
    public double getAngleRangeDegrees() { return MAX_ANGLE - MIN_ANGLE; }

    public double getMinMotorRotations() {
        return degreesToRotations(MIN_ANGLE);
    }

    public Command inc() {
        if (id == 31)
            return runOnce(() -> {cur = cur + 0.01; setAngleDegrees(cur);});
        else
            return runOnce(() -> {cur = cur;});
    }
    public Command dec() {
        if (id == 31)
            return runOnce(() -> {cur = cur - 0.01; setAngleDegrees(cur);});
        else
            return runOnce(() -> {cur = cur;});
    }
    public Command incDist() {
        if (id == 31)
            return runOnce(() -> {inches+=5;});
        else
            return runOnce(() -> {inches = inches;});
    }
    public Command decDist() {
        if (id == 31)
            return runOnce(() -> {inches-=5;});
        else
            return runOnce(() -> {inches = inches;});
    }

    public Command autoAngle(double d) {
        return runOnce(() -> {
            ShooterState state = ShooterCalculator.calculateLookup(d);
            reqAngle = state.angle;
            setAngleDegrees(reqAngle);
        });
    } 

    public double getMaxMotorRotations() {
        return degreesToRotations(MAX_ANGLE);
    }

    // =========================================================
    // ================== Utility ==============================
    // =========================================================

    private double degreesToRotations(double degrees) {
        return Units.degreesToRadians(degrees) / (2.0 * Math.PI);
    }

    private double rotationsToDegrees(double rotations) {
        return rotations * 360.0;
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    public Command setZero() {
        return runOnce(() -> { cur = 0; hoodAbsoluteEncoder.setPosition(0);});
    }
    // =========================================================
    // ================== Periodic =============================
    // =========================================================

    @Override
    public void periodic() {
    
        SmartDashboard.putNumber(id + " cur", cur);
        SmartDashboard.putNumber(id + " Hood Motor Angle (deg)", getMotorAngleDegrees());
        SmartDashboard.putNumber(id + " Hood Absolute Angle (deg)", getAbsoluteAngleDegrees());
        SmartDashboard.putNumber(id + " Hood Velocity", 
                hoodMotor.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber(id + " Hood Motor Position (deg)", hoodMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber(id + " dist", inches);
     //   ShooterState state = ShooterCalculator.calculate(NewTurretSubsystem.distance);
     //   reqAngle = state.angle;

    }
}