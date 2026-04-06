package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.*;
import frc.robot.ShooterCalculator;
import frc.robot.ShooterCalculator.ShooterState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;

public class IntakeSubsystem extends SubsystemBase {


    private TalonFXS IntakeMotor;

    public IntakeSubsystem() {
        IntakeMotor = new TalonFXS(18);

        // ================= TalonFX Config =================
        TalonFXSConfiguration config = new TalonFXSConfiguration();

        // ---- Feedback ----
        config.ExternalFeedback.ExternalFeedbackSensorSource = ExternalFeedbackSensorSourceValue.RemoteCANcoder;
        config.ExternalFeedback.SensorToMechanismRatio = 1;

        // ---- PID ----
        config.Slot0.kP = 9;
        config.Slot0.kI = 1.3;
        config.Slot0.kD = 0;
        config.Slot0.kS = 0;
        config.Slot0.kV = 0;
        config.Slot0.kA = 0;

        // Gravity compensation
        config.Slot0.kG = 0;

        // ---- Current Limits ----
        config.CurrentLimits.StatorCurrentLimit = 40;
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        // ---- Voltage Limits ----
        config.Voltage.PeakForwardVoltage = 12;
        config.Voltage.PeakReverseVoltage = -12;

        // ---- Ramps ----
        config.OpenLoopRamps.VoltageOpenLoopRampPeriod = 1;
        config.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 1;

        // ---- Output ----
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    }
   

    public void stop() {
        IntakeMotor.setControl(new VoltageOut(0));
    }

    public void RunIntake (double speed) {
        IntakeMotor.setControl(new VoltageOut(speed));
    }




    @Override
    public void periodic() {


    }

}