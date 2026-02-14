package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

// import com.revrobotics.CANSparkFlex;
// import com.revrobotics.SparkPIDController;
// import com.revrobotics.CANSparkBase.ControlType;
// import com.revrobotics.CANSparkBase.IdleMode;
// import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsytem extends SubsystemBase {
    private final TalonFX intakeRightMotor;
    private final TalonFX intakeLeftMotor;
    // private final SparkPIDController pid1;
    // private final SparkPIDController pid2;
  private final DutyCycleOut leftOut = new DutyCycleOut(0);
  private final DutyCycleOut rightOut = new DutyCycleOut(0);
  private final VelocityVoltage m_voltageVelocity = new VelocityVoltage(0);

    public ShooterSubsytem() {
        intakeRightMotor = new TalonFX(23); 
        intakeLeftMotor = new TalonFX(24); 

    var leftConfiguration = new TalonFXConfiguration();
    var rightConfiguration = new TalonFXConfiguration();
    /* User can optionally change the configs or leave it alone to perform a factory default */
   leftConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    leftConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    //leftConfiguration.Feedback.FeedbackRemoteSensorID = armAbsolutCANcoder.getDeviceID();
    //leftConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    leftConfiguration.Slot0.kP = 0.75;
 //   leftConfiguration.Slot0.kV = 0.01500000071246177;

    leftConfiguration.Voltage.PeakForwardVoltage = 5;
    leftConfiguration.Voltage.PeakReverseVoltage = -5;
   rightConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    rightConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Coast;
   // rightConfiguration.Feedback.FeedbackRemoteSensorID = armAbsolutCANcoder.getDeviceID();
    //rightConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.SyncCANcoder;
    rightConfiguration.Slot0.kP = 0.75;
    // rightConfiguration.Slot0.kI = 0.75;
    // rightConfiguration.Slot0.kD = 0;
 //   rightConfiguration.Slot0.kV = 0.01500000071246177;
    rightConfiguration.Voltage.PeakForwardVoltage = 5;
    rightConfiguration.Voltage.PeakReverseVoltage = -5;

    // absoluteEncoderConfiguration.MagnetSensor.

    intakeLeftMotor.getConfigurator().apply(leftConfiguration);
    intakeRightMotor.getConfigurator().apply(rightConfiguration);
        
        // pid1.setSmartMotionMaxAccel(0, 0);
        

        // pid1.setP(0.00165500000237487257); intakeRightMotor
        // pid2.setP(0.001655000000237487257);

        // pid1.setI(0);
        // pid2.setI(0);
        // pid1.setD(0);
        // pid2.setD(0);

        // pid1.setFF(0.0001500000071246177);
        // pid2.setFF(0.0001500000071246177);


    }


    // public void runOutTake(double N_speed, double P_speed) {
    //     N_speed = Math.max( Math.min(N_speed , 1) , -1);
    //     P_speed = Math.max( Math.min(N_speed , 1) , -1);
    //     leftOut.Output = N_speed;
    //     rightOut.Output = N_speed;

    //     intakeLeftMotor.setControl(leftOut);
    //     intakeRightMotor.setControl(rightOut);
    // }
    public void runOutTake(double N_speed , double P_speed){
        intakeLeftMotor.setControl(m_voltageVelocity.withVelocity(N_speed));
        intakeRightMotor.setControl(m_voltageVelocity.withVelocity(P_speed));

    }

    public Command runOutTakeCommand(double N_speed , double P_speed){
        return runOnce(() -> {
            runOutTake(N_speed, P_speed);
            }
        );
    }

    public void stopOutTake() {
        intakeLeftMotor.set(0);
        intakeRightMotor.set(0);

    }

    public double getRightMotorVelocity() {
        return intakeRightMotor.getVelocity().getValueAsDouble();
        }
        
        public double getLeftMotorVelocity() {
        return intakeLeftMotor.getVelocity().getValueAsDouble();
        }
    public void periodic() {
        double currentRightSpeed = this.getRightMotorVelocity();
        double currentLeftSpeed = this.getLeftMotorVelocity();

        SmartDashboard.putNumber("left Speed outake", currentLeftSpeed);
        SmartDashboard.putNumber("right Speed outake", currentRightSpeed);

        SmartDashboard.putBoolean("Outake Spool", Math.abs(currentRightSpeed - 70) <= 10 && Math.abs(currentLeftSpeed + 70) <= 10 ); // 10 = tolerance.
    }
}