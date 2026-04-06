package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HopperSubsystem extends SubsystemBase {
        private VelocityVoltage velocityVoltage;
    private DutyCycleOut dutyCycleOut;
    
    public boolean intakeState = false; // false up, true down
    public int zone = 0; // 0 friendly, 1 neutral, 2 enemy
    public boolean hopperFull = false;

    private final TalonFX hopper = new TalonFX(15);
    private final TalonFX kicker = new TalonFX(16);

    // Desired forward voltages
    private double hopperForwardVolts = 0.0;
    private double kickerForwardVolts = 0.0;

    // Pulse behavior
    private boolean pulseEnabled = false;
    private boolean reversing = false;

    // Example: every 2.0 sec, reverse for 0.5 sec
    private final double pulseIntervalSec = 0.5;
    private final double reverseDurationSec = 0.1;

    private double lastStateChangeTime = 0.0;

    private void setMotorVoltages(double hopperVolts, double kickerVolts) {
        hopper.setControl(new VoltageOut(hopperVolts));
        kicker.setControl(new VoltageOut(kickerVolts));
    }

    /**
     * Start running hopper system with periodic reverse pulses.
     * hopperVolts = hopper motor forward voltage
     * kickerVolts = kicker motor forward voltage
     */
    public void activate(double hopperVolts, double kickerVolts) {
        this.hopperForwardVolts = hopperVolts;
        this.kickerForwardVolts = kickerVolts;

        pulseEnabled = true;
        reversing = false;
        lastStateChangeTime = Timer.getFPGATimestamp();

        setMotorVoltages(hopperForwardVolts, kickerForwardVolts);
    }

    public void deactivate() {
        pulseEnabled = false;
        reversing = false;
        hopperForwardVolts = 0.0;
        kickerForwardVolts = 0.0;

        setMotorVoltages(0.0, 0.0);
    }

    @Override
    public void periodic() {
        if (!pulseEnabled) {
            return;
        }

        double now = Timer.getFPGATimestamp();

        if (!reversing) {
            // Stay in forward until it's time to do a short reverse pulse
            if (now - lastStateChangeTime >= pulseIntervalSec) {
                reversing = true;
                lastStateChangeTime = now;

                // Reverse both motors
                setMotorVoltages(-hopperForwardVolts, +kickerForwardVolts);
            } else {
                // Keep forward running
                setMotorVoltages(hopperForwardVolts, kickerForwardVolts);
            }
        } else {
            // Currently reversing
            if (now - lastStateChangeTime >= reverseDurationSec) {
                reversing = false;
                lastStateChangeTime = now;

                // Go back to forward
                setMotorVoltages(hopperForwardVolts, kickerForwardVolts);
            } else {
                // Keep reversing during pulse window
                setMotorVoltages(-hopperForwardVolts, kickerForwardVolts);
            }
        }
    }

    public Command activateCommand(double hopperVolts, double kickerVolts) {
        return runOnce(() -> activate(hopperVolts, kickerVolts));
    }

        public void RunHopper (double speed) {
        hopper.setControl(new VoltageOut(speed));
        }
    

    public Command deactivateCommand() {
        return runOnce(this::deactivate);
    }
}