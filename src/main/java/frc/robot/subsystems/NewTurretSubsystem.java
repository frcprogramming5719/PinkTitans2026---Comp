package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants.CanIDConstants;
// import frc.robot.Constants.TurretConstants;
// import frc.robot.Constants.TurretConstants.PivotConstants;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;
import edu.wpi.first.wpilibj.Timer;


import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.LimelightResults;
import frc.robot.LimelightHelpers.PoseEstimate;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.MechanismPositionConfig;
import yams.mechanisms.config.PivotConfig;
import yams.mechanisms.positional.Pivot;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;

import yams.motorcontrollers.local.SparkWrapper;
import yams.motorcontrollers.remote.TalonFXWrapper;


import yams.units.EasyCRT;
import yams.units.EasyCRTConfig;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import yams.motorcontrollers.local.NovaWrapper;
import frc.robot.subsystems.CommandSwerveDrivetrain;
 import yams.units.EasyCRT;
import yams.units.EasyCRTConfig;

public class NewTurretSubsystem extends SubsystemBase {

    private static final String RERUN_SEED = "Turret/CRT/RerunSeed";


 //   private SparkMax turretMotor = new SparkMax(CanIDConstants.turretID, MotorType.kBrushless);
 
    private TalonFX turretMotor = new TalonFX(20);
    private final TalonFXConfiguration config = new TalonFXConfiguration();

  //  private final AbsoluteEncoder cancoderB = turretMotor.getAbsoluteEncoder(); // 20t B SparkFlex
    private final CANcoder cancoderA = new CANcoder(41); // 19 A rio
    private final CANcoder cancoderB = new CANcoder(42); // 19 A rio

    // Create a timer to delay CRT run until encoders are ready

    private Timer startUpTimer = new Timer();
    private boolean startTimer = false;
    private boolean delayForCRTDone = false;
    private final Double absPositionASignal;
    private final Double absPositionBSignal;
    private final EasyCRTConfig easyCRTConfig;

    private boolean rotorSeededFromAbs = false;
    private double lastSeededTurretDeg = Double.NaN;
    private double lastSeedError = Double.NaN;
    private double lastAbsA = Double.NaN;
    private double lastAbsB = Double.NaN;
    private String lastSeedStatus = "NOT_ATTEMPTED";

  LimelightHelpers limelight;
  LimelightHelpers limelightPoseEstimator;
  private int outofAreaReading = 0;
  private boolean initialReading = false;


// public void configureMotor() {

//       config.Slot0.kP = 1;
//     config.Slot0.kI = 0;
//     config.Slot0.kD = 0;
//     config.Slot0.kS = 0;
//     config.Slot0.kV = 0;
//     config.Slot0.kA = 0;
//     config.Slot0.kG = 0;

    
//     // ================= Motion Magic =================
//     config.MotionMagic.MotionMagicCruiseVelocity = 180;
//     config.MotionMagic.MotionMagicAcceleration = 90;

//     // ================= Gear Ratio =================
//     config.Feedback.SensorToMechanismRatio = 3.0 * 4.0;

//     // ================= Current Limit =================
//     config.CurrentLimits.StatorCurrentLimit = 40;
//     config.CurrentLimits.StatorCurrentLimitEnable = true;

//     // ================= Voltage Compensation =================
//     config.Voltage.PeakForwardVoltage = 12;
//     config.Voltage.PeakReverseVoltage = -12;

//     // ================= Ramp Rates =================
//     config.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.25;
//     config.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.25;

//     // ================= Brake Mode =================
//     config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

//     // ================= Inversion =================
//     config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

//     turretMotor.getConfigurator().apply(config);
// }

private final SmartMotorControllerConfig smartMotorControllerConfig =
    new SmartMotorControllerConfig(this)   
        .withControlMode(SmartMotorControllerConfig.ControlMode.CLOSED_LOOP)
        .withIdleMode(SmartMotorControllerConfig.MotorMode.BRAKE)
        .withMotorInverted(false)
        .withEncoderInverted(false)
          .withClosedLoopController(
              35,
              0,
              0,
              DegreesPerSecond.of(180),
              DegreesPerSecondPerSecond.of(90))        // 180 and 90 mean how fast the turret cna spin per sec
        // atch your turret gear reduction
        .withGearing(new MechanismGearing(GearBox.fromReductionStages( 3, 4 )))
        .withStartingPosition(Degrees.of(0))
        .withTelemetry("TurretMotor", SmartMotorControllerConfig.TelemetryVerbosity.HIGH)
        .withStatorCurrentLimit(Amps.of(40))
        .withClosedLoopRampRate(Seconds.of(0.25))
        .withOpenLoopRampRate(Seconds.of(0.25));




  private final SmartMotorController motor =
      new TalonFXWrapper(turretMotor, DCMotor.getKrakenX60(1), smartMotorControllerConfig);

  private final MechanismPositionConfig robotToMechanism =
      new MechanismPositionConfig()
          .withMaxRobotHeight(Meters.of(1.5))
          .withMaxRobotLength(Meters.of(0.75))
          .withRelativePosition(new Translation3d(Meters.of(-0.25), // back from robot center
                                                  Meters.of(0),     // centered left/right
                                                  Meters.of(0.5))); // up from the floor reference


  private final PivotConfig m_config =
      new PivotConfig(motor)
          .withHardLimit(Degrees.of(-140), Degrees.of(140))
          .withSoftLimits(Degrees.of(-135), Degrees.of(135))
          .withTelemetry("TurretPivot", TelemetryVerbosity.HIGH)
          .withStartingPosition(Degrees.of(0))
          .withMechanismPositionConfig(robotToMechanism)
          .withMOI(Meter.of(0.001), Pounds.of(3));

  private final Pivot turret = new Pivot(m_config);




  public NewTurretSubsystem() {


      absPositionASignal = (getAbsoluteEncoderWithOffset());
      absPositionBSignal = cancoderB.getPosition().getValueAsDouble();

      easyCRTConfig = buildEasyCrtConfig(); 
      logCrtConfigTelemetry();
      SmartDashboard.putBoolean(RERUN_SEED, false);

  }

//   public void setupLimelight() {

//     limelight = new Limelight("limelight");
//     limelight
//         .getSettings()
//         .withPipelineIndex(0)
//         .withCameraOffset(
//             new Pose3d(
//                 Units.inchesToMeters(0),
//                 Units.inchesToMeters(0),
//                 Units.inchesToMeters(0),
//                 new Rotation3d(0, 0, Units.degreesToRadians(0))))
//         .withAprilTagIdFilter(List.of(17, 18, 19, 20, 21, 22, 6, 7, 8, 9, 10, 11))
//         .save();

//       limelightPoseEstimator = limelight.createPoseEstimator(LimelightPoseEstimator.EstimationMode.MEGATAG2);
//   }

  public void setupLimelight() {
    String llName = "limelight";
    LimelightHelpers.PoseEstimate poseEstimate = new LimelightHelpers.PoseEstimate();


    // Set pipeline index to 0
    LimelightHelpers.setPipelineIndex(llName, 0);

    // Set camera pose offset relative to robot
    // (X = forward, Y = side, Z = up, roll/pitch/yaw in degrees)
    LimelightHelpers.setCameraPose_RobotSpace(
        llName,
        Units.inchesToMeters(0), // forward
        Units.inchesToMeters(0), // side
        Units.inchesToMeters(0), // up
        0, // roll
        0, // pitch
        0  // yaw
    );

    // Set which AprilTag IDs to track
    LimelightHelpers.SetFiducialIDFiltersOverride(
        llName,
        new int[]{17, 18, 19, 20, 21, 22, 6, 7, 8, 9, 10, 11}
    );

    // If using MegaTag2 localization, make sure to call SetRobotOrientation elsewhere as needed

    // Initialize your PoseEstimator for MegaTag2
    poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(llName);

}






  public void periodic() {
    turret.updateTelemetry();

    double absA = cancoderA.getAbsolutePosition().getValueAsDouble();
    double absB = cancoderB.getAbsolutePosition().getValueAsDouble();
    SmartDashboard.putNumber("Turret/CANCoderA_AbsPos", absA);
    SmartDashboard.putNumber("Turret/CANCoderB_AbsPos", absB);

SmartDashboard.putNumber("Turret/Angle_Degrees", turret.getAngle().in(Degrees));


    }
  

  public void simulationPeriodic() {
    turret.simIterate();
  }

  public Command sysId() {
    return turret.sysId(Volts.of(4.0), Volts.of(8.0).per(Second), Second.of(30));
  }

  public Command set(double dutyCycle) {
      return turret.set(dutyCycle);
  }
  public Command setAngle(Angle angle) {
    return turret
        .setAngle(angle)
        .until(turret.isNear(angle, Degrees.of(2)));
  }

    public Command setAngleDynamic(Supplier<Angle> angle) {
        return turret.setAngle(angle);
    }

    public Angle getRawAngle() {
        return turret.getAngle();
    }

    public Angle getRobotAdjustedAngle() {
        return turret.getAngle().plus(Degrees.of(180));
    }

    public double getRobotRelativeYawRadians() {
        return getRawAngle().in(edu.wpi.first.units.Units.Radians);
    }

    /** Forces a CRT reseed attempt */
    public void rerunCrtSeed() {
        rotorSeededFromAbs = false;
        SmartDashboard.putNumber("Turret/CRT/ManualRerunTimestampSec", Timer.getFPGATimestamp());
        attemptRotorSeedFromCANCoders();
    }

    /**
     * Tries to solve turret position via CRT and seed the relative encoder with the result. Reads
     * both CANCoder values, runs the solver, updates the SmartMotorController encoder, and publishes
     * CRT status to the dashboard.
     */
    private void attemptRotorSeedFromCANCoders() {
        AbsSensorRead absRead = readAbsSensors();
        if (!absRead.ok()) {
            if (!"NO_DEVICES".equals(absRead.status())) {
                SmartDashboard.putString("Turret/CRT/SeedStatus", absRead.status());
            }
            lastSeedStatus = absRead.status();
            return;
        }

        double absA = absRead.absA();
        double absB = absRead.absB();
        lastAbsA = absA;
        lastAbsB = absB;

        var solver = new EasyCRT(easyCRTConfig);
        var solvedAngle = solver.getAngleOptional();

        SmartDashboard.putNumber("Turret/CRT/AbsA", absA);
        SmartDashboard.putNumber("Turret/CRT/AbsB", absB);
        SmartDashboard.putString("Turret/CRT/SolverStatus", solver.getLastStatus() + "");
        SmartDashboard.putNumber("Turret/CRT/SolverErrorRot", solver.getLastErrorRotations());
        SmartDashboard.putNumber("Turret/CRT/SolverIterations", solver.getLastIterations());

        if (solvedAngle.isEmpty()) {
            SmartDashboard.putBoolean("Turret/CRT/SolutionFound", false);
            lastSeedStatus = solver.getLastStatus() + "";
            return;
        }

        Angle turretRotations = solvedAngle.get();
        motor.setEncoderPosition(turretRotations);
        rotorSeededFromAbs = true;
        lastSeededTurretDeg = turretRotations.in(Degrees);
        lastSeedError = solver.getLastErrorRotations();
        SmartDashboard.putBoolean("Turret/CRT/SolutionFound", true);
        SmartDashboard.putNumber("Turret/CRT/SeededTurretDeg", lastSeededTurretDeg);
        SmartDashboard.putNumber("Turret/CRT/MatchErrorRot", lastSeedError);

        lastSeedStatus = "OK";
        SmartDashboard.putString("Turret/CRT/SeedStatus", lastSeedStatus);
        SmartDashboard.putBoolean("Turret/CRT/Seeded", rotorSeededFromAbs);
    }

    /** Reads both absolute encoders and returns their rotations plus a status. */
    private AbsSensorRead readAbsSensors() {
         Double absPositionASignal = (cancoderA.getAbsolutePosition().getValueAsDouble());
         Double absPositionBSignal = cancoderB.getAbsolutePosition().getValueAsDouble();

        boolean haveDevices = cancoderA != null && cancoderB != null;

        if (haveDevices) {

            return new AbsSensorRead(true, absPositionASignal, absPositionBSignal, "ok");
        }
        return new AbsSensorRead(false, Double.NaN, Double.NaN, "NO_DEVICES");
    }

    /** Build the CRT config */
    private EasyCRTConfig buildEasyCrtConfig() {
        return new EasyCRTConfig(
                cancoderA.getAbsolutePosition().asSupplier(),
                cancoderB.getAbsolutePosition().asSupplier())
                .withCommonDriveGear(1, 200, 19, 21)
                .withAbsoluteEncoderOffsets(Rotations.of(0), Rotations.of(-0.650758))
                .withAbsoluteEncoderInversions(false, false)
                .withMechanismRange(Rotations.of(-0.6), Rotations.of(0.6))
                .withMatchTolerance(Rotations.of(0.05))
                .withCrtGearRecommendationConstraints(1.2, 15, 60, 40);
        // } else {
        //   return null;
        // }
    }

    /** Publish CRT config-derived values for debugging coverage/ratios. */
    private void logCrtConfigTelemetry() {
        double mechanismRangeRot = easyCRTConfig.getMechanismRange().in(Rotations);
        double uniqueCoverageRot =
                easyCRTConfig.getUniqueCoverage().map(angle -> angle.in(Rotations)).orElse(Double.NaN);
        SmartDashboard.putNumber(
                "Turret/CRT/Config/RatioA", easyCRTConfig.getEncoder1RotationsPerMechanismRotation());
        SmartDashboard.putNumber(
                "Turret/CRT/Config/RatioB", easyCRTConfig.getEncoder2RotationsPerMechanismRotation());
        SmartDashboard.putNumber("Turret/CRT/Config/UniqueCoverageRot", uniqueCoverageRot);
        SmartDashboard.putBoolean(
                "Turret/CRT/Config/CoverageSatisfiesRange", easyCRTConfig.coverageSatisfiesRange());
        SmartDashboard.putNumber("Turret/CRT/Config/RequiredRangeRot", mechanismRangeRot);

        var configPair = easyCRTConfig.getRecommendedCrtGearPair();
        SmartDashboard.putBoolean("Turret/CRT/Config/RecommendedPairFound", configPair.isPresent());
        if (configPair.isPresent()) {
            var pair = configPair.get();
            SmartDashboard.putNumber("Turret/CRT/Config/Reccomender/RecommendedGearA", pair.gearA());
            SmartDashboard.putNumber("Turret/CRT/Config/Reccomender/RecommendedGearB", pair.gearB());
            SmartDashboard.putNumber(
                    "Turret/CRT/Config/Reccomender/RecommendedCoverageRot", pair.coverage().in(Rotations));
            SmartDashboard.putNumber("Turret/CRT/Config/Reccomender/RecommendedLcm", pair.lcm());
            SmartDashboard.putBoolean(
                    "Turret/CRT/Config/Reccomender/RecommendedCoprime",
                    EasyCRTConfig.isCoprime(pair.gearA(), pair.gearB()));
            SmartDashboard.putNumber(
                    "Turret/CRT/Config/Reccomender/RecommendedIterations", pair.theoreticalIterations());
        }
    }

    private Double getAbsoluteEncoderWithOffset() {

        return MathUtil.inputModulus(cancoderA.getAbsolutePosition().getValueAsDouble() - 0, 0, 1);
    }

    private static record AbsSensorRead(boolean ok, double absA, double absB, String status) {}
}