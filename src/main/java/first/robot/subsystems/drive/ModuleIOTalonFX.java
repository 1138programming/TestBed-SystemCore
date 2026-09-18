// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package first.robot.subsystems.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import first.robot.Constants;
import java.util.function.Supplier;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.util.Units;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Current;
import org.wpilib.units.measure.Voltage;
import org.wpilib.util.Alert;
import org.wpilib.util.Alert.Level;

/**
 * Module IO implementation for two TalonFX motors plus a CANcoder for absolute steer position.
 *
 * <p>The CANcoder is configured with the module's calibration offset, so its absolute position is
 * already the true module heading. That value seeds the turn TalonFX's internal rotor position once
 * at startup, after which the TalonFX closes the steer loop against its own (much faster) sensor.
 */
public class ModuleIOTalonFX implements ModuleIO {
  private static final double driveCurrentLimitAmps = 80;
  private static final double turnCurrentLimitAmps = 40;

  /** SDS MK4i L2: 6.12:1 drive, 150/7:1 steer. */
  public static final double driveReduction = (50.0 / 14.0) * (16.0 / 28.0) * (45.0 / 15.0);

  public static final double turnReduction = (150.0 / 7.0);

  private static final CANBus canBus = CANBus.systemcore(0);

  // Hardware objects
  private final TalonFX driveTalon;
  private final TalonFX turnTalon;
  private final CANcoder cancoder;

  // Config
  private final TalonFXConfiguration driveConfig = new TalonFXConfiguration();
  private final TalonFXConfiguration turnConfig = new TalonFXConfiguration();
  private final CANcoderConfiguration encoderConfig = new CANcoderConfiguration();

  // Control requests
  private final TorqueCurrentFOC torqueCurrentRequest = new TorqueCurrentFOC(0).withUpdateFreqHz(0);
  private final PositionTorqueCurrentFOC positionTorqueCurrentRequest =
      new PositionTorqueCurrentFOC(0.0).withUpdateFreqHz(0);
  private final VelocityTorqueCurrentFOC velocityTorqueCurrentRequest =
      new VelocityTorqueCurrentFOC(0.0).withUpdateFreqHz(0);
  private final CoastOut coast = new CoastOut();

  // Inputs from drive motor
  private final StatusSignal<Angle> drivePosition;
  private final StatusSignal<AngularVelocity> driveVelocity;
  private final StatusSignal<Voltage> driveAppliedVolts;
  private final StatusSignal<Current> driveSupplyCurrentAmps;
  private final StatusSignal<Current> driveTorqueCurrentAmps;

  // Inputs from turn motor
  private final StatusSignal<Angle> turnPosition;
  private final StatusSignal<AngularVelocity> turnVelocity;
  private final StatusSignal<Voltage> turnAppliedVolts;
  private final StatusSignal<Current> turnSupplyCurrentAmps;
  private final StatusSignal<Current> turnTorqueCurrentAmps;

  // Inputs from CANcoder
  private final StatusSignal<Angle> turnAbsolutePosition;

  private final Rotation2d encoderOffset;
  private final Alert seedFailedAlert;

  public ModuleIOTalonFX(DriveConstants.ModuleConfig config) {
    driveTalon = new TalonFX(config.driveMotorId(), canBus);
    turnTalon = new TalonFX(config.turnMotorId(), canBus);
    cancoder = new CANcoder(config.encoderId(), canBus);
    encoderOffset = config.encoderOffset();

    seedFailedAlert =
        new Alert(
            "steerSeedFailed" + config.turnMotorId(),
            "Steer CANcoder "
                + config.encoderId()
                + " never reported a position; that module's steering will be wrong. Check CAN"
                + " wiring and power-cycle.",
            Level.HIGH);

    // Configure the CANcoder. The calibration offset lives on the device, so absolute position is
    // the module heading directly.
    encoderConfig.MagnetSensor.MagnetOffset = encoderOffset.getRotations();
    encoderConfig.MagnetSensor.SensorDirection =
        config.encoderInverted()
            ? SensorDirectionValue.Clockwise_Positive
            : SensorDirectionValue.CounterClockwise_Positive;
    // Report in [-0.5, 0.5) rotations, matching Rotation2d's range.
    encoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
    tryUntilOk(5, () -> cancoder.getConfigurator().apply(encoderConfig, 0.25));

    // Configure drive motor
    driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    driveConfig.Slot0 =
        new Slot0Configs().withKP(DriveConstants.driveKp).withKI(0).withKD(DriveConstants.driveKd);
    driveConfig.Feedback.SensorToMechanismRatio = driveReduction;
    driveConfig.TorqueCurrent.PeakForwardTorqueCurrent = driveCurrentLimitAmps;
    driveConfig.TorqueCurrent.PeakReverseTorqueCurrent = -driveCurrentLimitAmps;
    driveConfig.CurrentLimits.StatorCurrentLimit = driveCurrentLimitAmps;
    driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    driveConfig.ClosedLoopRamps.TorqueClosedLoopRampPeriod = 0.02;
    tryUntilOk(5, () -> driveTalon.getConfigurator().apply(driveConfig, 0.25));
    tryUntilOk(5, () -> driveTalon.setPosition(0.0, 0.25));

    // Configure turn motor
    turnConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    turnConfig.Slot0 =
        new Slot0Configs().withKP(DriveConstants.turnKp).withKI(0).withKD(DriveConstants.turnKd);
    turnConfig.Feedback.SensorToMechanismRatio = turnReduction;
    turnConfig.ClosedLoopGeneral.ContinuousWrap = true;
    turnConfig.TorqueCurrent.PeakForwardTorqueCurrent = turnCurrentLimitAmps;
    turnConfig.TorqueCurrent.PeakReverseTorqueCurrent = -turnCurrentLimitAmps;
    turnConfig.CurrentLimits.StatorCurrentLimit = turnCurrentLimitAmps;
    turnConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    turnConfig.MotorOutput.Inverted =
        config.turnInverted()
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    tryUntilOk(5, () -> turnTalon.getConfigurator().apply(turnConfig, 0.25));

    // Create status signals
    drivePosition = driveTalon.getPosition();
    driveVelocity = driveTalon.getVelocity();
    driveAppliedVolts = driveTalon.getMotorVoltage();
    driveSupplyCurrentAmps = driveTalon.getSupplyCurrent();
    driveTorqueCurrentAmps = driveTalon.getTorqueCurrent();

    turnPosition = turnTalon.getPosition();
    turnVelocity = turnTalon.getVelocity();
    turnAppliedVolts = turnTalon.getMotorVoltage();
    turnSupplyCurrentAmps = turnTalon.getSupplyCurrent();
    turnTorqueCurrentAmps = turnTalon.getTorqueCurrent();

    turnAbsolutePosition = cancoder.getAbsolutePosition();

    // Seed the turn motor from the absolute encoder. This MUST succeed or the module's steering is
    // permanently offset, so block briefly for a real CAN frame rather than trusting a default 0.
    boolean seeded = false;
    for (int i = 0; i < 5; i++) {
      if (turnAbsolutePosition.waitForUpdate(0.25).getStatus().isOK()) {
        if (turnTalon.setPosition(turnAbsolutePosition.getValueAsDouble(), 0.25).isOK()) {
          seeded = true;
          break;
        }
      }
    }
    seedFailedAlert.set(!seeded);

    // Configure periodic frames
    BaseStatusSignal.setUpdateFrequencyForAll(
        1.0 / Constants.loopPeriodSecs, drivePosition, turnPosition, turnAbsolutePosition);
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        driveVelocity,
        driveAppliedVolts,
        driveSupplyCurrentAmps,
        driveTorqueCurrentAmps,
        turnVelocity,
        turnAppliedVolts,
        turnSupplyCurrentAmps,
        turnTorqueCurrentAmps);
    ParentDevice.optimizeBusUtilizationForAll(driveTalon, turnTalon, cancoder);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    // Update drive inputs
    inputs.driveConnected =
        BaseStatusSignal.refreshAll(
                drivePosition,
                driveVelocity,
                driveAppliedVolts,
                driveSupplyCurrentAmps,
                driveTorqueCurrentAmps)
            .isOK();
    inputs.drivePositionRad = Units.rotationsToRadians(drivePosition.getValueAsDouble());
    inputs.driveVelocityRadPerSec = Units.rotationsToRadians(driveVelocity.getValueAsDouble());
    inputs.driveAppliedVolts = driveAppliedVolts.getValueAsDouble();
    inputs.driveSupplyCurrentAmps = driveSupplyCurrentAmps.getValueAsDouble();
    inputs.driveTorqueCurrentAmps = driveTorqueCurrentAmps.getValueAsDouble();

    // Update turn inputs
    inputs.turnConnected =
        BaseStatusSignal.refreshAll(
                turnPosition,
                turnVelocity,
                turnAppliedVolts,
                turnSupplyCurrentAmps,
                turnTorqueCurrentAmps)
            .isOK();
    inputs.turnPosition = Rotation2d.fromRotations(turnPosition.getValueAsDouble());
    inputs.turnVelocityRadPerSec = Units.rotationsToRadians(turnVelocity.getValueAsDouble());
    inputs.turnAppliedVolts = turnAppliedVolts.getValueAsDouble();
    inputs.turnSupplyCurrentAmps = turnSupplyCurrentAmps.getValueAsDouble();
    inputs.turnTorqueCurrentAmps = turnTorqueCurrentAmps.getValueAsDouble();

    // Update CANcoder inputs. Log the raw (uncalibrated) angle, since that is what is needed to
    // work out a new encoder offset.
    inputs.turnEncoderConnected = BaseStatusSignal.refreshAll(turnAbsolutePosition).isOK();
    inputs.turnAbsolutePosition =
        Rotation2d.fromRotations(turnAbsolutePosition.getValueAsDouble()).minus(encoderOffset);
  }

  @Override
  public void runDriveOpenLoop(double output) {
    driveTalon.setControl(torqueCurrentRequest.withOutput(output));
  }

  @Override
  public void runTurnOpenLoop(double output) {
    turnTalon.setControl(torqueCurrentRequest.withOutput(output));
  }

  @Override
  public void runDriveVelocity(double velocityRadPerSec, double feedforward) {
    driveTalon.setControl(
        velocityTorqueCurrentRequest
            .withVelocity(Units.radiansToRotations(velocityRadPerSec))
            .withFeedForward(feedforward));
  }

  @Override
  public void runTurnPosition(Rotation2d rotation) {
    turnTalon.setControl(positionTorqueCurrentRequest.withPosition(rotation.getRotations()));
  }

  @Override
  public void coast() {
    driveTalon.setControl(coast);
    turnTalon.setControl(coast);
  }

  /** Attempts a device call until it reports success. Returns whether it ever succeeded. */
  public static boolean tryUntilOk(int maxAttempts, Supplier<StatusCode> command) {
    for (int i = 0; i < maxAttempts; i++) {
      if (command.get().isOK()) {
        return true;
      }
    }
    return false;
  }
}
