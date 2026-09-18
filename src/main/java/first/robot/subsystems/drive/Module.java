// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package first.robot.subsystems.drive;

import first.robot.Constants;
import first.robot.Constants.Mode;
import org.littletonrobotics.junction.Logger;
import org.wpilib.driverstation.RobotState;
import org.wpilib.math.controller.SimpleMotorFeedforward;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.kinematics.SwerveModulePosition;
import org.wpilib.math.kinematics.SwerveModuleVelocity;
import org.wpilib.math.util.Units;
import org.wpilib.util.Alert;
import org.wpilib.util.Alert.Level;

public class Module {
  private final ModuleIO io;
  private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
  private final int index;

  private final SimpleMotorFeedforward ffModel;

  private final Alert driveDisconnectedAlert;
  private final Alert turnDisconnectedAlert;
  private final Alert turnEncoderDisconnectedAlert;

  public Module(ModuleIO io, int index) {
    this.io = io;
    this.index = index;

    // Simulation is voltage controlled, the real robot is torque-current controlled, so the
    // feedforward constants differ by more than just tuning.
    ffModel =
        Constants.getMode() == Mode.SIM
            ? new SimpleMotorFeedforward(DriveConstants.driveSimKs, DriveConstants.driveSimKv)
            : new SimpleMotorFeedforward(DriveConstants.driveKs, DriveConstants.driveKv);

    driveDisconnectedAlert =
        new Alert(
            "driveDisconnected" + index,
            "Disconnected drive motor on module " + index + ".",
            Level.MEDIUM);
    turnDisconnectedAlert =
        new Alert(
            "turnDisconnected" + index,
            "Disconnected turn motor on module " + index + ".",
            Level.MEDIUM);
    turnEncoderDisconnectedAlert =
        new Alert(
            "turnEncoderDisconnected" + index,
            "Disconnected steer CANcoder on module " + index + ".",
            Level.MEDIUM);
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Drive/Module" + index, inputs);

    // Update alerts
    driveDisconnectedAlert.set(!inputs.driveConnected);
    turnDisconnectedAlert.set(!inputs.turnConnected);
    turnEncoderDisconnectedAlert.set(!inputs.turnEncoderConnected);

    // Coast when disabled
    if (RobotState.isDisabled()) {
      io.coast();
    }
  }

  /**
   * Runs the module with the specified setpoint state.
   *
   * @return the optimized state that was actually applied, for logging
   */
  public SwerveModuleVelocity runSetpoint(SwerveModuleVelocity state) {
    // As of 2027, optimize() and cosineScale() are pure - they return a new state instead of
    // mutating in place, so their results must be used.
    SwerveModuleVelocity optimized = state.optimize(getAngle()).cosineScale(getAngle());

    // Apply setpoints
    double speedRadPerSec = optimized.velocity / DriveConstants.wheelRadius;
    io.runDriveVelocity(speedRadPerSec, ffModel.calculate(speedRadPerSec));
    io.runTurnPosition(optimized.angle);

    return optimized;
  }

  /** Runs the module with the specified output while controlling to zero degrees. */
  public void runCharacterization(double output) {
    io.runDriveOpenLoop(output);
    io.runTurnPosition(Rotation2d.ZERO);
  }

  /** Disables all outputs to motors. */
  public void stop() {
    io.runDriveOpenLoop(0.0);
    io.runTurnOpenLoop(0.0);
  }

  /** Returns the current turn angle of the module. */
  public Rotation2d getAngle() {
    return inputs.turnPosition;
  }

  /** Returns the current drive position of the module in meters. */
  public double getPositionMeters() {
    return inputs.drivePositionRad * DriveConstants.wheelRadius;
  }

  /** Returns the current drive velocity of the module in meters per second. */
  public double getVelocityMetersPerSec() {
    return inputs.driveVelocityRadPerSec * DriveConstants.wheelRadius;
  }

  /** Returns the module position (turn angle and drive position). */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getPositionMeters(), getAngle());
  }

  /** Returns the module state (turn angle and drive velocity). */
  public SwerveModuleVelocity getVelocity() {
    return new SwerveModuleVelocity(getVelocityMetersPerSec(), getAngle());
  }

  /** Returns the module position in radians. */
  public double getWheelRadiusCharacterizationPosition() {
    return inputs.drivePositionRad;
  }

  /** Returns the module velocity in rotations/sec (Phoenix native units). */
  public double getFFCharacterizationVelocity() {
    return Units.radiansToRotations(inputs.driveVelocityRadPerSec);
  }
}
