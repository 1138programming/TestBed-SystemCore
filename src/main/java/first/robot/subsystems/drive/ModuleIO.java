// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package first.robot.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;
import org.wpilib.math.geometry.Rotation2d;

public interface ModuleIO {
  @AutoLog
  public static class ModuleIOInputs {
    public boolean driveConnected = false;
    public double drivePositionRad = 0.0;
    public double driveVelocityRadPerSec = 0.0;
    public double driveAppliedVolts = 0.0;
    public double driveSupplyCurrentAmps = 0.0;
    public double driveTorqueCurrentAmps = 0.0;

    public boolean turnConnected = false;
    public boolean turnEncoderConnected = false;

    /** Raw CANcoder reading, before the calibration offset. Used to find encoder offsets. */
    public Rotation2d turnAbsolutePosition = new Rotation2d();

    /** Calibrated module heading. Zero means the wheel points straight forward. */
    public Rotation2d turnPosition = new Rotation2d();

    public double turnVelocityRadPerSec = 0.0;
    public double turnAppliedVolts = 0.0;
    public double turnSupplyCurrentAmps = 0.0;
    public double turnTorqueCurrentAmps = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ModuleIOInputs inputs) {}

  /** Run the drive motor at the specified open loop value. */
  public default void runDriveOpenLoop(double output) {}

  /** Run the turn motor at the specified open loop value. */
  public default void runTurnOpenLoop(double output) {}

  /** Run the drive motor at the specified velocity. */
  public default void runDriveVelocity(double velocityRadPerSec, double feedforward) {}

  /** Run the turn motor to the specified rotation. */
  public default void runTurnPosition(Rotation2d rotation) {}

  /** Run in coast mode. */
  public default void coast() {}
}
