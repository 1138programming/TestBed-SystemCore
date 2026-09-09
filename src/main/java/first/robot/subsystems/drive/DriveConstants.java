// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package first.robot.subsystems.drive;

import first.robot.Constants;
import first.robot.Constants.RobotType;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.util.Units;

public class DriveConstants {
  public static final double trackWidthX = Units.inchesToMeters(20.75);
  public static final double trackWidthY = Units.inchesToMeters(20.75);
  public static final double driveBaseRadius = Math.hypot(trackWidthX / 2, trackWidthY / 2);
  public static final double maxLinearSpeed = 4.69;
  public static final double maxAngularSpeed = maxLinearSpeed / driveBaseRadius;
  public static final double maxLinearAcceleration = 22.0;

  // Drive/turn gains for the real robot. Both motors are run with torque-current FOC, so all of
  // these are in amps (or amps per unit of error), NOT volts.
  public static final double driveKs = 5.0; // Amps to overcome static friction
  public static final double driveKv = 0.0; // Amps per rad/sec of viscous drag
  public static final double driveKp = 35.0; // Amps per rot/sec of velocity error
  public static final double driveKd = 0.0;
  public static final double turnKp = 4000.0; // Amps per rotation of position error
  public static final double turnKd = 50.0;

  // Gains for the physics simulation. Simulation is voltage based, so these are in volts.
  public static final double driveSimKs = 0.03;
  public static final double driveSimKv = 0.13;
  public static final double driveSimKp = 0.05;
  public static final double driveSimKd = 0.0;
  public static final double turnSimKp = 8.0;
  public static final double turnSimKd = 0.0;

  /** Includes bumpers! */
  public static final double robotWidth =
      Units.inchesToMeters(28.0) + 2 * Units.inchesToMeters(2.0);

  /** Module locations, in the order FL, FR, BL, BR. +x is forward, +y is left. */
  public static final Translation2d[] moduleTranslations = {
    new Translation2d(trackWidthX / 2, trackWidthY / 2),
    new Translation2d(trackWidthX / 2, -trackWidthY / 2),
    new Translation2d(-trackWidthX / 2, trackWidthY / 2),
    new Translation2d(-trackWidthX / 2, -trackWidthY / 2)
  };

  public static final double wheelRadius = Units.inchesToMeters(1.9413001940413326);

  /**
   * Per-module hardware configuration, in the order FL, FR, BL, BR.
   *
   * <p>{@code encoderOffset} is written into the CANcoder's {@code MagnetSensor.MagnetOffset}, so
   * after calibration a module pointing straight forward must read 0. To recalibrate: point every
   * wheel forward (bevel gears all facing the same way), set the offsets below to zero, deploy,
   * then read {@code /Drive/ModuleN/turnAbsolutePosition} and negate each value here.
   */
  public static final ModuleConfig[] moduleConfigs = {
    // FL
    ModuleConfig.builder()
        .driveMotorId(12)
        .turnMotorId(9)
        .encoderId(2)
        .encoderOffset(Rotation2d.fromRadians(0.9022009671847623))
        .turnInverted(true)
        .encoderInverted(false)
        .build(),
    // FR
    ModuleConfig.builder()
        .driveMotorId(2)
        .turnMotorId(10)
        .encoderId(3)
        .encoderOffset(Rotation2d.fromRadians(1.6663099495963458))
        .turnInverted(true)
        .encoderInverted(false)
        .build(),
    // BL
    ModuleConfig.builder()
        .driveMotorId(15)
        .turnMotorId(11)
        .encoderId(4)
        .encoderOffset(Rotation2d.fromRadians(-0.09896592242077659))
        .turnInverted(true)
        .encoderInverted(false)
        .build(),
    // BR
    ModuleConfig.builder()
        .driveMotorId(3)
        .turnMotorId(8)
        .encoderId(5)
        .encoderOffset(Rotation2d.fromRadians(-3.051832863487227))
        .turnInverted(true)
        .encoderInverted(false)
        .build()
  };

  public static class PigeonConstants {
    public static final int id = Constants.getRobot() == RobotType.DEVBOT ? 3 : 30;
  }

  /**
   * Configuration for a single swerve module.
   *
   * @param driveMotorId CAN id of the drive TalonFX
   * @param turnMotorId CAN id of the turn TalonFX
   * @param encoderId CAN id of the steer CANcoder
   * @param encoderOffset Offset applied to the CANcoder so that forward reads zero
   * @param turnInverted Whether the turn motor is inverted
   * @param encoderInverted Whether the CANcoder counts clockwise-positive
   */
  public record ModuleConfig(
      int driveMotorId,
      int turnMotorId,
      int encoderId,
      Rotation2d encoderOffset,
      boolean turnInverted,
      boolean encoderInverted) {

    public static Builder builder() {
      return new Builder();
    }

    /** Hand-written builder, so the project needs no annotation processor for this. */
    public static final class Builder {
      private int driveMotorId;
      private int turnMotorId;
      private int encoderId;
      private Rotation2d encoderOffset = Rotation2d.ZERO;
      private boolean turnInverted;
      private boolean encoderInverted;

      public Builder driveMotorId(int driveMotorId) {
        this.driveMotorId = driveMotorId;
        return this;
      }

      public Builder turnMotorId(int turnMotorId) {
        this.turnMotorId = turnMotorId;
        return this;
      }

      public Builder encoderId(int encoderId) {
        this.encoderId = encoderId;
        return this;
      }

      public Builder encoderOffset(Rotation2d encoderOffset) {
        this.encoderOffset = encoderOffset;
        return this;
      }

      public Builder turnInverted(boolean turnInverted) {
        this.turnInverted = turnInverted;
        return this;
      }

      public Builder encoderInverted(boolean encoderInverted) {
        this.encoderInverted = encoderInverted;
        return this;
      }

      public ModuleConfig build() {
        return new ModuleConfig(
            driveMotorId, turnMotorId, encoderId, encoderOffset, turnInverted, encoderInverted);
      }
    }
  }

  private DriveConstants() {}
}
