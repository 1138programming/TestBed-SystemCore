// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2025.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import lombok.Builder;
import org.littletonrobotics.frc2025.Constants;
import org.littletonrobotics.frc2025.Constants.RobotType;

public class DriveConstants {
  public static final double trackWidthX = Units.inchesToMeters(20.75);
  public static final double trackWidthY = Units.inchesToMeters(20.75);
  public static final double driveBaseRadius = Math.hypot(trackWidthX / 2, trackWidthY / 2);
  public static final double maxLinearSpeed = 4.69;
  public static final double maxAngularSpeed = 4.69 / driveBaseRadius;
  public static final double maxLinearAcceleration = 22.0;

  public static final double driveKs = 5.0;
  public static final double driveKv = 0.0;
  public static final double driveKp = 35.0;
  public static final double driveKd = 0.0;
  public static final double turnKp = 4000.0;
  public static final double turnKd = 50.0;

  /** Includes bumpers! */
  public static final double robotWidth =
      Units.inchesToMeters(28.0) + 2 * Units.inchesToMeters(2.0);

  public static final Translation2d[] moduleTranslations = {
    new Translation2d(trackWidthX / 2, trackWidthY / 2),
    new Translation2d(trackWidthX / 2, -trackWidthY / 2),
    new Translation2d(-trackWidthX / 2, trackWidthY / 2),
    new Translation2d(-trackWidthX / 2, -trackWidthY / 2)
  };

  public static final double wheelRadius = Units.inchesToMeters(1.9413001940413326);

  public static final ModuleConfig[] moduleConfigs = {
    // FL
    ModuleConfig.builder()
        .driveMotorId(12)
        .turnMotorId(9)
        .encoderChannel(2)
        .encoderOffset(Rotation2d.fromRadians(0.9022009671847623))
        .turnInverted(true)
        .encoderInverted(false)
        .build(),
    // FR
    ModuleConfig.builder()
        .driveMotorId(2)
        .turnMotorId(10)
        .encoderChannel(3)
        .encoderOffset(Rotation2d.fromRadians(1.6663099495963458))
        .turnInverted(true)
        .encoderInverted(false)
        .build(),
    // BL
    ModuleConfig.builder()
        .driveMotorId(15)
        .turnMotorId(11)
        .encoderChannel(4)
        .encoderOffset(Rotation2d.fromRadians(-0.09896592242077659))
        .turnInverted(true)
        .encoderInverted(false)
        .build(),
    // BR
    ModuleConfig.builder()
        .driveMotorId(3)
        .turnMotorId(8)
        .encoderChannel(5)
        .encoderOffset(Rotation2d.fromRadians(-3.051832863487227))
        .turnInverted(true)
        .encoderInverted(false)
        .build()
  };

  public static class PigeonConstants {
    public static final int id = Constants.getRobot() == RobotType.DEVBOT ? 3 : 30;
  }

  @Builder
  public record ModuleConfig(
      int driveMotorId,
      int turnMotorId,
      int encoderChannel,
      Rotation2d encoderOffset,
      boolean turnInverted,
      boolean encoderInverted) {}
}
