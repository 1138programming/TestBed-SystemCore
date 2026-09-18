// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package first.robot.subsystems.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import first.robot.Constants;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.util.Units;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;

/** IMU implementation for the CTRE Pigeon 2 on the CAN bus. */
public class GyroIOPigeon2 implements GyroIO {
  private final Pigeon2 pigeon =
      new Pigeon2(DriveConstants.PigeonConstants.id, CANBus.systemcore(0));

  private final StatusSignal<Angle> yaw = pigeon.getYaw();
  private final StatusSignal<AngularVelocity> yawVelocity = pigeon.getAngularVelocityZWorld();

  public GyroIOPigeon2() {
    ModuleIOTalonFX.tryUntilOk(
        5, () -> pigeon.getConfigurator().apply(new Pigeon2Configuration(), 0.25));
    ModuleIOTalonFX.tryUntilOk(5, () -> pigeon.setYaw(0.0, 0.25));

    // Yaw feeds odometry every loop; the rate signal is only used for logging.
    yaw.setUpdateFrequency(1.0 / Constants.loopPeriodSecs);
    yawVelocity.setUpdateFrequency(50.0);
    pigeon.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = BaseStatusSignal.refreshAll(yaw, yawVelocity).isOK();
    inputs.yawPosition = Rotation2d.fromDegrees(yaw.getValueAsDouble());
    inputs.yawVelocityRadPerSec = Units.degreesToRadians(yawVelocity.getValueAsDouble());
  }
}
