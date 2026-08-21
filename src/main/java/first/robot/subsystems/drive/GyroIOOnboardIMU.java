// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2025.subsystems.drive;

import edu.wpi.first.wpilibj.OnboardIMU;
import edu.wpi.first.wpilibj.OnboardIMU.MountOrientation;

public class GyroIOOnboardIMU implements GyroIO {
  private final OnboardIMU imu = new OnboardIMU(MountOrientation.kFlat);

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = true;
    inputs.yawPosition = imu.getRotation2d();
    inputs.yawVelocityRadPerSec = imu.getGyroRateZ();
  }
}
