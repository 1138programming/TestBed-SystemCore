package first.robot.subsystems.drive;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import first.robot.Constants;
import first.robot.SimTestFixture;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.kinematics.SwerveModuleVelocity;

/**
 * End-to-end checks that the swerve stack actually drives: kinematics -> module optimization ->
 * simulated motors -> measured chassis velocity. These drive robot-relative, so the measured
 * velocity is directly comparable to the commanded one.
 */
class DriveSimTest {
  private Drive drive;

  @BeforeEach
  void setUp() {
    drive = SimTestFixture.drive();
    SimTestFixture.reset();
  }

  /** Runs the drive at a fixed velocity for a while and returns the measured chassis velocity. */
  private ChassisVelocities settleAt(ChassisVelocities target, int loops) {
    for (int i = 0; i < loops; i++) {
      SimTestFixture.tick();
      drive.periodic();
      drive.runVelocity(target);
    }
    SimTestFixture.tick();
    drive.periodic();
    return SimTestFixture.measured();
  }

  @Test
  void drivesStraightForward() {
    ChassisVelocities measured = settleAt(new ChassisVelocities(2.0, 0.0, 0.0), 200);
    assertEquals(2.0, measured.vx, 0.25, "forward velocity should track the setpoint");
    assertEquals(0.0, measured.vy, 0.25, "should not drift sideways");
    assertEquals(0.0, measured.omega, 0.25, "should not rotate");
  }

  @Test
  void drivesSideways() {
    ChassisVelocities measured = settleAt(new ChassisVelocities(0.0, 1.5, 0.0), 200);
    assertEquals(0.0, measured.vx, 0.25);
    assertEquals(1.5, measured.vy, 0.25, "strafe velocity should track the setpoint");
    assertEquals(0.0, measured.omega, 0.25);
  }

  @Test
  void spinsInPlace() {
    ChassisVelocities measured = settleAt(new ChassisVelocities(0.0, 0.0, 2.0), 200);
    assertEquals(0.0, measured.vx, 0.25);
    assertEquals(0.0, measured.vy, 0.25);
    assertEquals(2.0, measured.omega, 0.3, "angular velocity should track the setpoint");
  }

  @Test
  void drivesDiagonallyWhileRotating() {
    ChassisVelocities target = new ChassisVelocities(1.5, 1.0, 1.0);
    ChassisVelocities measured = settleAt(target, 250);
    assertEquals(target.vx, measured.vx, 0.3);
    assertEquals(target.vy, measured.vy, 0.3);
    assertEquals(target.omega, measured.omega, 0.3);
  }

  /**
   * A request beyond what the modules can do must be scaled down as a whole, not clipped per
   * module. This is what the discarded {@code desaturateWheelVelocities} result used to break.
   */
  @Test
  void desaturatesOverspeedRequests() {
    settleAt(new ChassisVelocities(DriveConstants.maxLinearSpeed * 3.0, 0.0, 0.0), 200);
    for (SwerveModuleVelocity state : drive.getModuleVelocities()) {
      assertTrue(
          Math.abs(state.velocity) <= DriveConstants.maxLinearSpeed * 1.1,
          "module speed "
              + state.velocity
              + " exceeded the max of "
              + DriveConstants.maxLinearSpeed);
    }
  }

  /**
   * Reversing direction must flip the wheel rather than steer the module 180 degrees. This is what
   * the discarded {@code optimize()} result used to break.
   */
  @Test
  void reversingFlipsWheelInsteadOfSteering() {
    settleAt(new ChassisVelocities(2.0, 0.0, 0.0), 200);
    ChassisVelocities measured = settleAt(new ChassisVelocities(-2.0, 0.0, 0.0), 200);

    assertEquals(-2.0, measured.vx, 0.25, "should track the reversed setpoint");
    for (SwerveModuleVelocity state : drive.getModuleVelocities()) {
      // Steering stayed near the forward/backward axis; the wheel spins backwards instead.
      double angleFromForward = Math.abs(state.angle.getDegrees());
      assertTrue(
          angleFromForward < 15.0 || angleFromForward > 165.0,
          "module should not be steered sideways, was " + state.angle.getDegrees() + " deg");
      assertTrue(
          Math.signum(state.velocity) * Math.cos(state.angle.getRadians()) < 0.0,
          "module should be driving backwards");
    }
  }

  /** The period used for discretization and sim integration must match the real robot loop. */
  @Test
  void loopPeriodMatchesRobotPeriod() {
    assertEquals(
        org.littletonrobotics.junction.LoggedRobot.defaultPeriodSecs,
        Constants.loopPeriodSecs,
        1e-9,
        "Constants.loopPeriodSecs must match the period handed to LoggedRobot");
  }

  /** stopWithX must park the modules in an X and hold still. */
  @Test
  void stopWithXHoldsStill() {
    settleAt(new ChassisVelocities(2.0, 0.0, 0.0), 100);
    drive.stopWithX();
    for (int i = 0; i < 100; i++) {
      SimTestFixture.tick();
      drive.periodic();
      drive.stop();
    }
    SimTestFixture.tick();
    drive.periodic();

    for (SwerveModuleVelocity state : drive.getModuleVelocities()) {
      assertEquals(0.0, state.velocity, 0.15, "modules should be stopped");
    }
    // Module headings should form the X pattern (45 degrees off axis).
    Rotation2d flHeading = drive.getModuleVelocities()[0].angle;
    assertEquals(45.0, Math.abs(flHeading.getDegrees()), 12.0, "front-left should sit at 45 deg");
  }
}
