package first.robot.commands;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import first.robot.SimTestFixture;
import first.robot.subsystems.drive.Drive;
import first.robot.subsystems.drive.DriveConstants;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.wpilib.command2.Command;
import org.wpilib.math.kinematics.ChassisVelocities;

/**
 * Verifies the driver-facing path: gamepad axis values -> {@code DriveCommands.joystickDrive} ->
 * chassis motion. Stick values here are already negated the same way {@code RobotContainer} negates
 * them, so {@code stickX = +1} means "left stick pushed fully forward".
 *
 * <p>{@link SimTestFixture#reset()} zeroes the estimated heading before each test, so the
 * field-relative command frame and the robot-relative measurement frame line up.
 */
class JoystickDriveTest {
  private Drive drive;

  // Mutable stick state, read by the command's suppliers.
  private double stickX;
  private double stickY;
  private double stickOmega;

  @BeforeEach
  void setUp() {
    drive = SimTestFixture.drive();
    stickX = 0.0;
    stickY = 0.0;
    stickOmega = 0.0;
    SimTestFixture.reset();
  }

  /** Runs the joystick drive command for the given number of robot loops. */
  private ChassisVelocities run(int loops) {
    Command command =
        DriveCommands.joystickDrive(drive, () -> stickX, () -> stickY, () -> stickOmega);
    command.initialize();
    for (int i = 0; i < loops; i++) {
      SimTestFixture.tick();
      drive.periodic();
      command.execute();
    }
    SimTestFixture.tick();
    drive.periodic();
    return SimTestFixture.measured();
  }

  @Test
  void stickForwardDrivesForward() {
    stickX = 1.0;
    ChassisVelocities measured = run(200);
    assertTrue(
        measured.vx > DriveConstants.maxLinearSpeed * 0.7,
        "full forward stick should drive near max speed, got " + measured.vx);
    assertEquals(0.0, measured.vy, 0.3, "should not drift sideways");
    assertEquals(0.0, measured.omega, 0.3, "should not rotate");
  }

  @Test
  void stickBackDrivesBackward() {
    stickX = -1.0;
    ChassisVelocities measured = run(200);
    assertTrue(
        measured.vx < -DriveConstants.maxLinearSpeed * 0.7,
        "full back stick should drive backward, got " + measured.vx);
    assertEquals(0.0, measured.vy, 0.3, "should not drift sideways");
  }

  @Test
  void stickLeftStrafesLeft() {
    stickY = 1.0;
    ChassisVelocities measured = run(200);
    assertTrue(
        measured.vy > DriveConstants.maxLinearSpeed * 0.7,
        "full left stick should strafe left (+y), got " + measured.vy);
    assertEquals(0.0, measured.vx, 0.3);
  }

  @Test
  void stickRightStrafesRight() {
    stickY = -1.0;
    ChassisVelocities measured = run(200);
    assertTrue(
        measured.vy < -DriveConstants.maxLinearSpeed * 0.7,
        "full right stick should strafe right (-y), got " + measured.vy);
    assertEquals(0.0, measured.vx, 0.3);
  }

  @Test
  void rightStickRotatesCounterClockwise() {
    stickOmega = 1.0;
    ChassisVelocities measured = run(200);
    assertTrue(
        measured.omega > drive.getMaxAngularSpeedRadPerSec() * 0.5,
        "full rotation stick should spin CCW, got " + measured.omega);
  }

  @Test
  void rightStickRotatesClockwise() {
    stickOmega = -1.0;
    ChassisVelocities measured = run(200);
    assertTrue(
        measured.omega < -drive.getMaxAngularSpeedRadPerSec() * 0.5,
        "negative rotation stick should spin CW, got " + measured.omega);
  }

  /** Small stick noise must not move the robot. */
  @Test
  void deadbandIgnoresSmallInputs() {
    stickX = 0.05;
    stickY = 0.05;
    stickOmega = 0.05;
    ChassisVelocities measured = run(150);
    assertEquals(0.0, measured.vx, 0.1, "inside the deadband the robot should hold still");
    assertEquals(0.0, measured.vy, 0.1);
    assertEquals(0.0, measured.omega, 0.1);
  }

  /** Diagonal stick input should drive along that diagonal. */
  @Test
  void diagonalStickDrivesDiagonally() {
    stickX = 0.7071;
    stickY = 0.7071;
    ChassisVelocities measured = run(220);
    assertTrue(measured.vx > 1.0, "expected forward motion, got " + measured.vx);
    assertTrue(measured.vy > 1.0, "expected leftward motion, got " + measured.vy);
    assertEquals(
        measured.vx, measured.vy, 0.4, "a 45 degree stick should drive along the 45 degree line");
  }

  /** Half stick should be clearly slower than full stick (the squared response curve). */
  @Test
  void halfStickIsSlowerThanFullStick() {
    stickX = 0.5;
    double half = run(200).vx;
    SimTestFixture.reset();
    stickX = 1.0;
    double full = run(200).vx;
    assertTrue(half > 0.1, "half stick should still move the robot, got " + half);
    assertTrue(half < full * 0.6, "half stick (" + half + ") should be well under full (" + full + ")");
  }
}
