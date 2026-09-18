package first.robot;

import static org.junit.jupiter.api.Assertions.assertTrue;

import first.robot.subsystems.drive.Drive;
import first.robot.subsystems.drive.DriveConstants;
import first.robot.subsystems.drive.GyroIO;
import first.robot.subsystems.drive.ModuleIOSim;
import org.littletonrobotics.junction.Logger;
import org.wpilib.hardware.hal.HAL;
import org.wpilib.hardware.hal.RobotMode;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.kinematics.SwerveDriveKinematics;
import org.wpilib.simulation.DriverStationSim;

/**
 * Shared simulation fixture for the drive tests.
 *
 * <p>The HAL, the AdvantageKit {@code Logger}, and {@code Alert} ids are all JVM-wide, so all test
 * classes in this source set share one initialization and one {@link Drive} instance.
 */
public final class SimTestFixture {
  private static final SwerveDriveKinematics kinematics =
      new SwerveDriveKinematics(DriveConstants.moduleTranslations);

  private static Drive drive;

  private SimTestFixture() {}

  /** Initializes the HAL, logger, and drive exactly once per JVM. */
  public static synchronized Drive drive() {
    if (drive == null) {
      assertTrue(HAL.initialize(), "HAL failed to initialize");
      DriverStationSim.setRobotMode(RobotMode.TELEOPERATED);
      DriverStationSim.setEnabled(true);
      DriverStationSim.setDsAttached(true);
      DriverStationSim.notifyNewData();

      // AdvantageKit refuses to start outside a LoggedRobot; this is its supported escape hatch
      // for custom robot bases and tests.
      Logger.AdvancedHooks.disableRobotBaseCheck();
      Logger.disableConsoleCapture();
      Logger.start();

      drive =
          new Drive(
              new GyroIO() {},
              new ModuleIOSim(),
              new ModuleIOSim(),
              new ModuleIOSim(),
              new ModuleIOSim());
    }
    return drive;
  }

  /** Advances one AdvantageKit logging cycle so processInputs/recordOutput have a valid frame. */
  public static void tick() {
    Logger.AdvancedHooks.invokePeriodicBeforeUser();
    Logger.AdvancedHooks.invokePeriodicAfterUser(0, 0);
  }

  /**
   * Brings the drive to rest and zeroes its estimated pose, so each test starts from a known state
   * with the field frame and the robot frame aligned.
   */
  public static void reset() {
    Drive d = drive();
    for (int i = 0; i < 150; i++) {
      tick();
      d.periodic();
      d.stop();
    }
    tick();
    d.periodic();
    d.setPose(new Pose2d());
  }

  /** Robot-relative chassis velocity implied by the current module states. */
  public static ChassisVelocities measured() {
    return kinematics.toChassisVelocities(drive().getModuleVelocities());
  }
}
