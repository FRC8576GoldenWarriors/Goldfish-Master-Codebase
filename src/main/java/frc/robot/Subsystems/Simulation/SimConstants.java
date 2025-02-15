package frc.robot.Subsystems.Simulation;

public class SimConstants {

  public static final Mode currentMode = Mode.REAL;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static class Swerve {
    public static double driveSpeed = 3.2;
    public static double driveAcceleraionSpeed = 7.5;
    public static double angularSpeed = (5 * Math.PI) / 1.75;
    public static double angularAccelationSpeed = 15;

    public static final int LEFT_FRONT_DRIVE_ID = 52;
    public static final int RIGHT_FRONT_DRIVE_ID = 54;
    public static final int LEFT_BACK_DRIVE_ID = 56;
    public static final int RIGHT_BACK_DRIVE_ID = 58;

    public static final int LEFT_FRONT_TURN_ID = 51;
    public static final int RIGHT_FRONT_TURN_ID = 53;
    public static final int LEFT_BACK_TURN_ID = 55;
    public static final int RIGHT_BACK_TURN_ID = 57;
  }

  public static class DriverConstants {
    public static final boolean isSimMode = true;
    public static final double xDeadband = 0.0825;
    public static final double yDeadband = 0.0825;
    public static final double turnDeadband = 0.0825;
    public static final double xCoefficient = 2.05;
    public static final double yCoefficient = 2.05;
    public static final double turnCoefficient = 1.675;
  }
}
