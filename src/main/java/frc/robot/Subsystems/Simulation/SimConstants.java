package frc.robot.Subsystems.Simulation;

public class SimConstants {
   public static class Swerve{
        public static double driveSpeed = 3.2;
        public static double driveAcceleraionSpeed = 7.5;
        public static double angularSpeed = (5*Math.PI)/1.75;
        public static double angularAccelationSpeed = 15;

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