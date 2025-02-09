package frc.robot.Subsystems.Simulation;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkAbsoluteEncoderSim;
import com.revrobotics.sim.SparkMaxAlternateEncoderSim;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.drivers.WarriorSparkMax;
import frc.lib.drivers.WarriorSparkMaxSim;
import frc.robot.Constants;
import edu.wpi.first.math.MathUtil;



public class SwerveModuleSim extends SubsystemBase {

    private WarriorSparkMaxSim driveSim;
    private WarriorSparkMaxSim turnSim;

    private SparkAbsoluteEncoderSim driveEncoderSim;
    private SparkAbsoluteEncoderSim turnEncoderSim;

    private Rotation2d lastAngle;
    private PIDController turnPIDController;

    private String key1_M, key2_M;
    
    private double turnRelativePositionRad = 0.0;
    private double turnAbsolutePositionRad = Math.random() * 2.0 * Math.PI;
    private double driveAppliedVolts = 0.0;
    private double turnAppliedVolts = 0.0;
    
    public SwerveModuleSim(
            int driveMotorId, 
            int turnMotorId, 
            boolean driveMotorReversed, 
            boolean turnMotorReversed
        ) {
        //System.out.println("[Init] Creating ModuleIOSim");
        driveSim = 
        new WarriorSparkMaxSim(
        new WarriorSparkMax(
                driveMotorId, 
                MotorType.kBrushless,
                driveMotorReversed, 
                IdleMode.kCoast, 
                Constants.SwerveConstants.DRIVE_CURRENT_LIMIT
            ), 
                DCMotor.getNEO(1)
            );
        turnSim = 
        new WarriorSparkMaxSim(
        new WarriorSparkMax(
                turnMotorId, 
                MotorType.kBrushless,
                turnMotorReversed, 
                IdleMode.kCoast, 
                Constants.SwerveConstants.DRIVE_CURRENT_LIMIT
            ), 
                DCMotor.getNEO(1)
            );

        driveEncoderSim = driveSim.getAbsoluteEncoderSim();
        turnEncoderSim = turnSim.getAbsoluteEncoderSim();

        turnPIDController = new PIDController(Constants.SwerveConstants.KP_TURNING, 0, 0.001);
        turnPIDController.enableContinuousInput(-Math.PI, Math.PI);

        key1_M = String.format("Drive Motor %s Velocity In/Actual", driveMotorId);
        key2_M = String.format("Turn Motor %s Velocity In/Actual", turnMotorId);

        SmartDashboard.putNumberArray(key1_M, new double[] {driveSim.getVelocity(), this.getDriveMotorVelocity(), driveSim.getPosition(), this.getDriveMotorPosition()});
        SmartDashboard.putNumberArray(key2_M, new double[] {turnSim.getVelocity(), this.getTurnMotorVelocity(), turnSim.getPosition(), this.getTurnMotorPosition()});



        lastAngle = getState().angle;
    }

    public void updateBoard() {
        SmartDashboard.putNumberArray(key1_M, new double[] {driveSim.getVelocity(), this.getDriveMotorVelocity(), driveSim.getPosition(), this.getDriveMotorPosition()});
        SmartDashboard.putNumberArray(key2_M, new double[] {turnSim.getVelocity(), this.getTurnMotorVelocity(), turnSim.getPosition(), this.getTurnMotorPosition()});
    }

    public double getDriveMotorPosition() {
        return driveSim.getAngle(); // * Constants.SwerveConstants.DRIVE_MOTOR_PCONVERSION;
    }

    public double getDriveMotorVelocity() {
        return driveSim.getVelocity();
    }

    public double getTurnMotorPosition() {
        return turnSim.getAngle(); // * Constants.SwerveConstants.TURN_MOTOR_PCONVERSION;
    }

    public double getTurnMotorVelocity() {
        return turnSim.getVelocity();
    }

    public double getAbsoluteEncoderAngle() {
        double angle = turnEncoderSim.getPosition();
        angle *= (Math.PI * 2);
        return angle;
        }

    public void resetEncoders() {
        driveEncoderSim.setPosition(0);
        turnEncoderSim.setPosition(
            (getAbsoluteEncoderAngle() / Constants.SwerveConstants.TURN_MOTOR_PCONVERSION));
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
            getDriveMotorVelocity(), new Rotation2d(getTurnMotorPosition()));
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            getDriveMotorPosition(), new Rotation2d(getTurnMotorPosition()));
    }

    public void stop() {
        driveSim.setVelocity(0);
        turnSim.setVelocity(0);
    }

    public void setSpeed(SwerveModuleState desiredState) {
        driveSim.setVelocity(
            desiredState.speedMetersPerSecond);
        }
    
    public void setAngle(SwerveModuleState desiredState) {
        turnSim.setAngle(desiredState.angle.getRadians());
    }

    public void setRobotPosition() {
        driveSim.setAngle(
            driveSim.getAngle() + driveSim.getVelocity() * driveSim.getInterval());
    }

    public double getInterval() {
        return turnSim.getInterval();
    }
    
    public void setDesiredState(SwerveModuleState desiredState) {
        // SmartDashboard.putNumber("Pre-optimized", desiredState.speedMetersPerSecond);
        desiredState = SwerveModuleState.optimize(desiredState, getState().angle);
        // SmartDashboard.putNumber("Post-optimized", desiredState.speedMetersPerSecond);
        setAngle(desiredState);
        setSpeed(desiredState);
        setRobotPosition();

        // SmartDashboard.putString("Swerve [" + driveMotor.getDeviceId() + "] State",
        // getState().toString());
    }
    
    public void setDriveVoltage(double volts) {
        driveAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
        driveSim.setMotorCurrent(driveAppliedVolts);
    }
    
    public void setTurnVoltage(double volts) {
        turnAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
        turnSim.setMotorCurrent(turnAppliedVolts);
    }
} 
