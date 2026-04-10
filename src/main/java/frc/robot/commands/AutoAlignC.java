package frc.robot.commands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.Constants.OIConstants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.*;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import com.fasterxml.jackson.databind.introspect.TypeResolutionContext.Empty;
import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.math.*;
import edu.wpi.first.math.controller.PIDController;

/**
 * AUTO-ALIGN WITH SENSOR FUSION
 * 
 * Vision: Accurate but slow (updates every 50-100ms, plus processing latency)
 * IMU: Fast but doesn't know "field north" - only tracks CHANGES in heading
 * 
 * SOLUTION: When vision updates, calculate the offset between IMU and field.
 *           Between updates, use IMU + offset for fast, accurate heading.
 * 
 *   fieldHeading = imuHeading + offset
 *   
 *   When vision arrives:
 *     offset = visionHeading - imuHeading
 */
public class AutoAlignC extends Command {
    public Rotation2d desiredHeading;
    public double rcdesiredheading;
    public Pose2d visionPose;
    public double visionTimestamp;
    private PIDController turnController;
    private final DriveSubsystem m_drive;
    private final VisionSS m_vision;
    public boolean skipperIsControllingHimselfAgainOhNoOhCrapOhDarnOhDearWaitNoItsFineWeActuallyTrustHimIfHeDoesntDoAnythingDumbHaveFunSkipper;
    // Sensor fusion state
    private Rotation2d m_imuToFieldOffset = new Rotation2d();  // Calibration offset
    private double m_lastVisionTimestamp = 0;
    private boolean m_hasValidOffset = false;
    
    // P controller gain (students: try a full PID!)
    private static final double kP = 0.02;
    
    // How old can vision data be before we stop trusting it?
    private static final double kVisionTimeoutSeconds = 0.5;
    
    public AutoAlignC(DriveSubsystem drive, VisionSS vision) {
        super();
        m_drive = drive;
        m_vision = vision;
        addRequirements(drive); 
        turnController = new PIDController(0, 0, 0);
        turnController.setTolerance(0);
    }
    public class PID {
        public static PIDController rotationPID = getRotationPID();
        private static PIDController getRotationPID() {
            PIDController pid = new PIDController(
                    Constants.PIDConstants.kDriveRotationP,
                    Constants.PIDConstants.kDriveRotationI,
                    Constants.PIDConstants.kDriveRotationD);
            pid.setTolerance(Constants.PIDConstants.kDriveRotationT);
            pid.enableContinuousInput(Constants.GyroConstants.kAlsoSS, Constants.GyroConstants.kSS);
            return pid;
        }
}



    @Override
    public void initialize() {
        m_hasValidOffset = false;  // Recalibrate when command starts
        m_lastVisionTimestamp = 0;
        SmartDashboard.putNumber("Vision/IMU Offset (deg)", 0);
        SmartDashboard.putNumber("AutoAlign/EstimatedHeading", 0);
        SmartDashboard.putNumber("AutoAlign/HeadingError", 0);
        SmartDashboard.putNumber("AutoAlign/TimeSinceVision", 0);
    }
    
    @Override
    public void execute() {
        // === STEP 1: Update offset when new vision data arrives ===
        Optional<EstimatedRobotPose> visionResult = m_vision.getRobotPose();

        if (visionResult != null && visionResult.isPresent()) {
            EstimatedRobotPose estimatedVisionPose = visionResult.get();
            Pose2d visionPose = estimatedVisionPose.estimatedPose.toPose2d();
            double visionTimestamp = estimatedVisionPose.timestampSeconds;

            if (visionTimestamp > m_lastVisionTimestamp) {
                m_lastVisionTimestamp = visionTimestamp;
                Rotation2d visionHeading = visionPose.getRotation();
                Rotation2d imuHeading = Rotation2d.fromDegrees(m_drive.getHeading());
                m_imuToFieldOffset = visionHeading.minus(imuHeading);
                m_hasValidOffset = true;
            }
        }

        // === STEP 2: Always show time since last vision ===
        double timeSinceVision = Timer.getFPGATimestamp() - m_lastVisionTimestamp;
        SmartDashboard.putNumber("AutoAlign/TimeSinceVision", timeSinceVision);
        SmartDashboard.putNumber("Vision/IMU Offset (deg)", m_imuToFieldOffset.getDegrees());

        if (!m_hasValidOffset) {
            driveWithDriverInput(0);
            return;
        }

        // === STEP 3: Estimate current field heading using IMU + offset ===
        Rotation2d imuHeading = Rotation2d.fromDegrees(m_drive.getHeading());
        Rotation2d estimatedFieldHeading = imuHeading.plus(m_imuToFieldOffset);
        SmartDashboard.putNumber("AutoAlign/EstimatedHeading", estimatedFieldHeading.getDegrees());

        // === STEP 4: Calculate desired heading to target ===
        Translation2d targetPos = m_vision.getTargetPosition();
        Translation2d robotPos = m_vision.getLastKnownPosition();
        desiredHeading = targetPos.minus(robotPos).getAngle();
        Robot.rcdesiredHeading = desiredHeading.getDegrees();

        // === STEP 5: Set PID setpoint live and drive ===
        PID.rotationPID.setSetpoint(desiredHeading.getDegrees());
        SmartDashboard.putNumber("AutoAlign/HeadingError",
            desiredHeading.minus(estimatedFieldHeading).getDegrees());

        driveWithDriverInput(-PID.rotationPID.calculate(
            DriveSubsystem.m_gyro.getYaw().getValue().in(Units.Degrees)));
        skipperIsControllingHimselfAgainOhNoOhCrapOhDarnOhDearWaitNoItsFineWeActuallyTrustHimIfHeDoesntDoAnythingDumbHaveFunSkipper = true;
    }
//}
    private void driveWithDriverInput(double autoRotation) {
        m_drive.drive(
            -MathUtil.applyDeadband(
                RobotContainer.m_driverController.getLeftY(), 
                OIConstants.kDriveDeadband),
            -MathUtil.applyDeadband(
                RobotContainer.m_driverController.getLeftX(), 
                OIConstants.kDriveDeadband),
            autoRotation,
            true
        );
    }
    
    private Translation2d getTargetPosition() {
        // Use alliance color to pick target
        if (RobotContainer.rc_visionSS.isblue) {
            return new Translation2d(Constants.HubCoords.blueHubX, Constants.HubCoords.blueHubY);
        } else {
            return new Translation2d(Constants.HubCoords.redHubX, Constants.HubCoords.redHubY);
        }
    }
    
    private Translation2d getLastKnownPosition() {
        // Use last vision position for target angle calculation
        // (Could also use odometry here for even better fusion)
        return new Translation2d(
            RobotContainer.rc_visionSS.RobotX,
            RobotContainer.rc_visionSS.RobotY
        );
    }
    
    @Override
    public void end(boolean interrupted) {
        m_drive.drive(0, 0, 0, true);
        skipperIsControllingHimselfAgainOhNoOhCrapOhDarnOhDearWaitNoItsFineWeActuallyTrustHimIfHeDoesntDoAnythingDumbHaveFunSkipper = false;

    }
    
    @Override
    public boolean isFinished() {
        return false;
    }
}
