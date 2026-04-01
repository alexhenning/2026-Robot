package frc.robot.commands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
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

import edu.wpi.first.math.*;

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
    public Pose2d visionPose;
    public double visionTimestamp;
    private final DriveSubsystem m_drive;
    private final VisionSS m_vision;
    
    // Sensor fusion state
    private Rotation2d m_imuToFieldOffset = new Rotation2d();  // Calibration offset
    private double m_lastVisionTimestamp = 0;
    private boolean m_hasValidOffset = false;
    
    // P controller gain (students: try a full PID!)
    private static final double kP = 0.02;
    
    // How old can vision data be before we stop trusting it?
    private static final double kVisionTimeoutSeconds = 0.5;
    
    public AutoAlignC(DriveSubsystem drive, VisionSS vision) {
        m_drive = drive;
        m_vision = vision;
        addRequirements(drive);
    }
    
    @Override
    public void initialize() {
        m_hasValidOffset = false;  // Recalibrate when command starts
    }
    
    @Override
    public void execute() {
        // === STEP 1: Update offset when new vision data arrives ===
        Optional<EstimatedRobotPose> visionResult = m_vision.getRobotPose();
        
        if (visionResult.isPresent()) {
            EstimatedRobotPose estimatedVisionPose = visionResult.get();
            Pose2d visionPose = estimatedVisionPose.estimatedPose.toPose2d();
            double visionTimestamp = estimatedVisionPose.timestampSeconds;
            
            // Only update if this is NEW vision data
            if (visionTimestamp > m_lastVisionTimestamp) {
                m_lastVisionTimestamp = visionTimestamp;
                
                // Vision tells us our TRUE heading in field coordinates
                Rotation2d visionHeading = visionPose.getRotation();
                
                // IMU tells us heading in its own reference frame
                Rotation2d imuHeading = Rotation2d.fromDegrees(m_drive.getHeading());
                
                // Calculate the magic offset: field = imu + offset
                // So: offset = field - imu
                m_imuToFieldOffset = visionHeading.minus(imuHeading);
                m_hasValidOffset = true;
                
                SmartDashboard.putNumber("Vision/IMU Offset (deg)", m_imuToFieldOffset.getDegrees());
            }
        }
        
        // === STEP 2: Check if we have valid calibration ===
        if (!m_hasValidOffset) {
            // No vision yet - can't auto-align, just pass through driver input
            driveWithDriverInput(0);  // No auto-rotation
            return;
        }
        
        // Check for stale vision (optional safety)
        double timeSinceVision = Timer.getFPGATimestamp() - m_lastVisionTimestamp;
        if (timeSinceVision > kVisionTimeoutSeconds) {
            SmartDashboard.putBoolean("AutoAlign/VisionStale", true);
            // Could choose to stop auto-aligning here, or keep using last offset
        }
        
        // === STEP 3: Estimate current field heading using IMU + offset ===
        // This runs at full robot speed (~50Hz) even when vision is slow!
        Rotation2d imuHeading = Rotation2d.fromDegrees(m_drive.getHeading());
        Rotation2d estimatedFieldHeading = imuHeading.plus(m_imuToFieldOffset);
        
        SmartDashboard.putNumber("AutoAlign/EstimatedHeading", estimatedFieldHeading.getDegrees());
        
        // === STEP 4: Calculate desired heading to target ===
        Translation2d targetPos = getTargetPosition();  // Hub location
        Translation2d robotPos = getLastKnownPosition(); // From vision
        
        desiredHeading = targetPos.minus(robotPos).getAngle();
        
        // === STEP 5: Compute error and rotation command ===
        Rotation2d headingError = desiredHeading.minus(estimatedFieldHeading);
        double rotationCommand = MathUtil.clamp(headingError.getRadians() * kP, -1.0, 1.0);
        
        SmartDashboard.putNumber("AutoAlign/HeadingError", headingError.getDegrees());
        
        // === STEP 6: Drive! ===
        //DriveSubsystem.aligntoHub(desiredHeading);
        driveWithDriverInput(rotationCommand);

    }
    
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
        DriveSubsystem.aligntoHub(desiredHeading);
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
        m_drive.drive(0, 0, 0, false);
    }
    
    @Override
    public boolean isFinished() {
        return false;
    }
}
