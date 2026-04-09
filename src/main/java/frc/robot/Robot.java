// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import java.util.Optional;
import java.util.Random;
import java.util.random.RandomGenerator;
import frc.robot.commands.*;
import org.photonvision.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathPlannerAuto;

import org.photonvision.*;
import org.photonvision.targeting.PhotonPipelineResult;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.RobotContainer;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  public static String alliance = "blue";

  private final RobotContainer m_robotContainer;

  public static double currenttime;
  public Rotation2d desiredHeading;
  public static double rcdesiredHeading;
  public double desiredState;
  public Pose2d visionPose;
  public double visionTimestamp;
  private PIDController turnController;
  public boolean skipperIsControllingHimselfAgainOhNoOhCrapOhDarnOhDearWaitNoItsFineWeActuallyTrustHimIfHeDoesntDoAnythingDumbHaveFunSkipper;
  // Sensor fusion state
  private Rotation2d m_imuToFieldOffset = new Rotation2d();  // Calibration offset
  private double m_lastVisionTimestamp = 0;
  private boolean m_hasValidOffset = false;
  public Optional<EstimatedRobotPose> visionResult;
  public Rotation2d imuHeading;
  public double rotationCommand;
  public Rotation2d headingError;
  public Translation2d robotPos;
  public Translation2d targetPos;
  public Rotation2d estimatedFieldHeading;
  public double timeSinceVision;
  public Rotation2d visionHeading;
  public EstimatedRobotPose estimatedVisionPose;
  public double time;

    
  // P controller gain (students: try a full PID!)
  private static final double kP = 0.02;
    
  // How old can vision data be before we stop trusting it?
  private static final double kVisionTimeoutSeconds = 0.5;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

    DriveSubsystem.m_gyro.reset();

    FollowPathCommand.warmupCommand().schedule();
    

  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();

    // boolean skipperIsControllingHimselfAgainOhNoOhCrapOhDarnOhDearWaitNoItsFineWeActuallyTrustHimIfHeDoesntDoAnythingDumbHaveFunSkipper = RobotContainer.rc_autoAlignC.skipperIsControllingHimselfAgainOhNoOhCrapOhDarnOhDearWaitNoItsFineWeActuallyTrustHimIfHeDoesntDoAnythingDumbHaveFunSkipper;
    
    // if (skipperIsControllingHimselfAgainOhNoOhCrapOhDarnOhDearWaitNoItsFineWeActuallyTrustHimIfHeDoesntDoAnythingDumbHaveFunSkipper) {
    //   SmartDashboard.putString("Auto Align Engaged?",  "YES");
    // }
    // else {
    //   SmartDashboard.putString("Auto Align Engaged?",  "NO");
    // }

    SmartDashboard.putNumber("RCDesiredHeading", rcdesiredHeading);

  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    //FollowPathCommand.warmupCommand().schedule();
  }

  @Override
  public void disabledPeriodic() {
    
  }

  /** This autonomous runs the aut onomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    Optional<Alliance> ally = DriverStation.getAlliance();
    
    if (ally.get() == Alliance.Red) {alliance = "red";}
    else if (ally.get() == Alliance.Blue) {alliance = "blue";}
    RobotContainer.rc_visionSS.driveCamera.setFPSLimit(30);
    RobotContainer.rc_visionSS.camera.setFPSLimit(30);
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    

    //RobotContainer.rc_visionSS.results.clear();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
      double time = Timer.getFPGATimestamp();
      double currenttime = RobotContainer.rc_basicAutoC.starttime - time;

      SmartDashboard.putNumber("Auto Timer: ", currenttime);
      if (RobotContainer.rc_visionSS.getRobotPose() == null) {
        SmartDashboard.putBoolean("Apriltag Detected", false);
      }
      else {
      visionResult = RobotContainer.rc_visionSS.getRobotPose();
      SmartDashboard.putBoolean("Apriltag Detected", true);
            EstimatedRobotPose estimatedVisionPose = visionResult.get();
            Pose2d visionPose = estimatedVisionPose.estimatedPose.toPose2d();
            double visionTimestamp = estimatedVisionPose.timestampSeconds;
            
            // Only update if this is NEW vision data
            if (visionTimestamp > m_lastVisionTimestamp) {
                m_lastVisionTimestamp = visionTimestamp;
                
                // Vision tells us our TRUE heading in field coordinates
                Rotation2d visionHeading = visionPose.getRotation();
                
                // IMU tells us heading in its own reference frame
                imuHeading = Rotation2d.fromDegrees(RobotContainer.m_robotDrive.getHeading());
                
                // Calculate the magic offset: field = imu + offset
                // So: offset = field - imu
                m_imuToFieldOffset = visionHeading.minus(imuHeading);
                m_hasValidOffset = true;
                
                SmartDashboard.putNumber("Vision/IMU Offset (deg)", m_imuToFieldOffset.getDegrees());
            
        }
        
        // === STEP 2: Check if we have valid calibration ===
        
        // Check for stale vision (optional safety)
        double timeSinceVision = Timer.getFPGATimestamp() - m_lastVisionTimestamp;
        if (timeSinceVision > kVisionTimeoutSeconds) {
            SmartDashboard.putBoolean("AutoAlign/VisionStale", true);
            // Could choose to stop auto-aligning here, or keep using last offset
        }
        
        // === STEP 3: Estimate current field heading using IMU + offset ===
        // This runs at full robot speed (~50Hz) even when vision is slow!
        imuHeading = Rotation2d.fromDegrees(RobotContainer.m_robotDrive.getHeading());
        Rotation2d estimatedFieldHeading = imuHeading.plus(m_imuToFieldOffset);
        
        SmartDashboard.putNumber("AutoAlign/EstimatedHeading", estimatedFieldHeading.getDegrees());
        
        // === STEP 4: Calculate desired heading to target ===
        Translation2d targetPos = RobotContainer.rc_visionSS.getTargetPosition();  // Hub location
        Translation2d robotPos = RobotContainer.rc_visionSS.getLastKnownPosition(); // From vision
        
        desiredHeading = targetPos.minus(robotPos).getAngle();
        rcdesiredHeading = desiredHeading.getDegrees();

        
        // === STEP 5: Compute error and rotation command ===
        Rotation2d headingError = desiredHeading.minus(estimatedFieldHeading);
        double rotationCommand = MathUtil.clamp(headingError.getRadians() * kP, -1.0, 1.0);
        
        SmartDashboard.putNumber("AutoAlign/HeadingError", headingError.getDegrees());
      }

    }
  
     


  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    
    //RobotContainer.rc_visionSS.results.clear();
    int limit = RobotContainer.rc_visionSS.driveCamera.getFPSLimit();

    RobotContainer.rc_visionSS.driveCamera.setFPSLimit(30);
    RobotContainer.rc_visionSS.camera.setFPSLimit(30);
    SmartDashboard.putNumber("FPS Limit", limit);
    SmartDashboard.putNumber("Vision/IMU Offset (deg)", 0);
    SmartDashboard.putNumber("AutoAlign/EstimatedHeading", 0);
    SmartDashboard.putNumber("AutoAlign/HeadingError", 0);
    
    Optional<Alliance> ally = DriverStation.getAlliance();

    if (ally.get() == Alliance.Red) {alliance = "red";}
    else if (ally.get() == Alliance.Blue) {alliance = "blue";}
    



    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    // Calculate drivetrain commands from Joystick values
        double forward = RobotContainer.m_driverController.getLeftY() * Constants.DriveConstants.kMaxLinearSpeed;
        double strafe = -RobotContainer.m_driverController.getLeftX() * Constants.DriveConstants.kMaxLinearSpeed;
        double turn = -RobotContainer.m_driverController.getRightX() * Constants.DriveConstants.kMaxAngularSpeed;




        // Auto-align when requested
      if (RobotContainer.rc_visionSS.getRobotPose() == null) {
        SmartDashboard.putBoolean("Apriltag Detected?", false);
      }
      else {
        SmartDashboard.putBoolean("Apriltag Detected?", true);
  
          visionResult = RobotContainer.rc_visionSS.getRobotPose();
          estimatedVisionPose = visionResult.get();
          visionPose = estimatedVisionPose.estimatedPose.toPose2d();
          visionTimestamp = estimatedVisionPose.timestampSeconds;
          
          // Only update if this is NEW vision data
          if (visionTimestamp > m_lastVisionTimestamp) {
              m_lastVisionTimestamp = visionTimestamp;
              
              // Vision tells us our TRUE heading in field coordinates
              visionHeading = visionPose.getRotation();
              
              // IMU tells us heading in its own reference frame
              imuHeading = Rotation2d.fromDegrees(RobotContainer.m_robotDrive.getHeading());
              
              // Calculate the magic offset: field = imu + offset
              // So: offset = field - imu
              m_imuToFieldOffset = visionHeading.minus(imuHeading);
              m_hasValidOffset = true;
              
              SmartDashboard.putNumber("Vision/IMU Offset (deg)", m_imuToFieldOffset.getDegrees());
          }

      
      
      // === STEP 2: Check if we have valid calibration ===
      
      // Check for stale vision (optional safety)
      timeSinceVision = Timer.getFPGATimestamp() - m_lastVisionTimestamp;
      if (timeSinceVision > kVisionTimeoutSeconds) {
          SmartDashboard.putBoolean("AutoAlign/VisionStale", true);
          // Could choose to stop auto-aligning here, or keep using last offset
      }
      
      // === STEP 3: Estimate current field heading using IMU + offset ===
      // This runs at full robot speed (~50Hz) even when vision is slow!
      imuHeading = Rotation2d.fromDegrees(RobotContainer.m_robotDrive.getHeading());
      estimatedFieldHeading = imuHeading.plus(m_imuToFieldOffset);
      
      SmartDashboard.putNumber("AutoAlign/EstimatedHeading", estimatedFieldHeading.getDegrees());
      
      // === STEP 4: Calculate desired heading to target ===
      targetPos = RobotContainer.rc_visionSS.getTargetPosition();  // Hub location
      robotPos = RobotContainer.rc_visionSS.getLastKnownPosition(); // From vision
      
      desiredHeading = targetPos.minus(robotPos).getAngle();
      rcdesiredHeading = desiredHeading.getDegrees();

      
      // === STEP 5: Compute error and rotation command ===
      headingError = desiredHeading.minus(estimatedFieldHeading);
       rotationCommand = MathUtil.clamp(headingError.getRadians() * kP, -1.0, 1.0);
      
      SmartDashboard.putNumber("AutoAlign/HeadingError", headingError.getDegrees());
    }
    }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}



  

}
