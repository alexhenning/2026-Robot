// Josh was here
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// Access the hidden files
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.IOException;
import java.util.Optional;
import java.util.function.Supplier;
import org.json.simple.parser.ParseException;
import org.photonvision.*;
import org.photonvision.targeting.PhotonPipelineResult;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.CANIDConstants;
import frc.robot.Constants.DriveConstants;
public class DriveSubsystem extends SubsystemBase {



  // Create MAXSwerveModules
  public final MAXSwerveModule m_frontLeft = 
      new MAXSwerveModule(
          CANIDConstants.kFrontLeftDrivingCanId,
          CANIDConstants.kFrontLeftTurningCanId,
          DriveConstants.kFrontLeftChassisAngularOffset);

  public final MAXSwerveModule m_frontRight =
      new MAXSwerveModule(
          CANIDConstants.kFrontRightDrivingCanId,
          CANIDConstants.kFrontRightTurningCanId,
          DriveConstants.kFrontRightChassisAngularOffset);

  public final MAXSwerveModule m_rearLeft =
      new MAXSwerveModule(
          CANIDConstants.kRearLeftDrivingCanId,
          CANIDConstants.kRearLeftTurningCanId,
          DriveConstants.kBackLeftChassisAngularOffset);

  public final MAXSwerveModule m_rearRight =
      new MAXSwerveModule(
          CANIDConstants.kRearRightDrivingCanId,
          CANIDConstants.kRearRightTurningCanId,
          DriveConstants.kBackRightChassisAngularOffset);




          

  // The gyro sensor
  public static final Pigeon2 m_gyro = new Pigeon2(0);

  // Odometry class for tracking robot pose
  public SwerveDriveOdometry m_odometry =
      new SwerveDriveOdometry(
          DriveConstants.kDriveKinematics,
          Rotation2d.fromDegrees(m_gyro.getYaw().getValue().in(Units.Degrees) % 360),
          new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
          });


  public DriveSubsystem() {
    // All other subsystem initialization
    // ...
    // Load the RobotConfig from the GUI settings. You should probably
    // store this in your Constants file
    RobotConfig config;
    try {
      config = RobotConfig.fromGUISettings();

    // Configure AutoBuilder last
    AutoBuilder.configure(
        this::getPose, // Robot pose supplier
        this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting
        // pose)
        this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        (speeds, feedforwards) ->
            driveRobotRelative(
                speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds.
        // Also optionally outputs individual module feedforwards
        new PPHolonomicDriveController( // PPHolonomicController is the built in path following
            // controller for holonomic drive trains
            new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
            new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
            ),
        config, // The robot configuration
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        this // Reference to this subsystem to set requirements
        );
        
      } catch (IOException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    } catch (ParseException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    }



  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(
        Rotation2d.fromDegrees(m_gyro.getYaw().getValue().in(Units.Degrees) % 360),
        new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
        });
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    //System.out.println(m_odometry.getPoseMeters());
    return m_odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(
        Rotation2d.fromDegrees(m_gyro.getYaw().getValue().in(Units.Degrees) % 360),
        new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
        },
        pose);
      System.out.printf("Reset Odometry: %s -> %s\n", pose, getPose());
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    // System.out.println(xSpeed);
    // System.out.println(ySpeed);
    // System.out.println(rot);
    // System.out.println(fieldRelative);
    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = 2 * xSpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = 2 * ySpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double rotDelivered = rot * DriveConstants.kMaxAngularSpeed;

    var swerveModuleStates =
        DriveConstants.kDriveKinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeedDelivered,
                    ySpeedDelivered,
                    rotDelivered,
                    Rotation2d.fromDegrees(m_gyro.getYaw().getValue().in(Units.Degrees)-360))
                : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  public void driveRobotRelative(ChassisSpeeds cs) {
    drive(cs.vxMetersPerSecond, cs.vyMetersPerSecond, cs.omegaRadiansPerSecond, false);
  }

  /** Sets the wheels into an X formation to prevent movement. */
  public Command setXCommand() {
        return this.run(
        () -> {
          m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
          m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
          m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
          m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
        });
  //   if (RobotContainer.m_driverController.rightBumper().getAsBoolean() && RobotContainer.m_operatorController.leftStick().getAsBoolean()) {
  //   return this.run(
  //       () -> {
  //         m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  //         m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
  //         m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
  //         m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  //       });
  // }
  //   else if (RobotContainer.m_driverController.rightBumper().getAsBoolean() == false && RobotContainer.m_driverController.leftStick().getAsBoolean() == true) {
  //     return this.run(
  //       () -> {
  //         m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  //         m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
  //         m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
  //         m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  //       });
  //     }
  //   else return null;
      
}


  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public Command zeroHeadingCommand() {
    return this.runOnce(() -> m_gyro.reset());
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return Rotation2d.fromDegrees(m_gyro.getYaw().getValue().in(Units.Degrees)).getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getAngularVelocityZWorld().getValue().in(Units.DegreesPerSecond) * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  public void resetPose(Pose2d pose) {
    // m_odometry.resetPosition(
    //     Rotation2d.fromDegrees(m_gyro.getYaw().getValue().in(Units.Degrees) % 360),
    //     new SwerveModulePosition[] {
    //       m_frontLeft.getPosition(),
    //       m_frontRight.getPosition(),
    //       m_rearLeft.getPosition(),
    //       m_rearRight.getPosition()
    //     },
    //     pose);
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    return DriveConstants.kDriveKinematics.toChassisSpeeds(
        m_frontLeft.getState(),
        m_frontRight.getState(),
        m_rearLeft.getState(),
        m_rearRight.getState());
  }
  public double getBlueHubAngle() {
    double dydx = ((RobotContainer.rc_visionSS.RobotY - Constants.HubCoords.blueHubY)/(RobotContainer.rc_visionSS.RobotX - Constants.HubCoords.blueHubX));
    double theta = Math.atan(dydx);
    return theta;
  }

    public double getRedHubAngle() {
    double dydx = ((RobotContainer.rc_visionSS.RobotY - Constants.HubCoords.redHubY)/(RobotContainer.rc_visionSS.RobotX - Constants.HubCoords.redHubX));
    double theta = Math.atan(dydx);
    return theta;
  }
  
  


  
}
