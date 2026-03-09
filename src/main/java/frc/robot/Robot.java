// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import java.util.Optional;
import java.util.Random;
import java.util.random.RandomGenerator;
import frc.robot.commands.*;
import org.photonvision.*;

import com.pathplanner.lib.commands.FollowPathCommand;

import frc.robot.subsystems.DriveSubsystem;
import org.photonvision.*;
import org.photonvision.targeting.PhotonPipelineResult;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.RobotContainer;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  public static String alliance;

  private final RobotContainer m_robotContainer;


  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

    DriveSubsystem.m_gyro.reset();
    

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
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    //FollowPathCommand.warmupCommand().schedule();
  }

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the aut onomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    Optional<Alliance> ally = DriverStation.getAlliance();
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (ally.get() == Alliance.Red) {alliance = "red";}
    else if (ally.get() == Alliance.Red) {alliance = "blue";}
    //RobotContainer.rc_visionSS.results.clear();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    
    //RobotContainer.rc_visionSS.results.clear();
    Optional<Alliance> ally = DriverStation.getAlliance();
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (ally.get() == Alliance.Red) {alliance = "red";}
    else if (ally.get() == Alliance.Red) {alliance = "blue";}
    



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
     //   if (RobotContainer.m_driverController.a().getAsBoolean() && targetVisible) {
            // Driver wants auto-alignment to tag 7
            // And, tag 7 is in sight, so we can turn toward it.
            // Override the driver's turn command with an automatic one that turns toward the tag.
       //     turn = -1.0 * targetYaw * Constants.VisionConstants.visionTurnKP * Constants.DriveConstants.kMaxAngularSpeed;

          // WILL NEED TO CHANGE VISION TURN KP VALUE IN CONSTANTS

        
        //climb if requested, and a tag is in sight
        // else if (RobotContainer.m_driverController.povUp().getAsBoolean() == true && targetVisible == true);
        //   new ClimbC(RobotContainer.rc_climbSS);
  
        
          // Command drivetrain motors based on target speeds
          //DriveSubsystem.drive(forward, strafe, turn);

    //Put debug information to the dashboard
    //SmartDashboard.putBoolean("Vision Target Visible", targetVisible);
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
