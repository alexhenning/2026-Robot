// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OIConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.*;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.*;


public class RobotContainer {

  // The robot's subsystems and commands are defined here...
  private static final DriveSubsystem m_robotDrive = new DriveSubsystem();
  //public static final VisionSS rc_visionSS = new VisionSS();
  public static final IntakeTiltSS rc_IntakeTiltSS = new IntakeTiltSS();
  public static final IntakeSS rc_intakeSS = new IntakeSS();
  public static final LauncherSS rc_launcherSS = new LauncherSS();
  public static final ClimbSS rc_climbSS = new ClimbSS();
  public static final KickerSS rc_KickerSS = new KickerSS();
  public static final ChangeModeSS rc_changeModeSS = new ChangeModeSS();

  public static final IntakeC rc_intakeC = new IntakeC(rc_intakeSS);
  //public static final LauncherC rc_launcherC = new LauncherC(rc_launcherSS);
  public static final LauncherSpeedC rc_launcherspeedC = new LauncherSpeedC(rc_launcherSS, 0.5);
  public static final ClimbC rc_climbC = new ClimbC(rc_climbSS);
  public static final KickerC rc_KickerC = new KickerC(rc_KickerSS);
  public static final StaticLauncherC rc_staticLauncherC = new StaticLauncherC(rc_launcherSS);
  public static final ChangeModeC rc_changeModeC = new ChangeModeC(rc_changeModeSS);
  public static final IntakeTiltC rc_intakeTiltC = new IntakeTiltC(rc_IntakeTiltSS);
  public static final ManualLauncherC rc_manualLauncherC = new ManualLauncherC(rc_launcherSS);

  public static final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
  public static final CommandXboxController m_operatorController =
      new CommandXboxController(OperatorConstants.kOperatorControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    //  rc_visionSS.setDefaultCommand(
    //   new RunCommand(
    //       () ->
    //         rc_visionSS.PrintTarget(),
    //         rc_visionSS
            
    //   )
    // );
    // Configure the trigger bindings
    configureBindings();
    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () ->
                m_robotDrive.drive(
                    -MathUtil.applyDeadband(
                        m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                    -MathUtil.applyDeadband(
                        m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                    -MathUtil.applyDeadband(
                        m_driverController.getRightX(), OIConstants.kDriveDeadband),
                    true),
            m_robotDrive));
    // Configure the trigger bindings
    configureBindings();
    // Configure default commands

  }
  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */
  private void configureBindings() {
    // Driver controller button commands
    m_driverController.leftStick().whileTrue(m_robotDrive.setXCommand());
    m_driverController.start().onTrue(m_robotDrive.zeroHeadingCommand());
    //.a().onTrue(rc_changeModeC);

    // Operator controller button commands

    m_driverController.a().whileTrue(rc_climbC);
    m_driverController.b().whileTrue(rc_climbC);

    m_operatorController.povRight().whileTrue(rc_intakeTiltC);
    m_operatorController.povLeft().whileTrue(rc_intakeTiltC);

    m_operatorController.leftBumper().whileTrue(rc_intakeC);
    m_operatorController.povDown().whileTrue(rc_intakeC);
    m_operatorController.leftTrigger().whileTrue(rc_KickerC);
    m_operatorController.rightTrigger().whileTrue(rc_KickerC);
    m_operatorController.rightBumper().whileTrue(rc_KickerC);
      
    // if (rc_changeModeSS.manual == false) {
    //   m_operatorController.rightTrigger().whileTrue(rc_staticLauncherC);
    //   //m_operatorController.rightBumper().whileTrue(rc_launcherC);
    // }

    // Shooter far to near
    // 12 ft
    m_operatorController.a().whileTrue(new LauncherSetSpeedForDistance(rc_launcherSS, rc_KickerSS, 7.6));
    // 10 ft
    m_operatorController.b().whileTrue(new LauncherSetSpeedForDistance(rc_launcherSS, rc_KickerSS, 7.4));
    // 8 ft
    m_operatorController.x().whileTrue(new LauncherSetSpeedForDistance(rc_launcherSS, rc_KickerSS, 7));
    // 6 ft
    m_operatorController.y().whileTrue(new LauncherSetSpeedForDistance(rc_launcherSS, rc_KickerSS, 6.5));

    //else if (rc_changeModeSS.manual == true) {
      m_driverController.povUp().onTrue(new LauncherSpeedC(rc_launcherSS, 1));
      m_driverController.povDown().onTrue(new LauncherSpeedC(rc_launcherSS, -1));
      m_driverController.povRight().onTrue(new LauncherSpeedC(rc_launcherSS, 0.1));
      m_driverController.povLeft().onTrue(new LauncherSpeedC(rc_launcherSS, -0.1));
      //m_driverController.rightTrigger().onTrue(rc_launcherC);
      m_operatorController.rightBumper().whileTrue(rc_manualLauncherC);
    //}

    // Launcher Testing button binds
    


    m_driverController.x().whileTrue(rc_KickerC);

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    System.out.println("getAutonomousCommand");
    return new PathPlannerAuto("Test Auto");
  }
}
