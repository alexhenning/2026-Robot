package frc.robot.commands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.OIConstants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.*;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LauncherSS;
import edu.wpi.first.math.*;

public class AutoAlignC extends Command{    
    
    public AutoAlignC(DriveSubsystem subsystem) {
        subsystem = RobotContainer.m_robotDrive;
        addRequirements(subsystem);
    }


    @Override
    public void initialize() {}

    @Override
    public void execute() {
    if (RobotContainer.rc_visionSS.isblue && !RobotContainer.rc_visionSS.results.isEmpty()) {
        RobotContainer.m_robotDrive.drive(
            -MathUtil.applyDeadband(
                        RobotContainer.m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                    -MathUtil.applyDeadband(
                        RobotContainer.m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                        RobotContainer.m_robotDrive.getBlueHubAngle(),
                        true
                        );
    }
    else if (!RobotContainer.rc_visionSS.isblue && !RobotContainer.rc_visionSS.results.isEmpty()) {
        RobotContainer.m_robotDrive.drive(
            -MathUtil.applyDeadband(
                        RobotContainer.m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                    -MathUtil.applyDeadband(
                        RobotContainer.m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                        RobotContainer.m_robotDrive.getRedHubAngle(),
                        true
                        );
    }
    else {RobotContainer.m_robotDrive.drive(
                    -MathUtil.applyDeadband(
                        RobotContainer.m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                    -MathUtil.applyDeadband(
                        RobotContainer.m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                    -MathUtil.applyDeadband(
                        RobotContainer.m_driverController.getRightX(), OIConstants.kDriveDeadband),
                    true);



    
  }    }
    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false;
    }
}


