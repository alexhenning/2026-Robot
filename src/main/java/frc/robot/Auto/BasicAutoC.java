package frc.robot.Auto;

import java.nio.ReadOnlyBufferException;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSS;
import frc.robot.subsystems.KickerSS;
import frc.robot.subsystems.LauncherSS;

public class BasicAutoC extends Command {

    public BasicAutoC(KickerSS subsystem, LauncherSS subsystem2, DriveSubsystem subsystem3) {
        subsystem = RobotContainer.rc_KickerSS;
        subsystem2 = RobotContainer.rc_launcherSS;
        subsystem3 = RobotContainer.m_robotDrive;
        addRequirements(subsystem);   
    }

    public double starttime;
    public double time;
    public double currenttime;
    double launchspeed;

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        launchspeed = RobotContainer.rc_launcherSS.calculateLaunchSpeed(RobotContainer.rc_visionSS.getDistanceToHub());
        RobotContainer.rc_KickerSS.KickerForward();
        RobotContainer.rc_launcherSS.spin(launchspeed);
    }

    @Override
    public void end(boolean interrupted) {
        RobotContainer.rc_KickerSS.KickerStop();
        RobotContainer.rc_launcherSS.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
