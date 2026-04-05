package frc.robot.Auto;

import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveRequest.RobotCentric;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.KickerSS;
import frc.robot.subsystems.LauncherSS;

public class AutoShootC extends Command {

    public AutoShootC(KickerSS subsystem, LauncherSS subsystem2) {
        subsystem = RobotContainer.rc_KickerSS;
        subsystem2 = RobotContainer.rc_launcherSS;
        addRequirements(subsystem);   
    }

    double time;
    double launchspeed;

    @Override
    public void initialize() {
        launchspeed = RobotContainer.rc_launcherSS.calculateLaunchSpeed(RobotContainer.rc_visionSS.getDistanceToHub());
        time = 0;
    }

    @Override
    public void execute() {
        if (time <= 1000) {
            RobotContainer.rc_launcherSS.spin(launchspeed);
        }
        else if (time <= 7000) {
            RobotContainer.rc_KickerSS.KickerForward();
            RobotContainer.rc_launcherSS.spin(launchspeed);
        }
        time = time + 20;

    }

    @Override
    public void end(boolean interrupted) {
        RobotContainer.rc_launcherSS.stop();
        RobotContainer.rc_KickerSS.KickerStop();
    }

    @Override
    public boolean isFinished() {
        if (time > 7000) {
            return true;
        }
        return false;
    }
}
