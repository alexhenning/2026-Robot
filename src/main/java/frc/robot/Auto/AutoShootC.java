package frc.robot.Auto;

import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveRequest.RobotCentric;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.IntakeTiltSS;
import frc.robot.subsystems.KickerSS;
import frc.robot.subsystems.LauncherSS;

public class AutoShootC extends Command {

    public AutoShootC(KickerSS m_kicker, LauncherSS m_launcher, IntakeTiltSS m_intaketilt) {
        m_kicker = RobotContainer.rc_KickerSS;
        m_launcher = RobotContainer.rc_launcherSS;
        m_intaketilt = RobotContainer.rc_IntakeTiltSS;
        addRequirements(m_kicker, m_intaketilt, m_launcher);   
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
        if (time < 7000 && time > 1000) {
            if (time < 2000) {
                RobotContainer.rc_IntakeTiltSS.reverse();
            }
            if (time < 3000 && time > 2000) {
                RobotContainer.rc_IntakeTiltSS.spin();
            }
            if (time < 4000 && time > 3000) {
                RobotContainer.rc_IntakeTiltSS.reverse();      
            }
            if (time > 4000 && time < 5000) {
                RobotContainer.rc_IntakeTiltSS.spin();
            }
            if (time < 5000 && time > 6000) {
                RobotContainer.rc_IntakeTiltSS.reverse();      
            }
            if (time > 6000 && time < 7000) {
                RobotContainer.rc_IntakeTiltSS.spin();
            }
            if (time % 1000 == 0) {
                RobotContainer.rc_IntakeTiltSS.stop();
            }
            
        }
        time = time + 20;

    }

    @Override
    public void end(boolean interrupted) {
        RobotContainer.rc_launcherSS.stop();
        RobotContainer.rc_KickerSS.KickerStop();
        RobotContainer.rc_IntakeTiltSS.stop();
    }

    @Override
    public boolean isFinished() {
        if (time > 7000) {
            return true;
        }
        return false;
    }
}
