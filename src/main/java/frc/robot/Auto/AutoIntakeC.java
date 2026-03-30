package frc.robot.Auto;


import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class AutoIntakeC extends Command{

    double starttime;
    double time;

    @Override
    public void initialize() {
        starttime = Timer.getFPGATimestamp();
    }

    @Override
    public void execute() {
        if (time-starttime < 1) {
            RobotContainer.rc_IntakeTiltSS.spin();
        }

        else if (time - starttime < 2) {
            RobotContainer.rc_intakeSS.IntakeForward();
        }

        else if (time - starttime < 3.5) {
            RobotContainer.rc_IntakeTiltSS.reverse();
        }
    }

    @Override
    public void end(boolean interrupted) {
        RobotContainer.rc_intakeSS.IntakeStop();
        RobotContainer.rc_IntakeTiltSS.stop();
    }

    @Override
    public boolean isFinished() {
        if (time <= 3.5) {
            return false;
        }
        else {
            return true;
        }
    }
}
