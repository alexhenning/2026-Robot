package frc.robot.Auto;


import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.IntakeSS;
import frc.robot.subsystems.IntakeTiltSS;
import frc.robot.subsystems.KickerSS;
import frc.robot.subsystems.LauncherSS;

public class AutoIntakeC extends Command{
    
    public AutoIntakeC(IntakeTiltSS subsystem, IntakeSS subsystem2) {
        subsystem = RobotContainer.rc_IntakeTiltSS;
        subsystem2 = RobotContainer.rc_intakeSS;
        addRequirements(subsystem);   
    }
    
    double time;

    @Override
    public void initialize() {
        time = 0;
    }

    @Override
    public void execute() {
        if (time <= 1000) {
            RobotContainer.rc_IntakeTiltSS.spin();
        }

        else if (time <= 2000) {
            RobotContainer.rc_intakeSS.IntakeForward();
        }

        else if (time <= 3500) {
            RobotContainer.rc_IntakeTiltSS.reverse();
            RobotContainer.rc_intakeSS.IntakeStop();
        }

        time = time + 20;
    }

    @Override
    public void end(boolean interrupted) {
        RobotContainer.rc_intakeSS.IntakeStop();
        RobotContainer.rc_IntakeTiltSS.stop();
    }

    @Override
    public boolean isFinished() {
        if (time <= 3500) {
            return false;
        }
        else {
            return true;
        }
    }
}
