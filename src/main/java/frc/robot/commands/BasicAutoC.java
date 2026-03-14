package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.IntakeSS;
import frc.robot.subsystems.KickerSS;

public class BasicAutoC extends Command {

    public BasicAutoC(KickerSS subsystem) {
        subsystem = RobotContainer.rc_KickerSS;
        addRequirements(subsystem);   
    }

    double start;

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        start = Timer.getFPGATimestamp();
        double launchspeed = RobotContainer.rc_launcherSS.calculateLaunchSpeed(RobotContainer.rc_visionSS.getDistanceToHub());
        
        //if (start - Timer.getFPGATimestamp() <= 1) {
           // RobotContainer.rc_launcherSS.spin(launchspeed);
       // }
        //else if (start - Timer.getFPGATimestamp() <= 9) {
        RobotContainer.rc_KickerSS.KickerForward();      
        RobotContainer.rc_launcherSS.spin(launchspeed);
       // }
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
