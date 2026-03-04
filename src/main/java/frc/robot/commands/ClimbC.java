package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ClimbSS;

public class ClimbC extends Command{

    public ClimbC(ClimbSS subsystem) {
        subsystem = RobotContainer.rc_climbSS;
        addRequirements(subsystem);   
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        if (RobotContainer.m_driverController.a().getAsBoolean()) {
            RobotContainer.rc_climbSS.spin();
        }
        else {
            RobotContainer.rc_climbSS.reverse();
        }
    }

    @Override
    public void end(boolean interrupted) {
        RobotContainer.rc_climbSS.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
