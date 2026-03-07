package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.KickerSS;
import frc.robot.subsystems.LauncherSS;
public class LauncherSetSpeedForDistance extends Command {
    LauncherSS launcher;
    KickerSS kicker;
    double voltage;

    public LauncherSetSpeedForDistance(LauncherSS launch_subsystem, KickerSS kicker_subsytem, double target_voltage) {
        launcher = launch_subsystem;
        kicker = kicker_subsytem;
        voltage = target_voltage;
        addRequirements(launcher, kicker);
    }

    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        launcher.spin(voltage - RobotContainer.m_operatorController.getLeftY());
        kicker.KickerForward();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        launcher.stop();
        kicker.KickerStop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
