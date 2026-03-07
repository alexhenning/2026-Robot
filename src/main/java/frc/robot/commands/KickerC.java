package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.KickerSS;

public class KickerC extends Command {
  /** Creates a new AlgaeC. */
  public KickerC(KickerSS subsystem) {
    subsystem = RobotContainer.rc_KickerSS;
    addRequirements(subsystem);   
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (RobotContainer.m_operatorController.leftTrigger().getAsBoolean()) {
      RobotContainer.rc_KickerSS.KickerReverse();
    }
    else {
      RobotContainer.rc_KickerSS.KickerForward();
    }
   
  }
// Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.rc_KickerSS.KickerStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
  
}
