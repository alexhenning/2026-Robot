// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.IntakeTiltSS;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeTiltC extends Command {
  /** Creates a new ClimbPIDC. */
  public final IntakeTiltSS m_elev;
  //private final DoubleSupplier m_desiredVelocity;

  public IntakeTiltC(IntakeTiltSS subsystem/*, DoubleSupplier desiredVelocity*/) {
    m_elev = subsystem;
   // m_desiredVelocity = desiredVelocity;
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //m_elev.setPosition(m_desiredVelocity.getAsDouble());
    if (RobotContainer.m_operatorController.povLeft().getAsBoolean()) {
      RobotContainer.rc_IntakeTiltSS.spin();
    }
    else {
      RobotContainer.rc_IntakeTiltSS.reverse();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //m_elev.setVelocity(0);
    RobotContainer.rc_IntakeTiltSS.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
