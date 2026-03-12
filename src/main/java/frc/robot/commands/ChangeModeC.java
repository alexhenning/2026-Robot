// package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.InstantCommand;
// import frc.robot.Robot;
// import frc.robot.RobotContainer;
// import frc.robot.subsystems.ChangeModeSS;
// import frc.robot.subsystems.ClimbSS;

// public class ChangeModeC extends InstantCommand{
    
//     public ChangeModeC(ChangeModeSS subsystem) {
//         subsystem = RobotContainer.rc_changeModeSS;
//         addRequirements(subsystem);   
//     }

//     @Override
//     public void initialize() {
//         if (RobotContainer.m_driverController.a().getAsBoolean()) {
//             RobotContainer.rc_changeModeSS.setTrue();
//         }
//     }

// }
