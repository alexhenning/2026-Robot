package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimbSS extends SubsystemBase {

    // Create climb motors
    public final SparkFlex climbmotor = new SparkFlex(Constants.CANIDConstants.kClimbMotorCANID, MotorType.kBrushless);
    
    // Climb functions
    public void spin() {
        climbmotor.set(0.5);
    }

    public void reverse() {
        climbmotor.set(-0.5);   
   }

   public void stop() {
    climbmotor.set(0);
   }

}
