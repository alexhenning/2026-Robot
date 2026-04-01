package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.Subsystem;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSS extends SubsystemBase {

    // Create intake motor
    public final SparkFlex Intake1 = new SparkFlex(Constants.CANIDConstants.IntakeSpinCANID, MotorType.kBrushless);

    // Intake functions
    public void IntakeForward(){
        Intake1.set(-0.5);
    }

    public void IntakeReverse() {
        Intake1.set(0.5);

    }
    
    public void IntakeStop() {
        Intake1.set(0);
    }
}
