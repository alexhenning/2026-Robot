package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Subsystem;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class KickerSS extends SubsystemBase {
  /** Creates a new AlgaeC. */
  public final SparkMax Fuel = new SparkMax(Constants.KickerCANID, MotorType.kBrushless);

    public void KickerForward(){
        Fuel.set(0.5);
    }

    public void KickerStop() {
        Fuel.set(0);
    }
}
