package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Subsystem;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class KickerSS extends SubsystemBase {
  /** Creates a new AlgaeC. */
    public final SparkMax Kicker = new SparkMax(Constants.CANIDConstants.KickerCANID, MotorType.kBrushless);
    public final SparkMax KickerBelt = new SparkMax(Constants.CANIDConstants.KickerBeltCANID, MotorType.kBrushless);

  

    public void KickerForward(){
        Kicker.set(0.5);
        KickerBelt.set(-0.5);
    }

   public void KickerReverse(){
        Kicker.set(-0.5);
        KickerBelt.set(0.5);

    }

    public void KickerStop() {
        Kicker.set(0);
        KickerBelt.set(0);
    }
}
