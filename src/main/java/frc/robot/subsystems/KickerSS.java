package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Subsystem;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class KickerSS extends SubsystemBase {

    // Create kicker motors
    public final SparkFlex Kicker = new SparkFlex(Constants.CANIDConstants.KickerCANID, MotorType.kBrushless);
    public final SparkFlex KickerBelt = new SparkFlex(Constants.CANIDConstants.KickerBeltCANID, MotorType.kBrushless);

    // Kicker functions
    public void KickerForward(){
        Kicker.set(0.75);
        KickerBelt.set(-0.75);
    }

   public void KickerReverse(){
        Kicker.set(-0.75);
        KickerBelt.set(0.75);

    }

    public void KickerStop() {
        Kicker.set(0);
        KickerBelt.set(0);
    }
}
