package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LauncherSS extends SubsystemBase{

    public double speed = 8;
    public int calc;

    public static final SparkMax launcher1 = new SparkMax(Constants.CANIDConstants.launcher1CANID, MotorType.kBrushless);
    public static final SparkMax launcher2 = new SparkMax(Constants.CANIDConstants.launcher2CANID, MotorType.kBrushless);


    // launcher at 0.675 gets it in from 170
    // Launcher at 0.625 gets it in at 130
    
    public Double changeSpeed(double change) {
      speed = speed + change;
      if (speed < 0) {
        speed = 0;
      }
      if (speed > 13) {
        speed = 13;
      }
      return speed;
    }

    public double getSpeed() {
        return speed;
    }

    public void manualspin() {
      launcher1.setVoltage(speed);
      launcher2.setVoltage(-speed); 
    }

    public void stop(){
        launcher1.set(0);
        launcher2.set(0);
    }

    public double calculateLaunchSpeed(double distanceToHub) {
      double launchspeed = (0.0184*distanceToHub + 4.95);
      calc = calc + 1;
      SmartDashboard.putNumber("Calculated # ", calc);
      SmartDashboard.putNumber("Calculated Launch Speed", launchspeed);
      return launchspeed;
    }
    
    public void spin(double launchspeed) {
      SmartDashboard.putNumber("Launcher Voltage ", launchspeed);
      launcher1.setVoltage(launchspeed);
      launcher2.setVoltage(-launchspeed);
    }

    public void staticlaunch() {
      launcher1.setVoltage(-7.4);
      launcher2.setVoltage(7.4);
    }

}
