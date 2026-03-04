package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ChangeModeSS extends SubsystemBase{

    public boolean manual = false;

    public void setTrue() {
        manual = true;
    }

    public void setFalse() {
        manual = false;
    }
}
