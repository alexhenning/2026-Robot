// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeTiltSS extends SubsystemBase {

  public final SparkMax Motor1 = new SparkMax(Constants.CANIDConstants.kIntakeTiltMotorCANID, MotorType.kBrushless);

  private final PIDController Velo_PID = new PIDController(Constants.kP, Constants.kI, Constants.kD);
  private final PIDController Pos_PID = new PIDController(Constants.kP, Constants.kI, Constants.kD);

  private boolean manual = true;

  private double desiredVelocity;
  private double desiredPosition;

  private double voltage;

  public void ElevStop() {
    Motor1.set(0);
  }

  public void ElevZero() {
    Motor1.set(-0.3);
  }

  public void PIDSS() {
    Motor1.setInverted(true);
    Motor1.getEncoder().setPosition(0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Elevator Position", getPosition());
    SmartDashboard.putNumber("Desired Elevator Velocity", desiredVelocity);
    SmartDashboard.putNumber("Elevator Voltage", getVoltage());

    setVoltage();
  }

  public void setVoltage() {
    double PIDVoltage;
    if (manual) { // Calculate position, if it is the position, stop the arm
      PIDVoltage = Velo_PID.calculate(getVelocity(), desiredVelocity);
    } else {
      // Otherwise try to go to the position
      double PIDVelocity = Pos_PID.calculate(getPosition(), desiredPosition);
      PIDVoltage = Velo_PID.calculate(getVelocity(), PIDVelocity);
      // double feedforwardVoltage = feedforward.calculate(getPosition(), 10);
      // PIDVoltage = PIDVoltage + feedforwardVoltage;
    }

    voltage = PIDVoltage;

    Motor1.setVoltage(PIDVoltage);
  }

  public double getVoltage() {
    return voltage;
  }

  public void setVelocity(double degreesPerSecond) {
    manual = true;
    desiredVelocity = degreesPerSecond;
  }

  public void setPosition(double degrees) {
    manual = false;
    desiredPosition = degrees;
  }

  public double getPosition() {
    return Motor1.getEncoder().getPosition() * 8;
  }

  public double getVelocity() {
    return Motor1.getEncoder().getVelocity() * (2 / 15);
  }
}