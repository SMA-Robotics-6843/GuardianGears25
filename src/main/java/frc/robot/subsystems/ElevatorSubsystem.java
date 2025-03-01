// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import frc.robot.Constants;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class ElevatorSubsystem extends SubsystemBase {
  private SparkMax elevatorMotorLeft = new SparkMax(Constants.elevatorMotorLeft, MotorType.kBrushless);
  private SparkMax elevatorMotorRight = new SparkMax(Constants.elevatorMotorRight, MotorType.kBrushless);
  
  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem() {}

  public void MoveElevator(double speed) {
    elevatorMotorLeft.set(-speed);
    elevatorMotorRight.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
