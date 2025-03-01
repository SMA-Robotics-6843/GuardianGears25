// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import frc.robot.Constants.ElevatorConstants;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class ElevatorSubsystem extends SubsystemBase {
  private SparkMax elevatorMotorLeft = new SparkMax(ElevatorConstants.elevatorMotorLeftID, MotorType.kBrushless);
  private SparkMax elevatorMotorRight = new SparkMax(ElevatorConstants.elevatorMotorRightID, MotorType.kBrushless);
  private final PIDController pid_elevatorMotorLeft = new PIDController(ElevatorConstants.elevatorMotorLeftkP, 0, 0);
  private final PIDController pid_elevatorMotorRight = new PIDController(ElevatorConstants.elevatorMotorRightkP, 0, 0);
  
  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem() {}

  public void MoveElevator(double speed) {
    elevatorMotorLeft.set(-speed);
    elevatorMotorRight.set(speed);
    SmartDashboard.putNumber("elevatorMotorLeft", elevatorMotorLeft.getEncoder().getPosition());
    SmartDashboard.putNumber("elevatorMotorRight", elevatorMotorRight.getEncoder().getPosition());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
