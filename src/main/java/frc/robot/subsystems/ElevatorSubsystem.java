// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.wpilibj2.command.Commands.parallel;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {
  // Left motor
  private SparkMax elevatorMotorLeft = new SparkMax(ElevatorConstants.elevatorMotorLeftID, MotorType.kBrushless);
  private final PIDController elevatorMotorLeftPID = new PIDController(ElevatorConstants.elevatorMotorLeftkP, 0, 0);
  
  // Right motor
  private SparkMax elevatorMotorRight = new SparkMax(ElevatorConstants.elevatorMotorRightID, MotorType.kBrushless);
  private final PIDController elevatorMotorRightPID = new PIDController(ElevatorConstants.elevatorMotorRightkP, 0, 0);
  
  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem() {
    

    setDefaultCommand(

        runOnce(

                () -> {
                    
                  elevatorMotorLeft.disable();

                  elevatorMotorRight.disable();

                })

            .andThen(run(() -> {
            }))

            .withName("Idle"));
  }

  public void moveElevator(double speed) {
    elevatorMotorLeft.set(-speed);
    elevatorMotorRight.set(speed);
  }

  public Command moveElevatorToSetpoint(double leftSetpointRotationsPerSecond, double rightSetpointRotationsPerSecond) {
    return parallel(

            run( 

                () -> {

                  elevatorMotorLeft.set(elevatorMotorLeftPID.calculate(elevatorMotorLeft.getEncoder().getPosition(), leftSetpointRotationsPerSecond));
                  elevatorMotorRight.set(elevatorMotorRightPID.calculate(elevatorMotorRight.getEncoder().getPosition(), rightSetpointRotationsPerSecond));

                })).withName("Move Elevator");
  }

  public Command elevatorToL2() {
    // left setpoint should be negative, right should be positive
    return moveElevatorToSetpoint(26, 26);
  }

  public Command elevatorToL3() {
    return moveElevatorToSetpoint(0, 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("elevatorMotorLeftEncoder", elevatorMotorLeft.getEncoder().getPosition());
    SmartDashboard.putNumber("elevatorMotorRightEncoder", elevatorMotorRight.getEncoder().getPosition());
  }
}