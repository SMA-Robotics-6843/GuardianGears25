// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static edu.wpi.first.wpilibj2.command.Commands.parallel;
import static frc.robot.constants.Constants.ElevatorConstants.*;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class ElevatorSubsystem extends SubsystemBase {
  // Left motor
  private SparkMax elevatorMotorLeft = new SparkMax(elevatorMotorLeftID, MotorType.kBrushless);
  private final PIDController elevatorMotorLeftPID = new PIDController(elevatorMotorLeftkP, elevatorMotorLeftkI,
      elevatorMotorLeftkD);

  // Right motor
  private SparkMax elevatorMotorRight = new SparkMax(elevatorMotorRightID, MotorType.kBrushless);
  private final PIDController elevatorMotorRightPID = new PIDController(elevatorMotorRightkP, elevatorMotorRightkI,
      elevatorMotorRightkD);

  public boolean isElevatorAtSetpoint;

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

  public Command moveElevator(double speed) {
    return parallel(

        run(

            () -> {

              elevatorMotorLeft.set(speed);
              elevatorMotorRight.set(-speed);

            }

        ));
  }

  public Command moveElevatorToSetpoint(double setpointRotationsPerSecond) {
    return parallel(

        run(() -> {

          elevatorMotorLeft.set(
              elevatorMotorLeftPID.calculate(elevatorMotorLeft.getEncoder().getPosition(), setpointRotationsPerSecond));
          elevatorMotorRight.set(elevatorMotorRightPID.calculate(elevatorMotorRight.getEncoder().getPosition(),
              -setpointRotationsPerSecond));

        }));
        
  }

  public boolean getIsElevatorAtSetPoint() {
    if (elevatorMotorLeftPID.atSetpoint() && elevatorMotorRightPID.atSetpoint()) {
      isElevatorAtSetpoint = true;
    } else {
      isElevatorAtSetpoint = false;
    }
    return isElevatorAtSetpoint;
  }

  public Command ElevatorUp() {
    return moveElevator(elevatorMotorsUpSpeed);
  }

  public Command ElevatorDown() {
    return moveElevator(elevatorMotorsDownSpeed);
  }

  public Command ElevatorToL1() {
    SmartDashboard.putBoolean("isElevatorAtSetpoint", getIsElevatorAtSetPoint());
    return moveElevatorToSetpoint(elevatorMotorsL1SetpointRotationsPerSecond);
  }

  public Command ElevatorToL2() {
    SmartDashboard.putBoolean("isElevatorAtSetpoint", getIsElevatorAtSetPoint());
    return moveElevatorToSetpoint(elevatorMotorsL2SetpointRotationsPerSecond);
  }

  public Command ElevatorToL3() {
    SmartDashboard.putBoolean("isElevatorAtSetpoint", getIsElevatorAtSetPoint());
    return moveElevatorToSetpoint(elevatorMotorsL3SetpointRotationsPerSecond);
  }

  public Command ElevatorToFeeding() {
    SmartDashboard.putBoolean("isElevatorAtSetpoint", getIsElevatorAtSetPoint());
    return moveElevatorToSetpoint(elevatorMotorsFeedingSetpointRotationsPerSecond);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("elevatorMotorLeft encoder", elevatorMotorLeft.getEncoder().getPosition());
    SmartDashboard.putNumber("elevatorMotorRight encoder", elevatorMotorRight.getEncoder().getPosition());
  }
}