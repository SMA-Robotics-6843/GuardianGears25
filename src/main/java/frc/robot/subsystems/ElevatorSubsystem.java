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

  public Command moveElevatorToSetpoint(double setpoint) {
    return parallel(

        run(() -> {

          elevatorMotorLeft.set(
              elevatorMotorLeftPID.calculate(elevatorMotorLeft.getEncoder().getPosition(), setpoint));
          elevatorMotorRight.set(elevatorMotorRightPID.calculate(elevatorMotorRight.getEncoder().getPosition(),
              -setpoint));

        }));

  }

  public void resetElevatorEncoders() {
    elevatorMotorLeft.getEncoder().setPosition(0);
    elevatorMotorRight.getEncoder().setPosition(0);
  }

  public boolean getIsElevatorAtSetPoint() {
    if (elevatorMotorLeftPID.atSetpoint() && elevatorMotorRightPID.atSetpoint()) {
      return true;
    } else {
      return false;
    }
  }

  public Command elevatorUp() {
    return moveElevator(elevatorMotorsUpSpeed);
  }

  public Command elevatorDown() {
    return moveElevator(elevatorMotorsDownSpeed);
  }

  public Command elevatorToL1() {
    return moveElevatorToSetpoint(elevatorMotorsL1Setpoint);
  }

  public Command elevatorToL2() {
    return moveElevatorToSetpoint(elevatorMotorsL2Setpoint);
  }

  public Command elevatorToL3() {
    return moveElevatorToSetpoint(elevatorMotorsL3Setpoint);
  }

  public Command elevatorToL4() {
    return moveElevatorToSetpoint(elevatorMotorsL4Setpoint);
  }

  public Command elevatorToFeeding() {
    return moveElevatorToSetpoint(elevatorMotorsFeedingSetpoint);
  }

  public Command elevatorToLowAlgae() {
    return moveElevatorToSetpoint(elevatorMotorsLowAlgaeSetpoint);
  }

  public Command elevatorToHighAlgae() {
    return moveElevatorToSetpoint(elevatorMotorsHighAlgaeSetpoint);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("isElevatorAtSetpoint", getIsElevatorAtSetPoint());
    SmartDashboard.putNumber("elevatorMotorLeft encoder", elevatorMotorLeft.getEncoder().getPosition());
    SmartDashboard.putNumber("elevatorMotorRight encoder", elevatorMotorRight.getEncoder().getPosition());
  }
}