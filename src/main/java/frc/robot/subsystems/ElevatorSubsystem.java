// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;

import static edu.wpi.first.wpilibj2.command.Commands.parallel;
import static frc.robot.constants.Constants.ElevatorConstants.*;
import static frc.robot.constants.Constants.AutomationConstants.*;
import static frc.robot.constants.Constants.LEDConstants.*;

import java.util.function.BooleanSupplier;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class ElevatorSubsystem extends SubsystemBase {
  // Left motor
  private SparkMax elevatorMotorLeft = new SparkMax(elevatorMotorLeftID, MotorType.kBrushless);
  private final PIDController elevatorMotorLeftPID 
    = new PIDController(elevatorMotorLeftkP, elevatorMotorLeftkI, elevatorMotorLeftkD);

  // Right motor
  private SparkMax elevatorMotorRight = new SparkMax(elevatorMotorRightID, MotorType.kBrushless);
  private final PIDController elevatorMotorRightPID 
    = new PIDController(elevatorMotorRightkP, elevatorMotorRightkI, elevatorMotorRightkD);

  private LEDSubsystem ledSubsystem;
  private Debouncer elevatorAtSetpointDebouncer = new Debouncer(.5, DebounceType.kRising);

  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem(LEDSubsystem m_ledSubsystem) {
    ledSubsystem = m_ledSubsystem;
    setDefaultCommand(
        runOnce(
            () -> {
              elevatorMotorLeft.disable();
              elevatorMotorRight.disable();
            })
            .andThen(Commands.none()));
  }

  public void resetElevatorEncoders() {
    elevatorMotorLeft.getEncoder().setPosition(0);
    elevatorMotorRight.getEncoder().setPosition(0);
  }

  public boolean isElevatorAtSetPoint() {
    return elevatorAtSetpointDebouncer.calculate(elevatorMotorLeftPID.atSetpoint() && elevatorMotorRightPID.atSetpoint());
  }

  BooleanSupplier isElevatorAtSetPointSupplier = this::isElevatorAtSetPoint;

  public Command elevatorUp() {
    return parallel(
        run(
            () -> {
              elevatorMotorLeft.set(elevatorMotorsUpSpeed);
              elevatorMotorRight.set(-elevatorMotorsUpSpeed);
            }
        ));
  }

  public Command elevatorDown() {
    return parallel(
        run(
            () -> {
              elevatorMotorLeft.set(elevatorMotorsDownSpeed);
              elevatorMotorRight.set(-elevatorMotorsDownSpeed);
            }
        ));
  }

  public Command elevatorToL1() {
    return parallel(
        run(() -> {
          elevatorMotorLeft.set(
              elevatorMotorLeftPID.calculate(elevatorMotorLeft.getEncoder().getPosition(), elevatorMotorsL1Setpoint));
          elevatorMotorRight.set(elevatorMotorRightPID.calculate(elevatorMotorRight.getEncoder().getPosition(),
              -elevatorMotorsL1Setpoint));
        }),
          ledSubsystem.setLED(yellow)
        );
  }

  public Command elevatorToL2() {
    return parallel(
        run(() -> {
          elevatorMotorLeft.set(
              elevatorMotorLeftPID.calculate(elevatorMotorLeft.getEncoder().getPosition(), elevatorMotorsL2Setpoint));
          elevatorMotorRight.set(elevatorMotorRightPID.calculate(elevatorMotorRight.getEncoder().getPosition(),
              -elevatorMotorsL2Setpoint));
        }),
          ledSubsystem.setLED(purple)
        );
  }

  public Command elevatorToL3() {
      return parallel(
        run(() -> {
          elevatorMotorLeft.set(
            elevatorMotorLeftPID.calculate(elevatorMotorLeft.getEncoder().getPosition(), elevatorMotorsL3Setpoint));
          elevatorMotorRight.set(
            elevatorMotorRightPID.calculate(elevatorMotorRight.getEncoder().getPosition(), -elevatorMotorsL3Setpoint));
        }),
        ledSubsystem.setLED(orange)
      );
  }

  public Command elevatorToL4() {
    return parallel(
        run(() -> {
          elevatorMotorLeft.set(
            elevatorMotorLeftPID.calculate(elevatorMotorLeft.getEncoder().getPosition(), elevatorMotorsL4Setpoint));
          elevatorMotorRight.set(
            elevatorMotorRightPID.calculate(elevatorMotorRight.getEncoder().getPosition(), -elevatorMotorsL4Setpoint));
        }),
        ledSubsystem.setLED(scrollingRainbow)
      ).until(isElevatorAtSetPointSupplier);
  }

  public Command elevatorToFeeding() {
    return parallel(
        run(() -> {
          elevatorMotorLeft.set(
            elevatorMotorLeftPID.calculate(elevatorMotorLeft.getEncoder().getPosition(), elevatorMotorsFeedingSetpoint));
          elevatorMotorRight.set(
            elevatorMotorRightPID.calculate(elevatorMotorRight.getEncoder().getPosition(), -elevatorMotorsFeedingSetpoint));
        }),
        ledSubsystem.setLED(green)
        );
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("isElevatorAtSetpoint", isElevatorAtSetPoint());
    SmartDashboard.putNumber("elevatorMotorLeft encoder", elevatorMotorLeft.getEncoder().getPosition());
    SmartDashboard.putNumber("elevatorMotorRight encoder", elevatorMotorRight.getEncoder().getPosition());
  }
}