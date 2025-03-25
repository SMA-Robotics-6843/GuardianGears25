// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import static edu.wpi.first.wpilibj2.command.Commands.parallel;
import static frc.robot.constants.Constants.ClimberConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {
  private static SparkMax climberMotor = new SparkMax(climberMotorID, MotorType.kBrushless);
  private EndEffectorSubsystem endEffector;

  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem(EndEffectorSubsystem m_endEffector) {

    endEffector = m_endEffector;

    setDefaultCommand(

        runOnce(

            () -> {

              climberMotor.disable();

            })

            .andThen(run(() -> {
            }))

            .withName("Idle"));
  }

  private Command moveClimber(double speed) {
    return parallel(

        run(() -> {

          climberMotor.set(speed);

        }),

        endEffector.holdEndEffector());

  }

  public Command windClimber() {
    return moveClimber(climberMotorSpeed);
  }

  public Command unwindClimber() {
    return moveClimber(-climberMotorSpeed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
