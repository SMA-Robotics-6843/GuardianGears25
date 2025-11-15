// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;

import static frc.robot.constants.Constants.secondsToRunIntakeFeeding;
import static frc.robot.constants.Constants.secondsToRunIntakeScoring;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Automations extends SequentialCommandGroup {
  private ElevatorSubsystem elevator;
  private EndEffectorSubsystem endEffector;

  public Automations(ElevatorSubsystem elevatorSubsystem, EndEffectorSubsystem endEffectorSubsystem) {
    elevator = elevatorSubsystem;
    endEffector = endEffectorSubsystem;
  }

  public Command CoralFromFeeding() {
    return Commands.sequence(
      // Run the elevator to feeding, then the end effector to feeding
      Commands.sequence(
        elevator.elevatorToFeeding(),
        endEffector.endEffectorToFeeding().withTimeout(.2)),
        
      // then run the end effector's motor in
      endEffector.fMotorIn().withTimeout(secondsToRunIntakeFeeding));
  }

  public Command ScoreCoralL1() {
    return Commands.sequence(
      // Run the elevator to L1, then the end effector to L1
      Commands.sequence(
        elevator.elevatorToL1(),
        endEffector.endEffectorToL1().withTimeout(.2)),
        
      // then run the end effector's motor out
      endEffector.fMotorOut().withTimeout(secondsToRunIntakeScoring));
  }
  
  public Command ScoreCoralL2() {
      return Commands.sequence(
        // Run the elevator to L2, then the end effector to L2
        Commands.sequence(
          elevator.elevatorToL2(),
          endEffector.endEffectorToL2().withTimeout(.2)),
          
        // then run the end effector's motor out
        endEffector.fMotorOut().withTimeout(secondsToRunIntakeScoring));
  }

  public Command ScoreCoralL3() {
    return Commands.sequence(
      // Run the elevator to L3, then the end effector to L3
      Commands.sequence(
        elevator.elevatorToL3(),
        endEffector.endEffectorToL3().withTimeout(.2)),
        
      // then run the end effector's motor out
      endEffector.fMotorOut().withTimeout(secondsToRunIntakeScoring));
  }

  public Command ScoreCoralL4() {
    return Commands.sequence(
      // Run the elevator to L4, then the end effector to L4
      Commands.sequence(
        elevator.elevatorToL4(),
        endEffector.endEffectorToL4().withTimeout(.2)),
        
      // then run the end effector's motor out
      endEffector.fMotorOut().withTimeout(secondsToRunIntakeScoring));
  }
}
