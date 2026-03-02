// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DeployIntakeCommand extends Command {
  IntakeSubsystem intakeSubsystem;
  PIDController intakePID;
  double angleMeasurement;
  /** Creates a new IntakeCommand. */
  public DeployIntakeCommand(IntakeSubsystem intakeSubsystem, double angleMeasurement) {
    this.intakeSubsystem = intakeSubsystem;
    intakePID = Constants.Intake.intakeDeployPID;
    this.angleMeasurement = angleMeasurement;

    addRequirements(intakeSubsystem);
  }


  /* --------NOTE: WE MIGHT NEED A FEED FORWARD ON INTAKE---------- */


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intakeSubsystem.intakeDeploy.set(intakePID.calculate(intakeSubsystem.deployEncoder.get(), angleMeasurement)); //NOT REAL MEASUREMENT YET
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.intakeDeploy.set(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
