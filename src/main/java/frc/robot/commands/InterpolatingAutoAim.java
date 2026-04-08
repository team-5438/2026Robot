// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.Constants;
import frc.robot.subsystems.ShootingSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class InterpolatingAutoAim extends Command {
  ShootingSubsystem shootingSubsystem;
  SwerveSubsystem swerveSubsystem;
  PIDController hoodPID;
  double outputtedAngle;
  CommandPS5Controller operator;
  InterpolatingDoubleTreeMap map;
  public boolean interAutoAimOn;
  /** Creates a new InterpolatingAutoAim. */
  public InterpolatingAutoAim(ShootingSubsystem shootingSubsystem, SwerveSubsystem swerveSubsystem, CommandPS5Controller operator) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shootingSubsystem = shootingSubsystem;
    this.swerveSubsystem = swerveSubsystem;
    this.operator = operator;
    hoodPID = Constants.Shooting.hoodPID;
    interAutoAimOn = true;

    map = new InterpolatingDoubleTreeMap();
    /*------- POWER = 0.9 ------ */
    map.put(2.108, 0.0);
    map.put(2.261, 0.0);
    map.put(2.413, 0.0);
    map.put(2.565, 0.0);
    map.put(2.718, 51.0);
    map.put(2.870, 66.0);
    map.put(3.023, 75.0);
    /*------ Power = 1 --------*/
    map.put(3.175, 0.0);
    map.put(3.327, 93.0);
    map.put(3.480, 120.0);
    map.put(3.632, 151.0);
    map.put(3.785, 125.0);
    map.put(3.937, 175.0);
    map.put(4.089, 165.0);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double distanceX = swerveSubsystem.getPose().getX() - Constants.HUB_X;
    double distanceY =swerveSubsystem.getPose().getY() - Constants.HUB_Y;
    double distanceFromHub = Math.sqrt((Math.pow(distanceX, 2) + Math.pow(distanceY, 2))); //Pythagorean Theorem

    outputtedAngle = map.get(distanceFromHub);
    if(outputtedAngle > shootingSubsystem.hoodAngleEncoderValue){
      shootingSubsystem.hoodAngleMotor.set(-Math.abs(hoodPID.calculate(shootingSubsystem.hoodAngleEncoderValue, outputtedAngle)));
    } else {
      shootingSubsystem.hoodAngleMotor.set(Math.abs(hoodPID.calculate(shootingSubsystem.hoodAngleEncoderValue, outputtedAngle)));
    }
    System.out.println("distance: " + distanceFromHub + "      output: " + outputtedAngle + "     current: " + shootingSubsystem.hoodAngleEncoderValue);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    interAutoAimOn = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(operator.povUp().getAsBoolean() || operator.povDown().getAsBoolean()){
      return true;
    }
    return false;
  }
}
