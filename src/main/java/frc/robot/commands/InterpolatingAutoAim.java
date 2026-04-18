// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.Constants;
import frc.robot.Robot;
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
    interAutoAimOn = false;

    map = new InterpolatingDoubleTreeMap();
    /*------- POWER = 0.9 ------ */
    map.put(2.108, 0.0);
    map.put(2.261, 0.0);
    map.put(2.413, 0.0);
    map.put(2.565, 0.0);
    map.put(2.718, 65.0); //51
    map.put(2.870, 80.0); //66
    map.put(3.023, 89.0); //75
    /*------ Power = 1 --------*/

    //ADDED 14 TO EVERY VALUE (The "Big Fix" Solution)
    /*
     * because zacky wacky is forcing me to explain the "Big Fix":
     * we are adding arbitrary values to the angle outputs of the map
     * because the PID isnt tuned enough to get the hood to the correct angle
     * so we're hoping that by adding these values, we can get it there
     */ 
    
    map.put(3.175, 0.0);
    map.put(3.327, 107.0); //93
    map.put(3.480, 120.0+24); //120
    map.put(3.632, 151.0+14);
    map.put(3.785, 125.0+15);
    map.put(3.937, 175.0+30);
    map.put(4.089, 165.0+18);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    interAutoAimOn = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double currentAngle = shootingSubsystem.hoodAngleEncoderValue;
    double distanceX = swerveSubsystem.getPose().getX() - Robot.correct_HUB_X;
    double distanceY =swerveSubsystem.getPose().getY() - Constants.HUB_Y;
    double distanceFromHub = Math.sqrt((Math.pow(distanceX, 2) + Math.pow(distanceY, 2))); //Pythagorean Theorem

    outputtedAngle = map.get(distanceFromHub);
   /* if(outputtedAngle > shootingSubsystem.hoodAngleEncoderValue){
      shootingSubsystem.hoodAngleMotor.set(-Math.abs(hoodPID.calculate(shootingSubsystem.hoodAngleEncoderValue, outputtedAngle)));
    } else if(outputtedAngle < shootingSubsystem.hoodAngleEncoderValue) {
      shootingSubsystem.hoodAngleMotor.set(Math.abs(hoodPID.calculate(shootingSubsystem.hoodAngleEncoderValue, outputtedAngle)));
    } */
    if((currentAngle >= outputtedAngle + 0.05 || currentAngle <= outputtedAngle - 0.05) && distanceFromHub <= 4.3 && currentAngle <= 250 && currentAngle >= -30 && !operator.povUp().getAsBoolean() && !operator.povDown().getAsBoolean()){
      shootingSubsystem.hoodAngleMotor.set(-hoodPID.calculate(currentAngle, outputtedAngle));
    } else shootingSubsystem.hoodAngleMotor.set(0);
    System.out.println("distance: " + distanceFromHub + "      output: " + outputtedAngle + "     current: " + currentAngle);
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
