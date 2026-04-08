// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Meters;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.spi.CurrencyNameProvider;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.subsystems.ShootingSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import swervelib.SwerveInputStream;
import swervelib.math.SwerveMath;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SpencerAutoAlign extends Command {
  SwerveSubsystem swerveSubsystem;
  PIDController anglePID;
  Pose2d robotPose2d;
  double robotX, robotY, 
         diffX, diffY,
         desiredAngle, currentAngle, diffAngle,
         PIDOutput;
  public boolean autoAlignOn;
  CommandXboxController driver;
  DoubleSupplier rightRotation;
  // ShootingSubsystem shootingSubsystem;
  GenericEntry autoAlignEntry;
  /** Creates a new SpencerAutoAim. */
  public SpencerAutoAlign(CommandXboxController driver, SwerveSubsystem swerveSubsystem, GenericEntry autoAlignEntry) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.swerveSubsystem = swerveSubsystem;
    // this.shootingSubsystem = shootingSubsystem;
    this.autoAlignEntry = autoAlignEntry;

    anglePID = Constants.anglePID;
    anglePID.enableContinuousInput(-Math.PI, Math.PI);
    this.driver = driver;
    addRequirements(swerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    autoAlignOn = true;
    rightRotation = () -> MathUtil.applyDeadband(driver.getRightX(), Constants.Driver.rightStick.X);
    // shootingSubsystem.shootWheelsLeft.set(-0.1);
    // shootingSubsystem.shootWheelsRight.set(0.1);
    autoAlignEntry.setBoolean(autoAlignOn);

  }

  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speedMod = driver.getRawAxis(XboxController.Axis.kRightTrigger.value) == 1 ? 0.8 : 1;

    robotPose2d = swerveSubsystem.getPose();
    robotX = robotPose2d.getMeasureX().in(Meters);
    robotY = robotPose2d.getMeasureY().in(Meters);
    diffX = Constants.HUB_X - robotX;
    diffY = Constants.HUB_Y - robotY;
    desiredAngle = (diffX == 0) ? 0 : Math.toDegrees(Math.atan2(diffY, diffX));
    currentAngle = robotPose2d.getRotation().getDegrees();
    diffAngle = currentAngle - desiredAngle;

    swerveSubsystem.driveFieldOriented(getTargetSpeeds(driver.getLeftY() * speedMod,
                          driver.getLeftX()  * speedMod,
                          Rotation2d.fromDegrees(desiredAngle)));
    System.out.println("P: " + swerveSubsystem.swerveDrive.swerveController.thetaController.getP());
    System.out.println("I: " + swerveSubsystem.swerveDrive.swerveController.thetaController.getI());
    System.out.println("D: " + swerveSubsystem.swerveDrive.swerveController.thetaController.getD());

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    autoAlignOn = false;
    // shootingSubsystem.shootWheelsLeft.set(0);
    // shootingSubsystem.shootWheelsRight.set(0);
    autoAlignEntry.setBoolean(autoAlignOn);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(rightRotation.getAsDouble() != 0){
      System.out.println("JASLKDFJLSAJFLKDSAMFLKDSAJFLKDSAJFLKSAJFLKSAJF");
      return true;
    }
    return false;
  }


    /**
   * Get the chassis speeds based on controller input of 1 joystick and one angle. Control the robot at an offset of
   * 90deg.
   *
   * @param xInput X joystick input for the robot to move in the X direction.
   * @param yInput Y joystick input for the robot to move in the Y direction.
   * @param angle  The angle in as a {@link Rotation2d}.
   * @return {@link ChassisSpeeds} which can be sent to the Swerve Drive.
   */
  public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, Rotation2d angle)
  {
    Translation2d scaledInputs = SwerveMath.cubeTranslation(new Translation2d(xInput, yInput));

    return getTargetSpeeds(scaledInputs.getX(),
                                                        scaledInputs.getY(),
                                                        angle.getRadians(),
                                                        swerveSubsystem.getHeading().getRadians(),
                                                        Constants.MAX_SPEED);
  }

    /**
   * Get the chassis speeds based on controller input of 1 joystick [-1,1] and an angle.
   *
   * @param xInput                     X joystick input for the robot to move in the X direction. X = xInput * maxSpeed
   * @param yInput                     Y joystick input for the robot to move in the Y direction. Y = yInput *
   *                                   maxSpeed;
   * @param angle                      The desired angle of the robot in radians.
   * @param currentHeadingAngleRadians The current robot heading in radians.
   * @param maxSpeed                   Maximum speed in meters per second.
   * @return {@link ChassisSpeeds} which can be sent to the Swerve Drive.
   */
  public ChassisSpeeds getTargetSpeeds(
      double xInput, double yInput, double angle, double currentHeadingAngleRadians, double maxSpeed)
  {
    // Convert joystick inputs to m/s by scaling by max linear speed.  Also uses a cubic function
    // to allow for precise control and fast movement.
    double x = xInput * maxSpeed;
    double y = yInput * maxSpeed;

    return getRawTargetSpeeds(x, y, angle, currentHeadingAngleRadians);
  }

  /**
   * Get the {@link ChassisSpeeds} based of raw speeds desired in meters/second and heading in radians.
   *
   * @param xSpeed                     X speed in meters per second.
   * @param ySpeed                     Y speed in meters per second.
   * @param targetHeadingAngleRadians  Target heading in radians.
   * @param currentHeadingAngleRadians Current heading in radians.
   * @return {@link ChassisSpeeds} the robot should move to.
   */

  public ChassisSpeeds getRawTargetSpeeds(double xSpeed, double ySpeed, double targetHeadingAngleRadians,
                                          double currentHeadingAngleRadians)
  {
    // Calculates an angular rate using a PIDController and the commanded angle. Returns a value between -1 and 1
    // which is then scaled to be between -maxAngularVelocity and +maxAngularVelocity.
    return getRawTargetSpeeds(xSpeed, ySpeed,
                              anglePID.calculate(currentHeadingAngleRadians, targetHeadingAngleRadians) *
                              swerveSubsystem.swerveDrive.swerveController.config.maxAngularVelocity);
  }

  /**
   * Get the {@link ChassisSpeeds} based of raw speeds desired in meters/second and heading in radians.
   *
   * @param xSpeed X speed in meters per second.
   * @param ySpeed Y speed in meters per second.
   * @param omega  Angular velocity in radians/second.
   * @return {@link ChassisSpeeds} the robot should move to.
   */
  public ChassisSpeeds getRawTargetSpeeds(double xSpeed, double ySpeed, double omega)
  {
    if (swerveSubsystem.swerveDrive.swerveController.xLimiter != null)
    {
      xSpeed = swerveSubsystem.swerveDrive.swerveController.xLimiter.calculate(xSpeed);
    }
    if (swerveSubsystem.swerveDrive.swerveController.yLimiter != null)
    {
      ySpeed = swerveSubsystem.swerveDrive.swerveController.yLimiter.calculate(ySpeed);
    }
    // if (angleLimiter != null)
    // {
    //   omega = angleLimiter.calculate(omega);
    // }

    return new ChassisSpeeds(xSpeed, ySpeed, omega);
  }
}
