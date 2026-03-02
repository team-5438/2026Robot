// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DutyCycle;
import frc.robot.utils.Controller;
import frc.robot.utils.GetAlliance;
import frc.robot.utils.StickDeadband;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static boolean isRedAlliance = GetAlliance.isRed();

  public static class VisionConstants
  {
    public static final boolean DRIVEWITHVISION = true;
  }
  public static final PIDController anglePID = new PIDController(10, 0, 0.1);
  public static final double HUB_X = 4.62534; //x of hub in meters  
  public static final double HUB_Y = 4.034536; //y of hub in meters
  public static final Controller Operator = new Controller(
    /* NOTE: this is a PS5 Controller */
    1, /* id */
    new StickDeadband(0.1, 0.1), /* left stick deadband */
    new StickDeadband(0.15, 0.15)); /* right stick deadband+ */

  public static final Controller Driver = new Controller(
    /* NOTE: this is a Xbox Controller */
    0, /* id */
    new StickDeadband(0.15          , 0.15), /* left stick deadband */
    new StickDeadband(0.15, 0.15)); /* right stick deadband */

  //public static final Rotation3d gyroOffset = new Rotation3d(0,0,90);
  public static final double MAX_SPEED  = 4.5; //in meters/sec

  public static final class Robot {
    /* this boolean is used to determine if we should flip the path in path planner */
    public static final boolean isRedAlliance = GetAlliance.isRed();
  }

  public static final class Intake {
    public static final int intakeDeployID = 1; //NOT REAL ID
    public static final int intakeSpinnyID = 2; //NOT REAL ID
    public static final int deployEncoderID = 0; //NOT REAL ID
    public static final PIDController intakeDeployPID = new PIDController(1, 0, 1); //UNTESTED

  }

  public static final class Shooting {
    public static final int shootWheelsID = 4; //NOT REAL ID
    public static final int feedWheelsID = 3; //NOT REAL ID
    public static final int hoodAngleMotorID = 6; //NOT REAL ID
    public static final int hoodAngleEncoderID = 1; //NOT REAL ID
  }

  public static final class QuestNav {
    //ADD ACTUAL VALUES FOR INITIAL POSE
    public static final Pose3d robotPose = isRedAlliance ? new Pose3d(0,0,0, null) : new Pose3d(0,0,0, null);
    public static final Matrix<N3, N1> QUESTNAV_STD_DEVS =
      VecBuilder.fill(
        0.02, // Trust down to 2cm in X direction
        0.02, // Trust down to 2cm in Y direction
        0.035 // Trust down to 2 degrees rotational
      );
    public static final Transform3d ROBOT_TO_QUEST = new Transform3d( /*TODO: Put your x, y, z, yaw, pitch, and roll offsets here!*/ );

  }
}
