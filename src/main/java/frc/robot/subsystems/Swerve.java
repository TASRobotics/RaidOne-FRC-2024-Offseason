// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.utils.SwerveUtils;

public class Swerve extends SubsystemBase {
  // Create SwerveModules
  private final SwerveModule m_LeftFront = new SwerveModule(
    Constants.DriveConstants.kLeftFrontThrottleID,
    Constants.DriveConstants.kLeftFrontRotorID,
    Constants.DriveConstants.kLeftFrontThrottleReversed,
    Constants.DriveConstants.kLeftFrontRotorOffsetngle
  );
      
  private final SwerveModule m_LeftBack = new SwerveModule(
    Constants.DriveConstants.kLeftBackThrottleID,
    Constants.DriveConstants.kLeftBackRotorID,
    Constants.DriveConstants.kLeftFrontThrottleReversed,
    Constants.DriveConstants.kLeftBackRotorOffsetngle
  );
      
  private final SwerveModule m_RightFront = new SwerveModule(
    Constants.DriveConstants.kRightFrontThrottleID,
    Constants.DriveConstants.kRightFrontRotorID,
    Constants.DriveConstants.kRightFrontThrottleReversed,
    Constants.DriveConstants.kRightFrontRotorOffsetngle
  );

  private final SwerveModule m_RightBack = new SwerveModule(
    Constants.DriveConstants.kRightBackThrottleID,
    Constants.DriveConstants.kRightBackRotorID,
    Constants.DriveConstants.kLeftFrontThrottleReversed,
    Constants.DriveConstants.kRightBackRotorOffsetngle
  );

  // Pigeon
  private final PigeonIMU m_Imu = new PigeonIMU(Constants.DriveConstants.kImuID);

  // Slew rate filter variables for controlling lateral acceleration
  private double m_currentRotation = 0.0;
  private double m_currentTranslationDir = 0.0;
  private double m_currentTranslationMag = 0.0;

  private SlewRateLimiter m_magLimiter = new SlewRateLimiter(Constants.DriveConstants.kMagnitudeSlewRate);
  private SlewRateLimiter m_rotLimiter = new SlewRateLimiter(Constants.DriveConstants.kRotationalSlewRate);
  private double m_prevTime = WPIUtilJNI.now() * 1e-6;

  // Odometry class for tracking robot pose
  SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
    DriveConstants.kDriveKinematics,
    Rotation2d.fromDegrees(m_Imu.getYaw()),
    new SwerveModulePosition[] {
      m_LeftFront.getPosition(),
      m_RightFront.getPosition(),
      m_LeftBack.getPosition(),
      m_RightBack.getPosition()
    }
  );

  /** Creates a new ExampleSubsystem. */
  public Swerve() {
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public CommandBase exampleMethodCommand(Swerve swerve, Trajectory trajectory) {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
      () -> {
        // Configure trajectory
        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
          Constants.AutoConstants.kMaxSpeedMetersPerSecond,
          Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
          // Add kinematics to ensure max speed is actually obeyed
          .setKinematics(Constants.DriveConstants.kDriveKinematics);

        var thetaController = new ProfiledPIDController(
          Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints
        );

        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
          trajectory,
          swerve::getPose, // Functional interface to feed supplier
          Constants.DriveConstants.kDriveKinematics,

          // Position controllers
          new PIDController(Constants.AutoConstants.kPXController, 0, 0),
          new PIDController(Constants.AutoConstants.kPYController, 0, 0),
          thetaController,
          swerve::setModuleStates,
          swerve
        );

        // Reset odometry to the starting pose of the trajectory.
        swerve.resetOdometry(trajectory.getInitialPose());

        // Run path following command, then stop at the end.
        swerveControllerCommand.andThen(() -> swerve.drive(0, 0, 0, false, false));
      }
    );
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a
   * digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(
      Rotation2d.fromDegrees(m_Imu.getYaw()),
      new SwerveModulePosition[] {
        m_LeftFront.getPosition(),
        m_RightFront.getPosition(),
        m_LeftBack.getPosition(),
        m_RightBack.getPosition()
      }
    );
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(
      Rotation2d.fromDegrees(m_Imu.getYaw()),
      new SwerveModulePosition[] {
        m_LeftFront.getPosition(),
        m_RightFront.getPosition(),
        m_LeftBack.getPosition(),
        m_RightBack.getPosition()
      },
      pose
    );
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param ixSpeed        Speed of the robot in the x direction (forward).
   * @param iySpeed        Speed of the robot in the y direction (sideways).
   * @param iRot           Angular rate of the robot.
   * @param iFieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   * @param iRateLimit     Whether to enable rate limiting for smoother control.
   */
  public void drive(double ixSpeed, double iySpeed, double iRot, boolean iFieldRelative, boolean iRateLimit) {
    
    double ixSpeedCommanded;
    double iySpeedCommanded;

    if (iRateLimit) {
      // Convert XY to polar for rate limiting
      double inputTranslationDir = Math.atan2(iySpeed, ixSpeed);
      double inputTranslationMag = Math.sqrt(Math.pow(ixSpeed, 2) + Math.pow(iySpeed, 2));

      // Calculate the direction slew rate based on an estimate of the lateral acceleration
      double directionSlewRate;
      if (m_currentTranslationMag != 0.0) {
        directionSlewRate = Math.abs(DriveConstants.kDirectionSlewRate / m_currentTranslationMag);
      } else {
        directionSlewRate = 500.0; //some high number that means the slew rate is effectively instantaneous
      }
      

      double currentTime = WPIUtilJNI.now() * 1e-6;
      double elapsedTime = currentTime - m_prevTime;
      double angleDif = SwerveUtils.AngleDifference(inputTranslationDir, m_currentTranslationDir);
      if (angleDif < 0.45 * Math.PI) {
        m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
      }
      else if (angleDif > 0.85 * Math.PI) {
        if (m_currentTranslationMag > 1e-4) { //some small number to avoid floating-point errors with equality checking
          // keep currentTranslationDir unchanged
          m_currentTranslationMag = m_magLimiter.calculate(0.0);
        }
        else {
          m_currentTranslationDir = SwerveUtils.WrapAngle(m_currentTranslationDir + Math.PI);
          m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
        }
      }
      else {
        m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(0.0);
      }
      m_prevTime = currentTime;
      
      ixSpeedCommanded = m_currentTranslationMag * Math.cos(m_currentTranslationDir);
      iySpeedCommanded = m_currentTranslationMag * Math.sin(m_currentTranslationDir);
      m_currentRotation = m_rotLimiter.calculate(iRot);


    } else {
      ixSpeedCommanded = ixSpeed;
      iySpeedCommanded = iySpeed;
      m_currentRotation = iRot;
    }

    // Convert the commanded speeds into the correct units for the drivetrain
    double ixSpeedDelivered = ixSpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond;
    double iySpeedDelivered = iySpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond;
    double iRotDelivered = m_currentRotation * DriveConstants.kMaxAngularSpeed;

    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        iFieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(ixSpeedDelivered, iySpeedDelivered, iRotDelivered, Rotation2d.fromDegrees(m_Imu.getYaw()))
            : new ChassisSpeeds(ixSpeedDelivered, iySpeedDelivered, iRotDelivered));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_LeftFront.setDesiredState(swerveModuleStates[0]);
    m_RightFront.setDesiredState(swerveModuleStates[1]);
    m_LeftBack.setDesiredState(swerveModuleStates[2]);
    m_RightBack.setDesiredState(swerveModuleStates[3]);
  }

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setX() {
    m_LeftFront.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_RightFront.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_LeftBack.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_RightBack.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] iDesiredState) {
    SwerveDriveKinematics.desaturateWheelSpeeds(iDesiredState, DriveConstants.kMaxSpeedMetersPerSecond);
    m_LeftFront.setDesiredState(iDesiredState[0]);
    m_RightFront.setDesiredState(iDesiredState[1]);
    m_LeftBack.setDesiredState(iDesiredState[2]);
    m_RightBack.setDesiredState(iDesiredState[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_LeftFront.resetEncoders();
    m_LeftBack.resetEncoders();
    m_RightFront.resetEncoders();
    m_RightBack.resetEncoders();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return Rotation2d.fromDegrees(m_Imu.getYaw()).getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    double[] xyz = {};
    m_Imu.getRawGyro(xyz);
    return xyz[2] * (DriveConstants.kImuReversed ? -1.0 : 1.0);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
