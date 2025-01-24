// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import java.util.function.DoubleSupplier;

public class CANDriveSubsystem extends SubsystemBase {
  private final VictorSPX leftLeader;
  private final VictorSPX leftFollower;
  private final VictorSPX rightLeader;
  private final VictorSPX rightFollower;

  private final Encoder leftEncoder;
  private final Encoder rightEncoder;

  private final Pigeon2 gyro;

  private final DifferentialDrive diffDrive;
  private final DifferentialDriveOdometry driveOdometry;

  private DifferentialDriveKinematics kinematics;

  private Pose2d position;

  private DifferentialDriveWheelSpeeds wheelSpeeds;

  

  public CANDriveSubsystem() {
    // create brushed motors for drive
    leftLeader = new VictorSPX(DriveConstants.LEFT_LEADER_ID);
    leftFollower = new VictorSPX(DriveConstants.LEFT_FOLLOWER_ID);
    rightLeader = new VictorSPX(DriveConstants.RIGHT_LEADER_ID);
    rightFollower = new VictorSPX(DriveConstants.RIGHT_FOLLOWER_ID);

    leftEncoder =
        new Encoder(DriveConstants.LEFT_DRIVE_ENCODER_A, DriveConstants.LEFT_DRIVE_ENCODER_B);
    rightEncoder =
        new Encoder(DriveConstants.RIGHT_DRIVE_ENCODER_A, DriveConstants.RIGHT_DRIVE_ENCODER_B);

    rightLeader.setInverted(false);
    rightFollower.setInverted(InvertType.FollowMaster);

    leftLeader.setInverted(true);
    leftFollower.setInverted(InvertType.FollowMaster);

    gyro = new Pigeon2(DriveConstants.PIGEON_DEVICE_ID);

    kinematics = new DifferentialDriveKinematics(DriveConstants.TRACK_WIDTH);

    


    diffDrive =
        new DifferentialDrive(
            (speed) -> leftLeader.set(ControlMode.PercentOutput, speed),
            (speed) -> rightLeader.set(ControlMode.PercentOutput, speed));

    // Create new odometry obkect
    driveOdometry =
        new DifferentialDriveOdometry(
            gyro.getRotation2d(), leftEncoder.getDistance(), rightEncoder.getDistance());
  }



  // Pigeon Functions for Odometry
    public void resetOdometry(Pose2d resetpose) {
      driveOdometry.resetPosition(
          gyro.getRotation2d(), leftEncoder.getDistance(), rightEncoder.getDistance(), resetpose);
    }

    public Pose2d getPosition() {
      return position;
    }



  // Get Current Speed
  public ChassisSpeeds getCurrentSpeeds() {
    return kinematics.toChassisSpeeds(wheelSpeeds);
  }


  //
  public void driveRobotRelative(ChassisSpeeds relativeSpeeds){
    diffDrive.arcadeDrive(relativeSpeeds.vyMetersPerSecond, relativeSpeeds.omegaRadiansPerSecond);
  }


  @Override
  public void periodic() {
    position =
        driveOdometry.update(
            gyro.getRotation2d(), leftEncoder.getDistance(), rightEncoder.getDistance());
    wheelSpeeds = new DifferentialDriveWheelSpeeds(leftEncoder.getRate(), rightEncoder.getRate());
  }

  //PATHPLANNER BUILDER

  

  /**
   * @param forward how much to drive intake forward
   */
  public Command arcadeDrive(DoubleSupplier forward, DoubleSupplier rotation) {
    return run(() -> diffDrive.arcadeDrive(forward.getAsDouble(), rotation.getAsDouble()));
  }
}  
