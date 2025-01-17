// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import java.util.function.DoubleSupplier;

public class CANDriveSubsystem extends SubsystemBase {
  private final VictorSPX leftLeader;
  private final VictorSPX leftFollower;
  private final VictorSPX rightLeader;
  private final VictorSPX rightFollower;

  public CANDriveSubsystem() {
    // create brushed motors for drive
    leftLeader = new VictorSPX(DriveConstants.LEFT_LEADER_ID);
    leftFollower = new VictorSPX(DriveConstants.LEFT_FOLLOWER_ID);
    rightLeader = new VictorSPX(DriveConstants.RIGHT_LEADER_ID);
    rightFollower = new VictorSPX(DriveConstants.RIGHT_FOLLOWER_ID);
    leftFollower.follow(leftLeader);
    rightFollower.follow(rightLeader);
    rightLeader.setInverted(true);
    rightFollower.setInverted(true);
  }

  @Override
  public void periodic() {}

  // Command to drive the robot with joystick inputs
  public Command driveCommand(DoubleSupplier forward, DoubleSupplier rotation) {
    return run(
        () -> {
          double left = 0;
          double right = 0;
          left = forward.getAsDouble();
          right = forward.getAsDouble();
          left -= rotation.getAsDouble();
          right += rotation.getAsDouble();
          left /= 2;
          right /= 2;
          leftLeader.set(ControlMode.PercentOutput, left);
          rightLeader.set(ControlMode.PercentOutput, right);
          SmartDashboard.putNumber("Left Drive Speed", left);
          SmartDashboard.putNumber("Right Drive Speed", right);
        });
  }
}
