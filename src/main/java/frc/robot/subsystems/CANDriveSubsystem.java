package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.sim.Pigeon2SimState;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

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

  // Simulation variables
  private EncoderSim leftEncoderSim;
  private EncoderSim rightEncoderSim;
  private Pigeon2SimState gyroSim;
  private DifferentialDrivetrainSim diffDriveSim;

  private Field2d field;

  public CANDriveSubsystem() {
    field = new Field2d();
    SmartDashboard.putData("Field", field); // Show the field on SmartDashboard

    // Create brushed motors for drive
    leftLeader = new VictorSPX(DriveConstants.LEFT_LEADER_ID);
    leftFollower = new VictorSPX(DriveConstants.LEFT_FOLLOWER_ID);
    rightLeader = new VictorSPX(DriveConstants.RIGHT_LEADER_ID);
    rightFollower = new VictorSPX(DriveConstants.RIGHT_FOLLOWER_ID);

    leftEncoder =
        new Encoder(DriveConstants.LEFT_DRIVE_ENCODER_A, DriveConstants.LEFT_DRIVE_ENCODER_B);
    rightEncoder =
        new Encoder(DriveConstants.RIGHT_DRIVE_ENCODER_A, DriveConstants.RIGHT_DRIVE_ENCODER_B);

    leftEncoder.setDistancePerPulse(
        Math.PI * DriveConstants.WHEEL_DIAMETER_METERS / DriveConstants.ENCODER_RESOLUTION);
    rightEncoder.setDistancePerPulse(
        Math.PI * DriveConstants.WHEEL_DIAMETER_METERS / DriveConstants.ENCODER_RESOLUTION);

    leftEncoderSim = new EncoderSim(leftEncoder);
    rightEncoderSim = new EncoderSim(rightEncoder);

    rightLeader.setInverted(false);
    rightFollower.setInverted(InvertType.FollowMaster);

    leftLeader.setInverted(true);
    leftFollower.setInverted(InvertType.FollowMaster);

    gyro = new Pigeon2(DriveConstants.PIGEON_DEVICE_ID);
    gyroSim = new Pigeon2SimState(gyro);

    kinematics = new DifferentialDriveKinematics(DriveConstants.TRACK_WIDTH_METERS);

    diffDrive =
        new DifferentialDrive(
            (speed) -> leftLeader.set(ControlMode.PercentOutput, speed),
            (speed) -> rightLeader.set(ControlMode.PercentOutput, speed));

    // Create drivetrain simulator
    diffDriveSim =
        new DifferentialDrivetrainSim(
            DCMotor.getNEO(2),
            DriveConstants.GEARING,
            DriveConstants.MOI,
            DriveConstants.MASS_KILOGRAMS,
            DriveConstants.WHEEL_DIAMETER_METERS,
            DriveConstants.TRACK_WIDTH_METERS,
            null);

    // Create new odometry object
    driveOdometry =
        new DifferentialDriveOdometry(
            gyro.getRotation2d(), leftEncoder.getDistance(), rightEncoder.getDistance());
  }

  // Pigeon Functions for Odometry
  public void resetOdometry(Pose2d resetPose) {
    leftEncoderSim.resetData();
    rightEncoderSim.resetData(); // Reset encoder values
    driveOdometry.resetPosition(
        gyro.getRotation2d(),
        leftEncoderSim.getDistance(),
        rightEncoderSim.getDistance(),
        resetPose);
  }

  public Pose2d getPosition() {
    return position;
  }

  // Get Current Speed
  public ChassisSpeeds getCurrentSpeeds() {
    return kinematics.toChassisSpeeds(wheelSpeeds);
  }

  public void driveRobotRelative(ChassisSpeeds relativeSpeeds) {
    diffDrive.arcadeDrive(relativeSpeeds.vyMetersPerSecond, relativeSpeeds.omegaRadiansPerSecond);
  }

  @Override
  public void periodic() {
    // Update position using odometry
    position =
        driveOdometry.update(
            gyro.getRotation2d(), leftEncoder.getDistance(), rightEncoder.getDistance());

    wheelSpeeds = new DifferentialDriveWheelSpeeds(leftEncoder.getRate(), rightEncoder.getRate());

    SmartDashboard.putNumber("LEFT ENCODER RATE", leftEncoder.getRate());
    SmartDashboard.putNumber("RIGHT ENCODER RATE", rightEncoder.getRate());

    field.setRobotPose(position);
  }

  @Override
  public void simulationPeriodic() {
    // Simulate the motor inputs to the drivetrain
    diffDriveSim.setInputs(
        leftLeader.getBusVoltage() * RobotController.getInputVoltage(),
        rightLeader.getBusVoltage() * RobotController.getInputVoltage());

    // Update the simulation state
    diffDriveSim.update(0.02); // Update at 20ms intervals

    // Update encoder and gyro states for simulation
    leftEncoderSim.setDistance(
        diffDriveSim.getLeftPositionMeters()); // Set distance from simulation
    leftEncoderSim.setRate(
        diffDriveSim.getLeftVelocityMetersPerSecond()); // Set rate from simulation
    rightEncoderSim.setDistance(
        diffDriveSim.getRightPositionMeters()); // Set distance from simulation
    rightEncoderSim.setRate(
        diffDriveSim.getRightVelocityMetersPerSecond()); // Set rate from simulation
    gyroSim.setRawYaw(diffDriveSim.getHeading().getDegrees()); // Update simulated gyro

    Logger.recordOutput("POSE", driveOdometry.getPoseMeters());

    // Update odometry in simulation
    driveOdometry.update(
        gyro.getRotation2d(), leftEncoderSim.getDistance(), rightEncoderSim.getDistance());
  }

  // Telemetry Commands
  public Command arcadeDrive(DoubleSupplier forward, DoubleSupplier rotation) {
    return run(() -> diffDrive.arcadeDrive(forward.getAsDouble(), rotation.getAsDouble()));
  }

  public Command stopRobotCommand() {
    return run(() -> diffDrive.arcadeDrive(0.0, 0.0));
  }
}
