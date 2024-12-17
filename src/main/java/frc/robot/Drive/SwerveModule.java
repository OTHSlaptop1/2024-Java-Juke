// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkBase;

public class SwerveModule {
  private static final double kWheelDiameter = 0.097; // in meters

  //private static final double kModuleMaxAngularVelocity = DriveSubsystem.kMaxAngularSpeed.baseUnitMagnitude();
  //private static final double kModuleMaxAngularAcceleration = 2 * Math.PI; // radians per second squared

  private static final double kDrivingMotorFreeSpeedRps = 5676.0 / 60; //NEO free speed is 5676 RPM
  private static final double kWheelCircumference = kWheelDiameter * Math.PI;

  private static final double kDrivingMotorReduction = 6.12;
  private static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumference / kDrivingMotorReduction);

  private static final double kDrivePositionFactor = kWheelDiameter * Math.PI / kDrivingMotorReduction;

  private final CANSparkMax m_driveMotor;
  private final CANSparkMax m_turningMotor;

  private final RelativeEncoder m_driveEncoder;
  private final AbsoluteEncoder m_turningEncoder;

  // Gains are for example purposes only - must be determined for your own robot!
  private SparkPIDController m_drivePIDController;
  private SparkPIDController m_turnPIDController;

  /**
   * Constructs a SwerveModule with a drive motor, turning motor, drive encoder and turning encoder.
   *
   * @param driveMotorChannel PWM output for the drive motor.
   * @param turningMotorChannel PWM output for the turning motor.
   */
  public SwerveModule(
      int driveMotorChannel,
      int turningMotorChannel) {
    m_driveMotor = new CANSparkMax(driveMotorChannel, MotorType.kBrushless);
    m_turningMotor = new CANSparkMax(turningMotorChannel, MotorType.kBrushless);

    m_drivePIDController = m_driveMotor.getPIDController();
    m_turnPIDController = m_turningMotor.getPIDController();

    m_driveEncoder = m_driveMotor.getEncoder();
    m_turningEncoder = m_turningMotor.getAbsoluteEncoder();

    m_drivePIDController.setFeedbackDevice(m_driveEncoder);

    m_drivePIDController.setP(.04); //insert values later
    m_drivePIDController.setI(0);
    m_drivePIDController.setD(0);
    m_drivePIDController.setFF(1 / kDriveWheelFreeSpeedRps);
    m_drivePIDController.setOutputRange(-1, 1);

    m_turnPIDController.setFeedbackDevice(m_turningEncoder);

    m_turnPIDController.setP(.75);
    m_turnPIDController.setI(0);
    m_turnPIDController.setD(.01);
    m_turnPIDController.setFF(0);
    m_turnPIDController.setOutputRange(-1, 1);

    m_turnPIDController.setPositionPIDWrappingEnabled(true);
    m_turnPIDController.setPositionPIDWrappingMinInput(0);
    m_turnPIDController.setPositionPIDWrappingMaxInput(2* Math.PI);

    m_driveEncoder.setPositionConversionFactor(kDrivePositionFactor);
    m_driveEncoder.setVelocityConversionFactor(kDrivePositionFactor / 60.0); //devided by 60 to convert to meters per second
    


    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    //m_turnPIDController.enableContinuousInput(-Math.PI, Math.PI);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(
        m_driveEncoder.getVelocity(), new Rotation2d(m_turningEncoder.getPosition()));
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        m_driveEncoder.getVelocity(), new Rotation2d(m_turningEncoder.getPosition()));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    var encoderRotation = new Rotation2d(m_turningEncoder.getPosition());

    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState state = SwerveModuleState.optimize(desiredState, encoderRotation);

    // Scale speed by cosine of angle error. This scales down movement perpendicular to the desired
    // direction of travel that can occur when modules change directions. This results in smoother
    // driving.
    state.speedMetersPerSecond *= state.angle.minus(encoderRotation).getCos();

    m_drivePIDController.setReference(state.speedMetersPerSecond, CANSparkBase.ControlType.kVelocity);
    m_turnPIDController.setReference(state.angle.getRadians(), CANSparkBase.ControlType.kPosition);

    // Calculate the drive output from the drive PID controller.
  }
}
