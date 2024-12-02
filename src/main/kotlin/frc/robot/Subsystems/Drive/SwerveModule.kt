package frc.robot.Subsystems.Drive

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.math.trajectory.TrapezoidProfile

import com.revrobotics.AbsoluteEncoder
import com.revrobotics.CANSparkMax
import com.revrobotics.RelativeEncoder
import com.revrobotics.CANSparkLowLevel.MotorType

import edu.wpi.first.units.Units.*
import frc.robot.Subsystems.DriveSubsystem

/**
 * Add your docs here.
 */ 
class SwerveModule(driveMotorChannel:Int, turningMotorChannel:Int) {    
    companion object {
        val kWheelDiameter = Meters.of(0.097)
        val kModuleMaxAngularVelocity = DriveSubsystem.kMaxAngularSpeed.baseUnitMagnitude()
        val kModuleMaxAngularAcceleration = 2 * Math.PI // radians per second squared
        val kDrivingMotorReduction = 6.12
        val kDrivePositionFactor = kWheelDiameter.baseUnitMagnitude() * Math.PI / kDrivingMotorReduction
    }
    val driveMotor = CANSparkMax(driveMotorChannel, MotorType.kBrushless)

    val turningMotor = CANSparkMax(turningMotorChannel, MotorType.kBrushless)

    val driveEncoder = driveMotor.getEncoder()
    val turningEncoder = turningMotor.getAbsoluteEncoder()

    val drivePIDController = PIDController(1.0, 0.0, 0.0)
    val turningPIDController = ProfiledPIDController(
          1.0, 0.0, 0.0,
          TrapezoidProfile.Constraints(
              kModuleMaxAngularVelocity, kModuleMaxAngularAcceleration))
              
    val driveFeedforward = SimpleMotorFeedforward(1.0, 3.0)
    val turnFeedforward = SimpleMotorFeedforward(1.0, 0.5)
    
    /**
     * Constructs a SwerveModule with a drive motor, turning motor, drive encoder and turning encoder.
     *
     * @param driveMotorChannel PWM output for the drive motor.
     * @param turningMotorChannel PWM output for the turning motor.
     */
    init {
    
        driveEncoder.setPositionConversionFactor(kDrivePositionFactor);
        driveEncoder.setVelocityConversionFactor(kDrivePositionFactor / 60.0); //devided by 60 to convert to meters per second
        
        // Limit the PID Controller's input range between -pi and pi and set the input
        // to be continuous.
        turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
    }

    /**
     * Returns the current state of the module.
     *
     * @return The current state of the module.
     */
    fun getState (): SwerveModuleState {
        return SwerveModuleState(driveEncoder.getVelocity(), Rotation2d(turningEncoder.getPosition()))
    }

    /**
     * Returns the current position of the module.
     *
     * @return The current position of the module.
     */
    fun getPosition() : SwerveModulePosition{
        return SwerveModulePosition(driveEncoder.getVelocity(), Rotation2d(turningEncoder.getPosition()))
    }

    /**
     * Sets the desired state for the module.
     *
     * @param desiredState Desired state with speed and angle.
     */
    fun setDesiredState( desiredState:SwerveModuleState) {
        val encoderRotation = Rotation2d(turningEncoder.getPosition())
    
        // Optimize the reference state to avoid spinning further than 90 degrees
        val state = SwerveModuleState.optimize(desiredState, encoderRotation)
    
        // Scale speed by cosine of angle error. This scales down movement perpendicular to the desired
        // direction of travel that can occur when modules change directions. This results in smoother
        // driving.
        state.speedMetersPerSecond *= state.angle.minus(encoderRotation).getCos()
    
        // Calculate the drive output from the drive PID controller.
        val driveOutput = drivePIDController.calculate(driveEncoder.getVelocity(), state.speedMetersPerSecond)
    
        val driveFeedforward = driveFeedforward.calculate(state.speedMetersPerSecond)
    
        // Calculate the turning motor output from the turning PID controller.
        val turnOutput = turningPIDController.calculate(turningEncoder.getPosition(), state.angle.getRadians())
    
        val turnFeedforward = turnFeedforward.calculate(turningPIDController.getSetpoint().velocity)
    
        driveMotor.setVoltage(driveOutput + driveFeedforward)
        turningMotor.setVoltage(turnOutput + turnFeedforward)
      }
}
