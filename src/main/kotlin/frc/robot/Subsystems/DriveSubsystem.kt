package frc.robot.Subsystems

import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.units.Units.*

import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.kinematics.SwerveDriveOdometry
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.wpilibj.AnalogGyro

import frc.robot.Subsystems.Drive.SwerveModule

/**
 * Add your docs here.
 */
class DriveSubsystem : SubsystemBase() {
    companion object {
        val kMaxSpeed = MetersPerSecond.of(3.0)
        val kMaxAngularSpeed = RadiansPerSecond.of(2*Math.PI)
    }

    val frontLeftLocation =  Translation2d(0.381, 0.381)
    val frontRightLocation =  Translation2d(0.381, -0.381)
    val backLeftLocation =  Translation2d(-0.381, 0.381)
    val backRightLocation =  Translation2d(-0.381, -0.381)
  
    val frontLeft =  SwerveModule(2, 1)
    val frontRight =  SwerveModule(4, 3)
    val backLeft =  SwerveModule(8, 7)
    val backRight =  SwerveModule(6, 5)
    fun modules():Array<SwerveModulePosition> {
        return arrayOf(
        frontLeft.getPosition(),
        frontRight.getPosition(), 
        backLeft.getPosition(),
        backRight.getPosition()
      )
    }
  
    val gyro =  AnalogGyro(10)
  
    val kinematics =
         SwerveDriveKinematics(
            frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation)
  
    val odometry =
         SwerveDriveOdometry(
            kinematics,
            gyro.getRotation2d(),
            modules())

    init {
        gyro.reset()
    }
    /**
     * Method to drive the robot using joystick info.
     *
     * @param xSpeed Speed of the robot in the x direction (forward).
     * @param ySpeed Speed of the robot in the y direction (sideways).
     * @param rot Angular rate of the robot.
     * @param fieldRelative Whether the provided x and y speeds are relative to the field.
     */
    fun drive(xSpeed : Double , ySpeed : Double, rot : Double, fieldRelative : Boolean, periodSeconds : Double) {
        var swerveModuleStates =
        kinematics.toSwerveModuleStates(
            ChassisSpeeds.discretize(
                if (fieldRelative)
                    {ChassisSpeeds.fromFieldRelativeSpeeds(
                        xSpeed, ySpeed, rot, gyro.getRotation2d())}
                else 
                    {ChassisSpeeds(xSpeed, ySpeed, rot)},
                periodSeconds))
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed)
        frontLeft.setDesiredState(swerveModuleStates[0])
        frontRight.setDesiredState(swerveModuleStates[1]) 
        backLeft.setDesiredState(swerveModuleStates[2])
        backRight.setDesiredState(swerveModuleStates[3])
    }

    /** Updates the field relative position of the robot. */
    fun updateOdometry(){
        odometry.update(
            gyro.getRotation2d(),
            modules())
    }
    
    override fun periodic() {
        // This method will be called once per scheduler run
    }

    override fun simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}
