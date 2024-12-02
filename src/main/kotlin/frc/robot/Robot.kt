package frc.robot

import edu.wpi.first.wpilibj.TimedRobot
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.filter.SlewRateLimiter
import edu.wpi.first.wpilibj.XboxController
import frc.robot.Subsystems.DriveSubsystem

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
class Robot : TimedRobot() {
    val controller = XboxController(0)
    val xSpeedLimiter = SlewRateLimiter(3.0);
    val ySpeedLimiter = SlewRateLimiter(3.0);
    val rotSpeedLimiter = SlewRateLimiter(3.0);

    var swerve = DriveSubsystem();
    /**
     * This function is run when the robot is first started up and should be used
     * for any initialization code.
     */
    override fun robotInit() {}

    override fun robotPeriodic() {}

    override fun autonomousInit() {}

    override fun autonomousPeriodic() {
        driveWithJoystick(false)
        swerve.updateOdometry()
    }

    override fun teleopInit() {}

    override fun teleopPeriodic() {
        driveWithJoystick(true)
    }

    override fun disabledInit() {}

    override fun disabledPeriodic() {}

    override fun testInit() {}

    override fun testPeriodic() {}

    fun driveWithJoystick(fieldRelative : Boolean) {
        // Get the x speed. We are inverting this because Xbox controllers return
        // negative values when we push forward.
        var xSpeed = -xSpeedLimiter.calculate(MathUtil.applyDeadband(controller.getLeftY(), 0.02)) * DriveSubsystem.kMaxSpeed.baseUnitMagnitude()

        // Get the y speed or sideways/strafe speed. We are inverting this because
        // we want a positive value when we pull to the left. Xbox controllers
        // return positive values when you pull to the right by default.
        var ySpeed =
            -ySpeedLimiter.calculate(MathUtil.applyDeadband(controller.getLeftX(), 0.02)) * DriveSubsystem.kMaxSpeed.baseUnitMagnitude()
    
        // Get the rate of angular rotation. We are inverting this because we want a
        // positive value when we pull to the left (remember, CCW is positive in
        // mathematics). Xbox controllers return positive values when you pull to
        // the right by default.
        var rot =
            -rotSpeedLimiter.calculate(MathUtil.applyDeadband(controller.getRightX(), 0.02)) * DriveSubsystem.kMaxAngularSpeed.baseUnitMagnitude()
    
         swerve.drive(xSpeed, ySpeed, rot, fieldRelative, getPeriod())
    }
}
