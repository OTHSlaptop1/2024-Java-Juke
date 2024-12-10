package frc.robot.Subsystems

import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj.smartdashboard.Field2d
import frc.robot.Subsystems.DriveSubsystem
import edu.wpi.first.networktables.NetworkTable
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.networktables.StructPublisher
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.struct.Pose2dStruct
import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.struct.Pose3dStruct
import edu.wpi.first.math.geometry.Transform2d
import edu.wpi.first.math.kinematics.SwerveDriveOdometry
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator

class OdometrySubsystem(val drive: DriveSubsystem, val field: Field2d) : SubsystemBase() {
  val odometry = drive.odometry


  val ntInst = NetworkTableInstance.getDefault()
  val odometryRobotPublisher = ntInst.getStructTopic<Pose2d>("/Odometry/Robot",Pose2dStruct()).publish()
  val odometryEstimatedRobotPublisher = ntInst.getStructTopic<Pose3d>("/Odometry/EstimatedRobot",Pose3dStruct()).publish()
  val odometryTrajectoryPublisher = ntInst.getStructTopic<Pose2d>("/Odometry/Trajectory",Pose2dStruct()).publish()
  val odometryTrajectorySetpointPublisher = ntInst.getStructTopic<Pose2d>("/Odometry/TrajectorySetpoint",Pose2dStruct()).publish()

    override fun periodic() {
        // This method will be called once per scheduler run
        var robotPose2d = odometry.update(drive.getRotation2dHeading(), drive.getModulePositions())
        odometryRobotPublisher.set(robotPose2d)
        field.setRobotPose(robotPose2d)
    }

    override fun simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  fun resetOdometry(pose : Pose2d) {
    //odometry.resetPosition(drive)
  }

  /**
   * Returns the field length.
   *
   * @return  The length of the field.
   */
  fun getFieldLength() {

  /**
   * Returns the field width.
   *
   * @return  The width of the field.
   */
  }
  fun getFieldWidth(){

  }

    
  /**
   * Returns the april tag field layout.
   *
   * @return  The april tag field layout.
   */
  fun getAprilTagFieldLayout(){}

  /**
   * Get a pose with an offset from an April Tag predefined location.
   *
   * @param ID The ID of the april tag to get the pose from.
   *
   * @param poseOffset The offset from the april tag
   *
   */
  fun getPoseFromAprilTagId(ID : Int, poseOffset : Transform2d){

  }
    /**
     * Creates a command using path planner to drive a path to the specified position
     *
     *  @param endPose                 The end position for the path.
     *  @param goalEndVelocity         The goal end velocity of the robot when reaching the target pose
     *  @param rotationDelayDistance   The distance the robot should move from the start position before attempting to rotate to the final rotation
     *
     *  @return A command pointer that of a command that will run the path to the position.
     */
  fun pathToPoseCommand(endPose: Pose2d, goalEndVelocity : Double, rotationDelayDistance : Double){

  }

  /**
  * Creates a command using path planner to drive a path to the specified position
  * that will be flipped based on the value of the path flipping supplier.
  *
  *  @param endPose                 The end position for the path.
  *  @param goalEndVelocity         The goal end velocity of the robot when reaching the target pose
  *  @param rotationDelayDistance   The distance the robot should move from the start position before attempting to rotate to the final rotation
  *
  *  @return A command pointer that of a command that will run the path to the position.
  */
  fun pathToPoseFlippedCommand(endPose: Pose2d, goalEndVelocity : Double, rotationDelayDistance : Double){

  }

}
