package frc.team4069.robot

import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel
import edu.wpi.first.wpilibj.*
import edu.wpi.first.wpilibj.geometry.Rotation2d
import edu.wpi.first.wpilibj.geometry.Translation2d
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState
import kotlin.math.absoluteValue

object Robot : TimedRobot() {
    const val MODULE_TURN_ID = 2
    const val MODULE_SPEED_ID = 1

    val joystick = Joystick(0)

    val module = SwerveModule(CANSparkMax(MODULE_SPEED_ID, CANSparkMaxLowLevel.MotorType.kBrushless),
        CANSparkMax(MODULE_TURN_ID, CANSparkMaxLowLevel.MotorType.kBrushless))

    val swerveDrive = SwerveDriveKinematics(
        Translation2d(0.2683, 0.2683),
        Translation2d(0.2683, -0.2683),
        Translation2d(-0.2683, 0.3572)
    )

    override fun teleopPeriodic() {
        val x = -joystick.getRawAxis(1).deadband(0.15)
        val y = -joystick.getRawAxis(0).deadband(0.15)
        val turn = -joystick.getRawAxis(2).deadband(0.15)
        if(x != 0.0 || y != 0.0 || turn != 0.0) {
            val moduleStates = swerveDrive.toSwerveModuleStates(ChassisSpeeds(x, y, turn))
            SwerveDriveKinematics.normalizeWheelSpeeds(moduleStates, 1.0)
            module.update(moduleStates[0])
        }
        if(x == 0.0 && y == 0.0 && turn == 0.0) {
            module.update(SwerveModuleState(0.0, module.currentDemand.angle))
        }
    }

    fun Double.deadband(threshold: Double) = if(this.absoluteValue < threshold) 0.0 else this
}


fun main() {
    RobotBase.startRobot { Robot }
}