package frc.team4069.robot

import com.revrobotics.CANSparkMax
import com.revrobotics.ControlType
import edu.wpi.first.wpilibj.controller.PIDController
import edu.wpi.first.wpilibj.geometry.Rotation2d
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import kotlin.math.abs
import kotlin.math.floor

class SwerveModule(val linearMotor: CANSparkMax, val turnMotor: CANSparkMax) {
    val turnEncoder = turnMotor.encoder
    val linearEncoder = linearMotor.encoder
//    val turnPid = turnMotor.pidController

    var currentDemand = SwerveModuleState(0.0, Rotation2d())
        private set

    val turnPid = PIDController(0.42, 0.0005, 0.004, 0.02)

    init {
        turnEncoder.position = 0.0
        turnEncoder.positionConversionFactor = 2 * Math.PI / 18.8
        turnPid.enableContinuousInput(0.0, 2 * Math.PI)
    }

    fun update(desiredState: SwerveModuleState) {
        val currentPos = norm(turnEncoder.position)
//        if(abs(2 * Math.PI - currentPos) < Math.toRadians(5.0)) {
//            turnEncoder.position = 0.0
//            currentPos = 0.0
//        }
        // Norm angle to 0..tau
        desiredState.angle = Rotation2d(norm(desiredState.angle.radians))

        SmartDashboard.putNumber("Module Position", Math.toDegrees(currentPos))
        SmartDashboard.putString("Desired State", desiredState.toString())

        val currentReversed = norm(currentPos + Math.PI)
        var angleDiff = abs(desiredState.angle.radians - currentPos)
        var reversedDiff = abs(desiredState.angle.radians - currentReversed)
        angleDiff = if(angleDiff > Math.PI) (2 * Math.PI) - angleDiff else angleDiff
        reversedDiff = if(reversedDiff > Math.PI) (2 * Math.PI) - reversedDiff else reversedDiff

        if(reversedDiff < angleDiff) {
            desiredState.angle = Rotation2d(norm(desiredState.angle.radians + Math.PI))
            desiredState.speedMetersPerSecond = -desiredState.speedMetersPerSecond
        }

        turnMotor.set(turnPid.calculate(currentPos, desiredState.angle.radians))
        linearMotor.set(desiredState.speedMetersPerSecond)
        currentDemand = desiredState
        SmartDashboard.putBoolean("Module at setpoint", turnPid.atSetpoint())
    }

    private fun norm(angle: Double): Double {
        return angle - 2 * Math.PI * floor(angle / (2 * Math.PI))
    }
}