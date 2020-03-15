package frc.team4069.robot

import com.revrobotics.CANSparkMax
import com.revrobotics.ControlType
import edu.wpi.first.wpilibj.controller.PIDController
import edu.wpi.first.wpilibj.geometry.Rotation2d
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState
import java.lang.Math.abs

class SwerveModule(val linearMotor: CANSparkMax, val turnMotor: CANSparkMax) {
    val turnEncoder = turnMotor.encoder
    val linearEncoder = linearMotor.encoder
    val turnPid = turnMotor.pidController

    var currentDemand = SwerveModuleState(0.0, Rotation2d())
        private set

    init {
        turnEncoder.position = 0.0
        turnEncoder.positionConversionFactor = 2 * Math.PI / 18.8
        turnPid.p = 0.42
        turnPid.i = 0.0005
        turnPid.d = 0.004
    }

    fun update(desiredState: SwerveModuleState) {
        if(abs(turnEncoder.position - (2 * Math.PI)) < 0.045) {
            turnEncoder.position = 0.0
        }
        println(turnEncoder.position * 360.0)
        turnPid.setReference(desiredState.angle.radians, ControlType.kPosition)
        linearMotor.set(desiredState.speedMetersPerSecond)
        currentDemand = desiredState
    }
}