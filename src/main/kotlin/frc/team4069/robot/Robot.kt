package frc.team4069.robot

import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.TimedRobot

object Robot : TimedRobot() {

    override fun robotInit() {
        for(id in 0..63) {
            val spark = CANSparkMax(id, CANSparkMaxLowLevel.MotorType.kBrushless)
            if(spark.firmwareString != "v0.0.0") {
                println(id)
            }
        }
    }
}

fun main() {
    RobotBase.startRobot { Robot }
}