package frc.team4069.test

import frc.team4069.robot.subsystems.flywheel.FlywheelController
import frc.team4069.saturn.lib.mathematics.matrix.*
import frc.team4069.saturn.lib.mathematics.units.radian
import frc.team4069.saturn.lib.mathematics.units.second
import frc.team4069.saturn.lib.mathematics.units.velocity
import org.junit.Assert.assertTrue
import org.junit.Test
import org.knowm.xchart.QuickChart
import org.knowm.xchart.SwingWrapper
import java.util.*

class FlywheelControllerTest {
    val controller = FlywheelController()

    @Test
    fun testControllerEnabled() {
        controller.enable()
        controller.reference = 350.radian.velocity
        val rand = Random()

        val states = mutableListOf<Double>()
        val inputs = mutableListOf<Double>()
        val times = mutableListOf<Double>()

        var t = 0.0

        // 150 timesteps at 0.01s dt
        for(_i in 0..150) {
            t += 0.01
            controller.measuredVelocity = (controller.plant.x[0] + rand.nextGaussian() * 7.5).radian.velocity
            val u = controller.update(0.01.second)
            controller.plant.update(controller.plant.x, mat(`1`, `1`).fill(u.value), 0.01)
            states += controller.velocity.value
            inputs += u.value
            times += t
        }

        val statesChart = QuickChart.getChart("Velocity vs time", "Time", "Velocity", "Velocity", times, states)
        val inputsChart = QuickChart.getChart("Control Effort", "Time", "Voltage", "Control Effort", times, inputs)
//        SwingWrapper(listOf(statesChart, inputsChart)).displayChartMatrix()
//        while(true) {
//            Thread.sleep(10000)
//        }
        assertTrue("Controller should converge to reference. Reference ${controller.reference.value}, velocity ${controller.velocity.value}", controller.atGoal)
    }
}