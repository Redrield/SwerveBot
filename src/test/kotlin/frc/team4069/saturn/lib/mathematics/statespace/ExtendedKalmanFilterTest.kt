package frc.team4069.saturn.lib.mathematics.statespace

import edu.wpi.first.wpilibj.geometry.Pose2d
import edu.wpi.first.wpilibj.math.StateSpaceUtils
import edu.wpi.first.wpiutil.math.numbers.N2
import edu.wpi.first.wpiutil.math.numbers.N3
import edu.wpi.first.wpiutil.math.numbers.N5
import frc.team4069.saturn.lib.mathematics.matrix.*
import frc.team4069.saturn.lib.mathematics.model.gearbox
import frc.team4069.saturn.lib.mathematics.model.kMotorCim
import frc.team4069.saturn.lib.mathematics.twodim.geometry.Twist2d
import frc.team4069.saturn.lib.mathematics.units.*
import org.junit.Test
import kotlin.math.cos
import kotlin.math.pow
import kotlin.math.sin

class ExtendedKalmanFilterTest {

    fun dynamics(x: Vector<N5>, u: Vector<N2>): Vector<N5> {
        val motors = gearbox(kMotorCim, 2)

        val Ghigh = 7.08
        val rb = 0.8382.meter / 2.0
        val r = 0.0746124.meter
        val m = 63.503.kilo.gram
        val J = SIUnit<Mult<Kilogram, Mult<Meter, Meter>>>(5.6) // Moment of inertia

        val C1 = -Ghigh.pow(2) * motors.Kt.value / (motors.Kv.value * motors.R.value * r.value.pow(2))
        val C2 = Ghigh * motors.Kt.value / (motors.R.value * r.value)
        val k1 = (1 / m.value + rb.value.pow(2) / J.value)
        val k2 = (1 / m.value - rb.value.pow(2) / J.value)

        val vl = x[3].meter.velocity
        val vr = x[4].meter.velocity

        val Vl = u[0].volt
        val Vr = u[1].volt

        val result = zeros(`5`)
        val v = 0.5 * (vl + vr)
        result[0] = v.value * cos(x[2])
        result[1] = v.value * sin(x[2])
        result[2] = ((vr - vl) / (2.0 * rb)).value
        result[3] =
            k1 * ((C1 * vl.value) + (C2 * Vl.value)) +
                    k2 * ((C1 * vr.value) + (C2 * Vr.value))
        result[4] =
            k2 * ((C1 * vl.value) + (C2 * Vl.value)) +
                    k1 * ((C1 * vr.value) + (C2 * Vr.value))

        return result
    }

    fun localMeasurementModel(x: Vector<N5>, u: Vector<N2>): Vector<N3> {
        return vec(`3`).fill(x[2], x[3], x[4])
    }

    fun globalMeasurementModel(x: Vector<N5>, u: Vector<N2>): Vector<N5> {
        return x
    }

    @Test
    fun extendedKalmanFilterTest() {
        val dt = 0.00505.second

        val observer = ExtendedKalmanFilter(
            `5`, `2`, `3`, this::dynamics, this::localMeasurementModel,
            vec(`5`).fill(0.5, 0.5, 10.0, 1.0, 1.0),
            vec(`3`).fill(0.0001, 0.01, 0.01), dt
        )

        val u = vec(`2`).fill(12.0, 12.0)

        observer.predict(u, dt)

        val localY = localMeasurementModel(observer.xHat, u)
        observer.correct(u, localY)

        val globalY = globalMeasurementModel(observer.xHat, u)
        val R = StateSpaceUtils.makeCovMatrix({ 5 }, vec(`5`).fill(0.01, 0.01, 0.0001, 0.01, 0.01))
        observer.correct(`5`, u, globalY, this::globalMeasurementModel, R)
    }

    private fun pexp(twist: Vector<N3>): Vector<N3> {
        val twist = Twist2d(twist[0].meter, twist[1].meter, twist[2].radian)

        val pose = Pose2d().exp(twist)
        return vec(`3`).fill(pose.translation.x, pose.translation.y, pose.rotation.radians)
    }
}
