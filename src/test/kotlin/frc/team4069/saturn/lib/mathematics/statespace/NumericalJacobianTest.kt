package frc.team4069.saturn.lib.mathematics.statespace

import edu.wpi.first.wpiutil.math.numbers.N2
import edu.wpi.first.wpiutil.math.numbers.N4
import frc.team4069.saturn.lib.mathematics.matrix.*
import org.ejml.dense.row.MatrixFeatures_DDRM
import org.junit.Assert
import org.junit.Test

class NumericalJacobianTest {

    @Test
    fun testNumericalJacobian() {
        val A = mat(`4`, `4`).fill(1, 2, 4, 1,
                                    5, 2, 3, 4,
                                    5, 1, 3, 2,
                                    1, 1, 3, 7)
        val B = mat(`4`, `2`).fill(1, 1,
                                    2, 1,
                                    3, 2,
                                    3, 7)

        val AxBuFn = { x: Vector<N4>, u: Vector<N2> -> A * x + B * u }

        val newA = numericalJacobianX(`4`, `4`, AxBuFn, zeros(`4`), zeros(`2`))

        Assert.assertTrue("A and newA should be identical", MatrixFeatures_DDRM.isEquals(A.storage.ddrm, newA.storage.ddrm, 1E-5))

        val newB = numericalJacobianU(`4`, `2`, AxBuFn, zeros(`4`), zeros(`2`))
        Assert.assertTrue("B and newB should be identical", MatrixFeatures_DDRM.isEquals(B.storage.ddrm, newB.storage.ddrm, 1E-5))
    }
}