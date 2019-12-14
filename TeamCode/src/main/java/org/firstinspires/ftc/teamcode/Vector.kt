package org.firstinspires.ftc.teamcode

import kotlin.math.cos
import kotlin.math.sin
import kotlin.math.sqrt

class Vector(var x: Double, var y: Double,val speed: Double = 0.5, val name: String = "Unnamed Vector") {


    override fun toString(): String {
        return "($x,$y) at speed $speed"
    }

    fun reflectOverXAxis(){
        y = -y
    }

    fun rotate(radians: Float) {
        val xRotated = x * cos(radians) - y * sin(radians)
        val yRotated = x * sin(radians) + y * cos(radians)
        x = xRotated
        y = yRotated
    }
    fun rotated(radians: Double): Vector {

        return Vector(
                x * cos(radians) - y * sin(radians),
                x * sin(radians) + y * cos(radians),
                speed,
                name)
    }
     val length
     get() = sqrt(x * x + y * y)
}
