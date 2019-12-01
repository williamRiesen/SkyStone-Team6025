package org.firstinspires.ftc.teamcode

class Vector(val x: Double, val y: Double,val speed: Double = 0.5, val name: String = "Unnamed Vector") {
    override fun toString(): String {
        return "($x,$y) at speed $speed"
    }
}
