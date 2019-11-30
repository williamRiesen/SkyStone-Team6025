package org.firstinspires.ftc.teamcode

class Vector(val x: Int, val y: Int,val speed: Double = 0.5, val name: String = "Unnamed Vector") {
    override fun toString(): String {
        return "($x,$y) at speed $speed"
    }
}
