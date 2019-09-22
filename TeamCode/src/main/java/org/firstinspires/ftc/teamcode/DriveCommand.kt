package org.firstinspires.ftc.teamcode

// comment added to test version control

import kotlin.math.cos
import kotlin.math.sin

class DriveCommand(var xSpeed: Float, var ySpeed: Float, var rotationSpeed: Float) {

    public fun rotate(radians: Float) {
        val x = xSpeed * cos(radians) - ySpeed * sin(radians)
        val y = xSpeed * sin(radians) + ySpeed * cos(radians)
        xSpeed = x
        ySpeed = y
    }


}