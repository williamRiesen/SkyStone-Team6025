package org.firstinspires.ftc.teamcode

import kotlin.math.cos
import kotlin.math.sin

class DriveCommand(var xSpeed: Double, var ySpeed: Double, var rotationSpeed: Double) {

    public fun rotate(radians: Float) {
        val x = xSpeed * cos(radians) - ySpeed * sin(radians)
        val y = xSpeed * sin(radians) + ySpeed * cos(radians)
        xSpeed = x
        ySpeed = y
    }

    public fun rotated(radians: Double):DriveCommand {

        return DriveCommand(xSpeed * cos(radians) - ySpeed * sin(radians),
                xSpeed * sin(radians) + ySpeed * cos(radians),
                rotationSpeed)
    }


}