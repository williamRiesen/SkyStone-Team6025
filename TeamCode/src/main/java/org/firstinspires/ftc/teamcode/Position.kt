package org.firstinspires.ftc.teamcode

import kotlin.math.PI
import kotlin.math.absoluteValue

const val MAX_SPEED = 500 // ticks per second

class Position(var x: Float, var y: Float, var heading: Float) {

    operator fun minus(subtrahend: Position): Position {
        val difference = Position(
                this.x - subtrahend.x,
                this.y - subtrahend.y,
                this.heading - subtrahend.heading
        )
        return difference
    }

    fun allTermsLessThan(tolerance: Position) = this.x.absoluteValue > tolerance.x.absoluteValue
            && this.y.absoluteValue > tolerance.y.absoluteValue
            && this.heading.absoluteValue > tolerance.heading.absoluteValue

    fun calculateTravelTime() {
        var minTimeLinear = if (this.x > this.y) {
            this.x / MAX_SPEED
        } else {
            this.y / MAX_SPEED
        }
        var minTimeAngular = this.heading * 2 * PI / MAX_SPEED
    }
}


