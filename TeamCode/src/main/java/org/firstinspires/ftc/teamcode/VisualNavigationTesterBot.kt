package org.firstinspires.ftc.teamcode


import android.content.Context
import com.qualcomm.ftccommon.SoundPlayer
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference
import org.firstinspires.ftc.robotcore.external.navigation.Orientation
import kotlin.math.*

const val startingXPosition = 0.0
const val startingYPosition = 0.0
const val startingHeading = 0.0
const val arrivalTolerance = 2.0
const val DEGREES_PER_RADIAN = 180.0 / PI

class VisualNavigationTesterBot(hardwareMap: HardwareMap) {

    val visualNavigator = VisualNavigator(hardwareMap)

    var xPosition = startingXPosition
    var yPosition = startingYPosition
    var xTarget = startingXPosition
    var yTarget = startingYPosition
    var heading = startingHeading

    private val atTarget: Boolean
        get() {
            val xAtTarget = (xPosition - xTarget).absoluteValue <= arrivalTolerance
            val yAtTarget = (yPosition - yTarget).absoluteValue <= arrivalTolerance
            return xAtTarget && yAtTarget
        }

    private fun updateSighting(): Boolean {
        val position = visualNavigator.getCurrentPosition()
        val mmPerInch = 25.4
        if (position == null) return false
        else {
            val translation = position.translation
            xPosition = translation.get(0) / mmPerInch
            yPosition = translation.get(1) / mmPerInch
            // express the rotation of the robot in degrees.
            val rotation: Orientation = Orientation.getOrientation(position, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS)
            heading = rotation.thirdAngle.toDouble()
            return true
        }
    }

    fun autoDriveTo(newXTarget: Double, newYTarget: Double, telemetry: Telemetry) {
        xTarget = newXTarget
        yTarget = newYTarget
        updateSighting()
        while (!atTarget) {
            val bearingToTarget = atan2(yTarget - yPosition, xTarget - xPosition)
            val rangeToTarget = sqrt((yTarget - yPosition).pow(2) + (xTarget - xPosition).pow(2))
            val driveCommand = FORWARD.rotated(bearingToTarget)

//            telemetry.addData("Position", "{x, y} = %.1f, %.1f", xPosition, yPosition)
            telemetry.addData("Bearing to Target", bearingToTarget  * DEGREES_PER_RADIAN)
            telemetry.addData("Heading","%.0f", heading  * DEGREES_PER_RADIAN)
            telemetry.addData("Turn by", "%.2f", (bearingToTarget-heading)  * DEGREES_PER_RADIAN)
            telemetry.addData("Range to target", "%.1f", rangeToTarget)
            telemetry.update()
            updateSighting()
        }
    }

    private val myApp: Context = hardwareMap.appContext
    private val params = SoundPlayer.PlaySoundParams()
    fun playSound(sound: Sound) {

        var soundID = -1
        var soundPlaying = false
        // obtain sound file identifier if a valid one exists
        if (myApp.resources.getIdentifier(sound.resourceName, "raw", myApp.packageName).also { soundID = it } != 0) {
            // Signal that the sound is now playing.
            soundPlaying = true
            // Start playing, and also Create a callback that will clear the playing flag when the sound is complete.
            SoundPlayer.getInstance().startPlaying(myApp, soundID, params, null,
                    Runnable { soundPlaying = false })
        }
    }
}
