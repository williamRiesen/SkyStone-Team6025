package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference
import org.firstinspires.ftc.robotcore.external.navigation.Orientation
import kotlin.math.PI
import kotlin.math.absoluteValue

const val mmPerInch = 25.4

class TurtleDozer3(hardwareMap: HardwareMap, val telemetry: Telemetry) {
    val visualNavigator = VisualNavigator(hardwareMap)
    val tailHook: Servo? = hardwareMap.get(Servo::class.java, "tailhook")
    val rightFrontDrive: DcMotor? = hardwareMap.get(DcMotor::class.java, "rightFrontDrive")
    val leftFrontDrive: DcMotor? = hardwareMap.get(DcMotor::class.java, "leftFrontDrive")
    val rightRearDrive: DcMotor? = hardwareMap.get(DcMotor::class.java, "rightRearDrive")
    val leftRearDrive: DcMotor? = hardwareMap.get(DcMotor::class.java, "leftRearDrive")
    var xPosition = 0.0
    var yPosition = 0.0
    var xTarget = 0.0
    var yTarget = 0.0
    var heading = 0.0
    var yDragTarget = 0.0

    val allMotors = listOf(rightFrontDrive, leftFrontDrive, rightRearDrive, leftRearDrive)
    var lastSavedHeading = 0.0
    var desiredHeading = PI / 2.0
    val driftAngleTolerance = PI / 8.0

    val isCorrecting
        get() =
            heading in lastSavedHeading..desiredHeading || heading in desiredHeading..lastSavedHeading

    private fun trialPull() {
        lastSavedHeading = heading
        while ((heading - lastSavedHeading).absoluteValue < driftAngleTolerance) {
            updateSighting()
        }
        if (isCorrecting) {
            // keep correcting until reaching targetheading
        } else {
            // stop and slide some more
        }
    }

    private fun slideOver(inches: Double) {
//        telemetry.addData("Status","slideOver called.")
//        telemetry.update()
//        Thread.sleep(1000)
//        val slideTarget = Vector(xPosition + inches, yPosition, name = "Slide over")
//        visuallyNavigateTo(slideTarget)
    }

    fun dragWithVisualCorrection(y: Double, telemetry: Telemetry) {
        yDragTarget = y
        desiredHeading = PI / 2.0
        val increment = 4.0
        updateSighting()

        while ((yDragTarget - yPosition).absoluteValue >= arrivalTolerance) {
            setDriveMotion(FORWARD)

            val driftAngle = desiredHeading - heading
            telemetry.addData("Heading", heading * 180 /PI)
            telemetry.update()
            when {
                driftAngle in -driftAngleTolerance..driftAngleTolerance -> {
                    val driveCommand = DriveCommand(xSpeed = 0.0, ySpeed = 0.5, rotationSpeed = 0.0)

                    setDriveMotion(driveCommand)
                }
                driftAngle < -driftAngleTolerance -> slideOver(increment)
                driftAngle > driftAngleTolerance -> slideOver(-increment)
            }
//            trialPull()
//            if (!isCorrecting) {
//                when {
//                    driftAngle < -driftAngleTolerance -> slideOver(increment)
//                    driftAngle > driftAngleTolerance -> slideOver(-increment)
//                }
//            }
            updateSighting()
        }
        setDriveMotion(STOP)
    }


    private val motorsBusy: Boolean
        get() {
            var checkMotor = false
            for (motor in allMotors) {
                if (motor != null) {
                    if (motor.isBusy) checkMotor = true
                }
            }
            return false // when actual robot is available, change return from false to checkMotor
        }

    fun driveByEncoder(vector: Vector) {
        val x = vector.x * TICKS_PER_INCH
        val y = vector.y * TICKS_PER_INCH
        val rightFront = (-x + y) * ONE_OVER_SQRT2
        val leftFront = (-x - y) * ONE_OVER_SQRT2
        val rightRear = (x + y) * ONE_OVER_SQRT2
        val leftRear = (x - y) * ONE_OVER_SQRT2
        val timer = ElapsedTime()

        for (motor in allMotors) {
            if (motor != null) {
                motor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
                motor.mode = DcMotor.RunMode.RUN_TO_POSITION
            }
        }

        if (rightFrontDrive != null) {
            rightFrontDrive.targetPosition = rightFront.toInt()
        }
        if (leftFrontDrive != null) {
            leftFrontDrive.targetPosition = leftFront.toInt()
        }
        if (rightRearDrive != null) {
            rightRearDrive.targetPosition = rightRear.toInt()
        }
        if (leftRearDrive != null) {
            leftRearDrive.targetPosition = leftRear.toInt()
        }

        timer.reset()

        telemetry.addData("Right Front Drive Target Position", rightFrontDrive?.targetPosition)
        telemetry.addData("Right Front Drive Current Position", rightFrontDrive?.currentPosition)
        telemetry.addData("Right Front Drive Power", rightFrontDrive?.power)
        telemetry.addData("Right Front Drive isBusy", rightFrontDrive?.isBusy)
        telemetry.update()

        while (motorsBusy) {
            val momentaryPower =
                    if (timer.milliseconds() < RAMP_UP_MS) timer.milliseconds() / RAMP_UP_MS * vector.speed
                    else vector.speed
            for (motor in allMotors) if (motor != null) {
                motor.power = momentaryPower
            }
        }
        for (motor in allMotors) if (motor != null) {
            motor.power = 0.0
        }
    }

    fun setDriveMotion(command: DriveCommand) {
        for (motor in allMotors) if (motor != null) {
            motor.mode = DcMotor.RunMode.RUN_USING_ENCODER
        }
        val xSpeedScaled = command.xSpeed * ONE_OVER_SQRT2
        val ySpeedScaled = command.ySpeed * ONE_OVER_SQRT2
        if (rightFrontDrive != null) {
            rightFrontDrive.power = -xSpeedScaled + ySpeedScaled - command.rotationSpeed
        }
        if (leftFrontDrive != null) {
            leftFrontDrive.power = -xSpeedScaled - ySpeedScaled - command.rotationSpeed
        }
        if (rightRearDrive != null) {
            rightRearDrive.power = xSpeedScaled + ySpeedScaled - command.rotationSpeed
        }
        if (leftRearDrive != null) {
            leftRearDrive.power = xSpeedScaled - ySpeedScaled - command.rotationSpeed
        }
        telemetry.addData("Heading", heading * 180 /PI)
        telemetry.addData("setDriveMotion", command)
        telemetry.update()
    }

    fun deployHook() {
        if (tailHook != null) {
            tailHook.position = 0.0
        }
        showStatus("Hook deployed")
    }

    fun unlatchHook() {
        if (tailHook != null) {
            tailHook.position = 0.5
        }
        showStatus("Hook unlatched")
    }

    fun showStatus(string: String) {
        telemetry.addData("Status", string)
        telemetry.update()
    }

    fun updateSighting(): Boolean {
        val position = visualNavigator.getCurrentPosition()
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

    fun visuallyNavigateTo(vector: Vector) {
        xTarget = vector.x.toDouble()
        yTarget = vector.y.toDouble()
        updateSighting()

        while (!atTarget) {
            val driveCommand = DriveCommand(xTarget - xPosition, yTarget - yPosition, 0.0)
            telemetry.addData("Step", vector.name)
            telemetry.addData("Drive command", driveCommand)
            telemetry.update()
            updateSighting()
        }
        showStatus("Arrived.")
    }


    private val atTarget
        get() =
            (xTarget - xPosition).absoluteValue <= arrivalTolerance &&
                    (yTarget - yPosition).absoluteValue <= arrivalTolerance

}
