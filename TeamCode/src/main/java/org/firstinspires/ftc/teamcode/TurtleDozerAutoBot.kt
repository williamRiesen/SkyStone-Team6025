package org.firstinspires.ftc.teamcode

import com.qualcomm.hardware.rev.RevBlinkinLedDriver
import com.qualcomm.robotcore.hardware.CRServo
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
import kotlin.math.atan2

const val MAX_POWER = 0.75
val UNIT = DriveCommand(0.0, 1.0, 0.0)
val SLOW_FORWARD = DriveCommand(0.0, 0.75, 0.0)
val FORWARD = DriveCommand(0.0, 0.5, 0.0)
val STOP = DriveCommand(0.0, 0.0, 0.0)
const val arrivalTolerance = 2.0
const val TICKS_PER_INCH = 128
const val ONE_OVER_SQRT2 = 0.70710678118
const val INCHES_PER_SEC_PER_POWER_UNIT = 20.5
var moveOnFieldCommand = DriveCommand(0.0, 0.0, 0.0)
val driveCommand = DriveCommand(0.0, 0.0, 0.0)
val headingTolerance = PI / 90.0
val clawRestPosition = 0.35

class TurtleDozerAutoBot3(hardwareMap: HardwareMap, val telemetry: Telemetry) {
    //    private val visualNavigator = VisualNavigator(hardwareMap)
    val inertialMotionUnit: InertialMotionUnit = InertialMotionUnit(hardwareMap)
    //    var parameters: BNO055IMU.Parameters = BNO055IMU.Parameters()
    private val tailHook: Servo? = hardwareMap.get(Servo::class.java, "tailhook")
    private val rightFrontDrive: DcMotor = hardwareMap.get(DcMotor::class.java, "rightFrontDrive")
    private val leftFrontDrive: DcMotor = hardwareMap.get(DcMotor::class.java, "leftFrontDrive")
    private val rightRearDrive: DcMotor = hardwareMap.get(DcMotor::class.java, "rightRearDrive")
    private val leftRearDrive: DcMotor = hardwareMap.get(DcMotor::class.java, "leftRearDrive")
    val kennethClawRight: Servo = hardwareMap.get(Servo::class.java, "kennethClawRight")
    val kennethClawLeft: Servo = hardwareMap.get(Servo::class.java, "kennethClawLeft")
    private val kennethElevator: CRServo = hardwareMap.get(CRServo::class.java, "kennethElevator")
    val blinkyLights: RevBlinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver::class.java, "blinkyLights")
    var xPosition = startLocation.x
    var yPosition = startLocation.y
    var xTarget = 0.0
    var yTarget = 0.0
    val heading
        get() = inertialMotionUnit.getHeading().toDouble() + startHeading
    var priorHeading = 0.0
    private val mmPerInch = 25.4
    private val allMotors = listOf(rightFrontDrive, leftFrontDrive, rightRearDrive, leftRearDrive)
//    var desiredHeading = PI / 2.0
    private val driftAngleTolerance = PI / 18.0
    private val slideIncrement = 2.0
    private val slightSlideIncrement = 1.0
    private val timer = ElapsedTime()
    private var accel = 0.0
    private var xAccel = 0.0
    private var yAccel = 0.0
    var driveCommand = DriveCommand(0.0, 0.0, 0.0)

    fun bumpDrive(autonomousStep: AutonomousStep) {
        for (motor in allMotors) {
            motor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
            motor.mode = DcMotor.RunMode.RUN_USING_ENCODER
        }
        if (autonomousStep.length != 0.0) {
            val runDuration = autonomousStep.length / (autonomousStep.speed * INCHES_PER_SEC_PER_POWER_UNIT)
            val runTimer = ElapsedTime()
            accel = inertialMotionUnit.getAcceleration()
            xAccel = inertialMotionUnit.xAccel
            yAccel = inertialMotionUnit.yAccel
            driveCommand = DriveCommand(
                    xSpeed = autonomousStep.x * autonomousStep.speed / autonomousStep.length,
                    ySpeed = autonomousStep.y * autonomousStep.speed / autonomousStep.length,
                    rotationSpeed = 0.0)

            while (runTimer.seconds() < 0.5 || (runTimer.seconds() < runDuration && xAccel > -0.75)) {
                driveCommand.rotationSpeed = (autonomousStep.desiredHeading - robot.heading)
                setDriveMotion(driveCommand)
                updateAccelLights()
            }
        }
        setDriveMotion(STOP)
    }

    private fun updateAccelLights() {
        accel = inertialMotionUnit.getAcceleration()
        xAccel = inertialMotionUnit.xAccel
        yAccel = inertialMotionUnit.yAccel

        if (xAccel > 0.25) blinkyLights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED)
        if (xAccel < -0.25) blinkyLights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE)
        if (yAccel > 0.25) blinkyLights.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW)
        if (yAccel < -0.25) blinkyLights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN)
        Thread.sleep(50)
        blinkyLights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK)
    }

    fun rotateToHeading(radians: Double) {
        val targetHeading = radians
        while ((targetHeading - heading).absoluteValue > headingTolerance) {
            setDriveMotion(DriveCommand(
                    xSpeed = 0.0,
                    ySpeed = 0.0,
                    rotationSpeed = (targetHeading - heading) * ROTATION_SPEED_ADJUST))
        }
        setDriveMotion(STOP)
    }

    private fun slideOver(inches: Double) {
        blinkyLights.setPattern(RevBlinkinLedDriver.BlinkinPattern.WHITE)
        driveByEncoder(AutonomousStep(inches, 0.0, 0.5, "Slide over."))
        blinkyLights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK)
    }


    fun dragWithVisualGuidance(yGoalLine: Double) {

        var pullOutcome = tryStraightPull(yGoalLine)

        while (pullOutcome != PullOutcome.GOAL_LINE_REACHED) {
            when (pullOutcome) {
                PullOutcome.COURSE_CORRECTED -> tryReducedTrimPull(yGoalLine)
                PullOutcome.DRIFTED_CLOCKWISE -> tryTrimCounterClockwisePull(yGoalLine)
                PullOutcome.DRIFTED_COUNTERCLOCKWISE -> tryTrimClockwisePull(yGoalLine)
            }
        }

    }

    private fun tryStraightPull(yGoalLine: Double, desiredHeading: Double = PI/2.0): PullOutcome {
        var pullOutcome: PullOutcome? = null
        setDriveMotion(FORWARD)
        while (pullOutcome == null) {
            updateReckoning()
            pullOutcome = when {
                (yPosition - yGoalLine).absoluteValue < arrivalTolerance -> PullOutcome.GOAL_LINE_REACHED
                (heading - desiredHeading) > driftAngleTolerance -> PullOutcome.DRIFTED_COUNTERCLOCKWISE
                (desiredHeading - heading) > driftAngleTolerance -> PullOutcome.DRIFTED_CLOCKWISE
                else -> null
            }
        }
        setDriveMotion(STOP)
        return pullOutcome
    }

    private fun tryTrimClockwisePull(yGoalLine: Double,desiredHeading: Double = PI/2.0): PullOutcome {
        slideOver(slideIncrement)
        var pullOutcome: PullOutcome? = null
        val savedHeading = heading
        setDriveMotion(FORWARD)
        while (pullOutcome == null) {
            updateReckoning()
            pullOutcome = when {
                (yPosition - yGoalLine).absoluteValue < arrivalTolerance -> PullOutcome.GOAL_LINE_REACHED
                (savedHeading - heading) > driftAngleTolerance -> PullOutcome.DRIFTED_COUNTERCLOCKWISE
                (desiredHeading - heading).absoluteValue < driftAngleTolerance -> PullOutcome.COURSE_CORRECTED
                else -> null
            }
        }
        setDriveMotion(STOP)
        return pullOutcome
    }

    private fun tryTrimCounterClockwisePull(yGoalLine: Double,desiredHeading: Double = PI/2.0): PullOutcome {
        slideOver(-slideIncrement)
        var pullOutcome: PullOutcome? = null
        priorHeading = heading
        setDriveMotion(FORWARD)
        while (pullOutcome == null) {
            updateReckoning()
            pullOutcome = when {
                (yPosition - yGoalLine).absoluteValue < arrivalTolerance -> PullOutcome.GOAL_LINE_REACHED
                (heading - priorHeading) > driftAngleTolerance -> PullOutcome.DRIFTED_CLOCKWISE
                (desiredHeading - heading).absoluteValue < driftAngleTolerance -> PullOutcome.COURSE_CORRECTED
                else -> null
            }
        }
        setDriveMotion(STOP)
        return pullOutcome
    }

    private fun tryReducedTrimPull(yGoalLine: Double,desiredHeading: Double = PI/2.0): PullOutcome {
        if (priorHeading > desiredHeading) slideOver(slightSlideIncrement)
        else slideOver(-slightSlideIncrement)

        var pullOutcome: PullOutcome? = null
        setDriveMotion(FORWARD)
        while (pullOutcome == null) {
            updateReckoning()
            pullOutcome = when {
                (yPosition - yGoalLine).absoluteValue < arrivalTolerance -> PullOutcome.GOAL_LINE_REACHED
                (heading - desiredHeading) > driftAngleTolerance -> PullOutcome.DRIFTED_COUNTERCLOCKWISE
                (desiredHeading - heading) > driftAngleTolerance -> PullOutcome.DRIFTED_CLOCKWISE
                else -> null
            }
        }
        setDriveMotion(STOP)
        return pullOutcome
    }

    private val motorsBusy: Boolean
        get() {
            var checkMotor = false
            for (motor in allMotors) {
                if (motor.isBusy) checkMotor = true
            }
            return checkMotor
        }

    fun driveByEncoder(autonomousStep: AutonomousStep) {

        // ratio of motor speeds should be in proportion to the distance needed to travel
        // creating a diagonal motion and simultaneous arrival.
        // speeds should be scaled such that robot movement looks like
        // speed is equivalent to power setting 1.0 in the direction of movement.

        val x = autonomousStep.x * TICKS_PER_INCH
        val y = autonomousStep.y * TICKS_PER_INCH
        val speed = autonomousStep.speed
        val length = autonomousStep.length * TICKS_PER_INCH

        val rightRear = (x + y) * ONE_OVER_SQRT2
        val leftFront = -rightRear
        val leftRear = (x - y) * ONE_OVER_SQRT2
        val rightFront = -leftRear

        val timer = ElapsedTime()

        rightFrontDrive.targetPosition = rightFront.toInt()
        leftFrontDrive.targetPosition = leftFront.toInt()
        rightRearDrive.targetPosition = rightRear.toInt()
        leftRearDrive.targetPosition = leftRear.toInt()

        for (motor in allMotors) {
            motor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
            motor.mode = DcMotor.RunMode.RUN_TO_POSITION
        }

        timer.reset()

        while (motorsBusy) {
            rightFrontDrive.power = speed * MAX_POWER * rightFront / length
            leftFrontDrive.power = speed * MAX_POWER * leftFront / length
            rightRearDrive.power = speed * MAX_POWER * rightRear / length
            leftRearDrive.power = speed * MAX_POWER * leftRear / length

        }
        for (motor in allMotors) {
            motor.power = 0.0
        }
    }

    private fun setDriveMotion(command: DriveCommand) {
        driveCommand = command
        for (motor in allMotors) {
            motor.mode = DcMotor.RunMode.RUN_USING_ENCODER
        }
        val xSpeedScaled = driveCommand.xSpeed * ONE_OVER_SQRT2
        val ySpeedScaled = driveCommand.ySpeed * ONE_OVER_SQRT2

        rightFrontDrive.power = -xSpeedScaled + ySpeedScaled - driveCommand.rotationSpeed
        leftFrontDrive.power = -xSpeedScaled - ySpeedScaled - driveCommand.rotationSpeed
        rightRearDrive.power = xSpeedScaled + ySpeedScaled - driveCommand.rotationSpeed
        leftRearDrive.power = xSpeedScaled - ySpeedScaled - driveCommand.rotationSpeed

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

    private fun updateLocation() {
        val sightingWasSuccessful = false // updateSighting()
        if (!sightingWasSuccessful) {
            updateReckoning()
        }
    }

    fun fixTheHeading() {
        val clockwise = DriveCommand(0.0, 0.0, 0.5)
        val counterClockwise = DriveCommand(0.0, 0.0, -0.5)
        while (inertialMotionUnit.getHeading() !in PI / 2.0 - PI / 20.0..PI / 2.0 + PI / 20.0) {
            when (inertialMotionUnit.getHeading()) {
                in PI / 2.0 - PI / 20.0..-PI -> setDriveMotion(clockwise)
                in PI / 2.0 + PI / 20.0..PI -> setDriveMotion(counterClockwise)
            }
        }
    }


    private fun updateReckoning() {
        val elapsedTime = timer.seconds() - timeOfLastPositionPlot
        // for testing with drive only{
        moveOnFieldCommand = driveCommand
        //}
        val xTravel = moveOnFieldCommand.xSpeed * elapsedTime * INCHES_PER_SEC_PER_POWER_UNIT
        val yTravel = moveOnFieldCommand.ySpeed * elapsedTime * INCHES_PER_SEC_PER_POWER_UNIT
        xPosition += xTravel
        yPosition += yTravel
        timeOfLastPositionPlot = timer.seconds()
//        heading = inertialMotionUnit.getHeading().toDouble() + startHeading
    }

    private fun lesserOf(first: Int, second: Int): Double {
        if (first.absoluteValue < second.absoluteValue) return first.toDouble()
        else return -second.toDouble()

    }

    fun drive(autonomousStep: AutonomousStep) {
        for (motor in allMotors) {
            motor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
            motor.mode = DcMotor.RunMode.RUN_USING_ENCODER
        }
        if (autonomousStep.length != 0.0) {
            val runDuration = autonomousStep.length / (autonomousStep.speed * INCHES_PER_SEC_PER_POWER_UNIT)
            val runTimer = ElapsedTime()
            driveCommand = DriveCommand(
                    xSpeed = autonomousStep.x * autonomousStep.speed / autonomousStep.length,
                    ySpeed = autonomousStep.y * autonomousStep.speed / autonomousStep.length,
                    rotationSpeed = 0.0)

            while (runTimer.seconds() < runDuration) {
                driveCommand.rotationSpeed = (autonomousStep.desiredHeading - robot.heading)
                setDriveMotion(driveCommand)
                updateAccelLights()
            }
        }
        setDriveMotion(STOP)
    }

    private var rightFrontLastPosition = 0
    private var leftFrontLastPosition = 0
    private var rightRearLastPosition = 0
    private var leftRearLastPosition = 0
    private var timeOfLastPositionPlot = 0.0



    fun navigateTo(autonomousStep: AutonomousStep) {
        xTarget = autonomousStep.x
        yTarget = autonomousStep.y
        for (motor in allMotors) {
            motor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
            motor.mode = DcMotor.RunMode.RUN_USING_ENCODER
        }
        updateLocation()
        while (!atTarget) {
            val desiredHeading = PI / 2.0
            val rotation = heading - desiredHeading
            val bearing = atan2(yTarget - yPosition, xTarget - xPosition)
            moveOnFieldCommand = DriveCommand(0.0, autonomousStep.speed, -rotation).rotated(bearing)
            driveCommand = moveOnFieldCommand.rotated(-heading)
            setDriveMotion(driveCommand)
            updateLocation()
//            telemetry.addData("x", xPosition)
            telemetry.addData("yTarget", yTarget)
            telemetry.addData("y", yPosition)
//            telemetry.addData("Delta x", (xTarget - xPosition).absoluteValue)
            telemetry.addData("Delta y", (yTarget - yPosition).absoluteValue)
//            telemetry.addData("Heading",heading)
//            telemetry.addData("Bearing",bearing)
            telemetry.update()
        }
        setDriveMotion(STOP)

    }

    private val atTarget
        get() =
            (xTarget - xPosition).absoluteValue <= arrivalTolerance &&
                    (yTarget - yPosition).absoluteValue <= arrivalTolerance

}
