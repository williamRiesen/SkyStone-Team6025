package org.firstinspires.ftc.teamcode


import android.content.Context
import com.qualcomm.ftccommon.SoundPlayer
import com.qualcomm.hardware.rev.RevBlinkinLedDriver
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.Telemetry
import kotlin.math.*

const val ONE_OVER_SQRT2 = 0.70710
const val TICKS_PER_DEGREE_OF_PIVOT = 1200 / 90
const val TICKS_PER_INCH = 128
const val POWER_LIMIT = 0.5
const val RAMP_UP_MS = 500.0

val FIRE = RevBlinkinLedDriver.BlinkinPattern.FIRE_LARGE
val STROBE_RED = RevBlinkinLedDriver.BlinkinPattern.STROBE_RED
val SCAN_RED = RevBlinkinLedDriver.BlinkinPattern.LARSON_SCANNER_RED
val CHASE_RED = RevBlinkinLedDriver.BlinkinPattern.LIGHT_CHASE_RED
val GREEN = RevBlinkinLedDriver.BlinkinPattern.GREEN
val BLUE = RevBlinkinLedDriver.BlinkinPattern.BLUE
val RED = RevBlinkinLedDriver.BlinkinPattern.RED
val STROBE_WHITE = RevBlinkinLedDriver.BlinkinPattern.STROBE_WHITE
val YELLOW = RevBlinkinLedDriver.BlinkinPattern.YELLOW

val FORWARD = DriveCommand(0.0, 1.0, 0.0)
val SLOW_FORWARD = DriveCommand(0.25, 0.0, 0.0)
val BACKWARD = DriveCommand(0.0, -1.0, 0.0)
val RIGHT = DriveCommand(1.0, 0.0, 0.0)
val LEFT = DriveCommand(-1.0, 0.0, 0.0)
val CLOCKWISE = DriveCommand(0.0, 0.0, 1.0)
val COUNTERCLOCKWISE = DriveCommand(0.0, 0.0, -1.0)
val STOP = DriveCommand(0.0, 0.0, 0.0)

//class TurtleDozer(hardwareMap: HardwareMap) {
class TurtleDozer(
        private val myApp: Context,
        private val params: SoundPlayer.PlaySoundParams,
        private val rightFrontDrive: DcMotor,
        private val leftFrontDrive: DcMotor,
        private val rightRearDrive: DcMotor,
        private val leftRearDrive: DcMotor,
        val tailhook: Servo,
        val dozerbladeRight: Servo,
        val dozerbladeLeft: Servo,
//        val cameraMonitorViewId: Int,
//        val visualNavigator: VisualNavigator,
        val motionSensor: MotionSensor
) {
    val allMotors = listOf(rightFrontDrive, leftFrontDrive, rightRearDrive, leftRearDrive)


    var cameraMonitorViewId: Int? = null
    var visualNavigator: VisualNavigator? = null

    companion object {
        fun build(hardwareMap: HardwareMap): TurtleDozer {


            val params = SoundPlayer.PlaySoundParams()

            try {
                val rightFrontDrive = hardwareMap.get(DcMotor::class.java, "rightFrontDrive")
                val leftFrontDrive: DcMotor = hardwareMap.get(DcMotor::class.java, "leftFrontDrive")

                val rightRearDrive: DcMotor = hardwareMap.get(DcMotor::class.java, "rightRearDrive")
                val leftRearDrive: DcMotor = hardwareMap.get(DcMotor::class.java, "leftRearDrive")

                val tailhook: Servo = hardwareMap.get(Servo::class.java, "tailhook")
                val dozerbladeRight: Servo = hardwareMap.get(Servo::class.java, "dozerbladeRight")
                val dozerbladeLeft: Servo = hardwareMap.get(Servo::class.java, "dozerbladeLeft")

                val motionSensor = MotionSensor(hardwareMap)

                val myApp = hardwareMap.appContext

                throw IllegalArgumentException()
                return TurtleDozer(
                        myApp,
                        params,
                        rightFrontDrive,
                        leftFrontDrive,
                        rightRearDrive,
                        leftRearDrive,
                        tailhook,
                        dozerbladeRight,
                        dozerbladeLeft,
//                        cameraMonitorViewId,
//                        visualNavigator,
                        motionSensor
                )
            } catch (err: IllegalArgumentException) {
                val rightFrontDrive = SimulatedDcMotor()
                val leftFrontDrive = SimulatedDcMotor()

                val rightRearDrive = SimulatedDcMotor()
                val leftRearDrive = SimulatedDcMotor()

                val tailhook: Servo = SimulatedServo()
                val dozerbladeRight = SimulatedServo()
                val dozerbladeLeft = SimulatedServo()

                val motionSensor = MotionSensor(hardwareMap)

                val myApp = hardwareMap.appContext
                return TurtleDozer(
                        myApp,
                        params,
                        rightFrontDrive,
                        leftFrontDrive,
                        rightRearDrive,
                        leftRearDrive,
                        tailhook,
                        dozerbladeRight,
                        dozerbladeLeft,
//                        cameraMonitorViewId,
//                        visualNavigator,
                        motionSensor
                )
            }
        }
    }

    init {
        params.loopControl = 0
        params.waitForNonLoopingSoundsToFinish = true
        rightFrontDrive.targetPosition = 0
        leftFrontDrive.targetPosition = 0
        rightRearDrive.targetPosition = 0
        leftRearDrive.targetPosition = 0
    }

    fun installVisualNavigator(hardwareMap: HardwareMap) {
        cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName())
        visualNavigator = VisualNavigator(hardwareMap)
    }

    private val motorsBusy: Boolean
        get() {
            var checkMotor = false
            for (motor in allMotors) {
                if (motor.isBusy) checkMotor = true
            }
            return checkMotor
        }

    fun setLights(value: RevBlinkinLedDriver.BlinkinPattern) {
//        blinkyLights.setPattern(value)
    }

    fun showStatus(string: String) {
//        telemetry.addData("Status",string)
//        telemetry.update()
    }

    fun stopAllMotors() {
        for (motor in allMotors) {
            motor.power = 0.0
        }
    }

    fun setDriveMotion(command: DriveCommand) {
        for (motor in allMotors) motor.mode = DcMotor.RunMode.RUN_USING_ENCODER
        val xSpeedScaled = command.xSpeed * ONE_OVER_SQRT2
        val ySpeedScaled = command.ySpeed * ONE_OVER_SQRT2
        rightFrontDrive.power = -xSpeedScaled + ySpeedScaled - command.rotationSpeed
        leftFrontDrive.power = -xSpeedScaled - ySpeedScaled - command.rotationSpeed
        rightRearDrive.power = xSpeedScaled + ySpeedScaled - command.rotationSpeed
        leftRearDrive.power = xSpeedScaled - ySpeedScaled - command.rotationSpeed
    }

    fun driveByGyro(vector: Vector, telmetry: Telemetry) {
        val x = (vector.x * TICKS_PER_INCH)
        val y = (vector.y * TICKS_PER_INCH)
        val speed = vector.speed
        val rightFront = ((-x + y) * ONE_OVER_SQRT2).toInt()
        val leftFront = ((-x - y) * ONE_OVER_SQRT2).toInt()
        val rightRear = ((x + y) * ONE_OVER_SQRT2).toInt()
        val leftRear = ((x - y) * ONE_OVER_SQRT2).toInt()
        val xSpeed = speed * x / (x + y).toDouble()
        val ySpeed = speed * y / (x + y).toDouble()
        val desiredHeading = 0.0
        val correctionFactor = 10.0
        val driveCommand = DriveCommand(xSpeed, ySpeed, 0.0)

        for (motor in allMotors) {
            motor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
            motor.mode = DcMotor.RunMode.RUN_USING_ENCODER
        }

        while (rightFrontDrive.currentPosition.absoluteValue < rightFront.absoluteValue &&
                leftFrontDrive.currentPosition.absoluteValue < leftFront.absoluteValue &&
                rightRearDrive.currentPosition.absoluteValue < rightRear.absoluteValue &&
                leftRearDrive.currentPosition.absoluteValue < leftRear.absoluteValue) {

            val heading = motionSensor.getHeading()
            val correctionAngle = (heading - desiredHeading) * correctionFactor
            telmetry.addData("Heading", heading)
            telmetry.addData("Correction Angle", correctionAngle)
            telmetry.update()
            val correctedDriveCommand = driveCommand.rotated(correctionAngle)
            setDriveMotion(correctedDriveCommand)
        }

        stopAllMotors()
    }

    fun driveByEncoder(vector: Vector) {
        showStatus("Driving by Encoder to $vector")
        val x = vector.x * TICKS_PER_INCH
        val y = vector.y * TICKS_PER_INCH
        val rightFront = (-x + y) * ONE_OVER_SQRT2
        val leftFront = (-x - y) * ONE_OVER_SQRT2
        val rightRear = (x + y) * ONE_OVER_SQRT2
        val leftRear = (x - y) * ONE_OVER_SQRT2
        val timer = ElapsedTime()

        for (motor in allMotors) {
            motor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
            motor.mode = DcMotor.RunMode.RUN_TO_POSITION
        }

        rightFrontDrive.targetPosition = rightFront.toInt()
        leftFrontDrive.targetPosition = leftFront.toInt()
        rightRearDrive.targetPosition = rightRear.toInt()
        leftRearDrive.targetPosition = leftRear.toInt()

        timer.reset()

        while (motorsBusy) {
            val momentaryPower =
                    if (timer.milliseconds() < RAMP_UP_MS) timer.milliseconds() / RAMP_UP_MS * vector.speed
                    else vector.speed
            for (motor in allMotors) motor.power = momentaryPower
        }
        for (motor in allMotors) motor.power = 0.0
        showStatus("Drive by encodor target reached.")
    }

    fun rotate(degrees: Int) {
        val target = degrees * TICKS_PER_DEGREE_OF_PIVOT
        for (motor in allMotors) {
            motor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
            motor.mode = DcMotor.RunMode.RUN_TO_POSITION
            motor.targetPosition = target
            motor.power = POWER_LIMIT
        }
        while (motorsBusy) {
            Thread.sleep(50)
        }
    }

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

    fun deployHook() {
        tailhook.position = 0.0
        showStatus("Hook delployed")
    }

    fun unlatchHook() {
        tailhook.position = 0.5
        showStatus("Hook unlatched")
    }

    private fun slideOver(inches: Double) {
        driveByEncoder(Vector(inches, 0.0))
    }

    fun dragWithBalance(telemetry: Telemetry) {

        val timer = ElapsedTime()
        timer.reset()
        val targetHeading = PI.toFloat()
        var status = "unknown"
        while (timer.seconds() < 20) {
            val startHeading = motionSensor.getHeading()
            setDriveMotion(DriveCommand(0.0, 0.2, 0.0))
            while ((motionSensor.getHeading() - startHeading).absoluteValue < PI / 90.0 ||
                    (status == "improving" &&
                            (motionSensor.getHeading() - targetHeading).absoluteValue < PI / 90.0))
                setDriveMotion(STOP)


            status = if (motionSensor.getHeading() in startHeading..targetHeading ||
                    motionSensor.getHeading() in targetHeading..startHeading) {
                "improving"
            } else {
                "worsening"
            }

            telemetry.addData("Status", status)
            telemetry.addData("RunLength", rightRearDrive.currentPosition)
            telemetry.update()
            val increment = 2.0 * 200 / rightRearDrive.currentPosition
            if (motionSensor.getHeading() - startHeading < 0.0) {
                slideOver(-increment)
            } else {
                slideOver(increment)
            }

        }
    }
}

