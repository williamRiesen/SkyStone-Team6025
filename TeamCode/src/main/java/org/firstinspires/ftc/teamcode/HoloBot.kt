package org.firstinspires.ftc.teamcode

import com.qualcomm.hardware.rev.RevBlinkinLedDriver
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.HardwareMap

const val ONE_OVER_SQRT2 = 0.70710
const val TICKS_PER_DEGREE_OF_PIVOT = 4050 / 90
const val TICKS_PER_CM = 100


val FIRE = RevBlinkinLedDriver.BlinkinPattern.FIRE_LARGE
val STROBE_RED = RevBlinkinLedDriver.BlinkinPattern.STROBE_RED
val SCAN_RED = RevBlinkinLedDriver.BlinkinPattern.LARSON_SCANNER_RED
val CHASE_RED = RevBlinkinLedDriver.BlinkinPattern.LIGHT_CHASE_RED
val GREEN = RevBlinkinLedDriver.BlinkinPattern.GREEN
val BLUE = RevBlinkinLedDriver.BlinkinPattern.BLUE
val RED = RevBlinkinLedDriver.BlinkinPattern.RED
val STROBE_WHITE = RevBlinkinLedDriver.BlinkinPattern.STROBE_WHITE
val YELLOW = RevBlinkinLedDriver.BlinkinPattern.YELLOW


val FORWARD = DriveCommand(0.0F, 1.0F, 0.0F)
val BACKWARD = DriveCommand(0.0F, -1.0F, 0.0F)
val RIGHT = DriveCommand(1.0F, 0.0F, 0.0F)
val LEFT = DriveCommand(-1.0F, 0.0F, 0.0F)
val CLOCKWISE = DriveCommand(0.0F, 0.0F, 1.0F)
val COUNTERCLOCKWISE = DriveCommand(0.0F, 0.0F, -1.0F)

class HoloBot(hardwareMap: HardwareMap) {


    private val rightFrontDrive = hardwareMap.get(DcMotor::class.java, "rightFrontDrive")
    private val leftFrontDrive = hardwareMap.get(DcMotor::class.java, "leftFrontDrive")
    private val rightRearDrive = hardwareMap.get(DcMotor::class.java, "rightRearDrive")
    private val leftRearDrive = hardwareMap.get(DcMotor::class.java, "leftRearDrive")
    private val allMotors = listOf(rightFrontDrive, leftFrontDrive, rightRearDrive, leftRearDrive)

    private val blinkyLights = hardwareMap.get(RevBlinkinLedDriver::class.java, "blinkin")

//    val cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName())

    val visualLocalizer = VisualLocalizer(hardwareMap)
    val motionSensor = MotionSensor(hardwareMap)


    fun setLights(value: RevBlinkinLedDriver.BlinkinPattern) {
        blinkyLights.setPattern(value)
    }

    fun stopAllMotors() {
        for (motor in allMotors) {
            motor.power = 0.0
        }
    }

    fun setDriveMotion(command: DriveCommand) {
        val xSpeedScaled = command.xSpeed * ONE_OVER_SQRT2
        val ySpeedScaled = command.ySpeed * ONE_OVER_SQRT2
        val rightFront = xSpeedScaled + ySpeedScaled + command.rotationSpeed
        val leftFront = -xSpeedScaled + ySpeedScaled + command.rotationSpeed
        val rightRear = xSpeedScaled - ySpeedScaled + command.rotationSpeed
        val leftRear = -xSpeedScaled - ySpeedScaled + command.rotationSpeed
        rightFront.coerceIn(-1.0, 1.0)
        leftFront.coerceIn(-1.0, 1.0)
        rightRear.coerceIn(-1.0, 1.0)
        leftRear.coerceIn(-1.0, 1.0)
        rightFrontDrive.power = rightFront
        leftFrontDrive.power = leftFront
        rightRearDrive.power = rightRear
        leftRearDrive.power = leftRear
    }

    fun driveByEncoder(xCM: Float, yCM: Float) {
        for (motor in allMotors) {
            motor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
            motor.mode = DcMotor.RunMode.RUN_TO_POSITION
        }
        val x = xCM * TICKS_PER_CM
        val y = yCM * TICKS_PER_CM
        val rightFront = (x + y) * ONE_OVER_SQRT2
        val leftFront = (-x + y) * ONE_OVER_SQRT2
        val rightRear = (x - y) * ONE_OVER_SQRT2
        val leftRear = (-x - y) * ONE_OVER_SQRT2

        rightFrontDrive.targetPosition = rightFront.toInt()
        leftFrontDrive.targetPosition = leftFront.toInt()
        rightRearDrive.targetPosition = rightRear.toInt()
        leftRearDrive.targetPosition = leftRear.toInt()

        for (motor in allMotors) motor.power = 1.0
    }

    fun pivotByEncoder(degrees: Float) {
        val target = degrees * TICKS_PER_DEGREE_OF_PIVOT
        for (motor in allMotors) {
            motor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
            motor.mode = DcMotor.RunMode.RUN_TO_POSITION
            motor.targetPosition = target.toInt()
            motor.power = 1.0
        }
    }
}