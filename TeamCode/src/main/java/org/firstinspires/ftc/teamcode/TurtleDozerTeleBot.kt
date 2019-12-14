package org.firstinspires.ftc.teamcode

import com.qualcomm.hardware.bosch.BNO055IMU
import com.qualcomm.hardware.bosch.BNO055IMU.Parameters
import com.qualcomm.hardware.rev.RevBlinkinLedDriver
import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.robotcore.external.Telemetry
import kotlin.math.PI


class TurtleDozerTeleBot(hardwareMap: HardwareMap, val telemetry: Telemetry) {
    val tailHook: Servo? = hardwareMap.get(Servo::class.java, "tailhook")
    val blinkyLights: RevBlinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver::class.java, "blinkyLights")
    private val dozerBladeRight: Servo? = hardwareMap.get(Servo::class.java, "dozerbladeRight")
    private val dozerBladeLeft: Servo? = hardwareMap.get(Servo::class.java, "dozerbladeLeft")
    val kenneth: CRServo = hardwareMap.get(CRServo::class.java, "kenneth")
    private val rightFrontDrive: DcMotor? = hardwareMap.get(DcMotor::class.java, "rightFrontDrive")
    private val leftFrontDrive: DcMotor? = hardwareMap.get(DcMotor::class.java, "leftFrontDrive")
    private val rightRearDrive: DcMotor? = hardwareMap.get(DcMotor::class.java, "rightRearDrive")
    private val leftRearDrive: DcMotor? = hardwareMap.get(DcMotor::class.java, "leftRearDrive")
//    val inertialMotionUnit: BNO055IMU = hardwareMap.get(BNO055IMU::class.java, "imu")
val inertialMotionUnit: InertialMotionUnit = InertialMotionUnit(hardwareMap)
    var parameters: Parameters = BNO055IMU.Parameters()
    init {
        parameters.angleUnit = com.qualcomm.hardware.bosch.BNO055IMU.AngleUnit.DEGREES
        parameters.accelUnit = com.qualcomm.hardware.bosch.BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"
        parameters.loggingEnabled = true
        parameters.loggingTag = "IMU"
        parameters.accelerationIntegrationAlgorithm = com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator()
//        if (inertialMotionUnit != null) {
//            inertialMotionUnit.initialize(parameters)
//        }
    }
    var heading = 0.0
    val allMotors = listOf(rightFrontDrive, leftFrontDrive, rightRearDrive, leftRearDrive)


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
        telemetry.addData("Heading", heading * 180 / PI)
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
    fun stopAllMotors() {
        for (motor in allMotors) {
            motor!!.power = 0.0
        }
    }

    var dozerBladePosition: Double
        get() = dozerBladeRight?.position ?: 0.0
        set(value) {
            if (dozerBladeRight != null) {
                dozerBladeRight.position = value
            }
            if (dozerBladeLeft != null) {
                dozerBladeLeft.position = 1.0 - value
            }
        }
}
