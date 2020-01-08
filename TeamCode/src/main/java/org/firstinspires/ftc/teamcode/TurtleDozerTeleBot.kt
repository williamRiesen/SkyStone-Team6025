package org.firstinspires.ftc.teamcode

import com.qualcomm.hardware.bosch.BNO055IMU
import com.qualcomm.hardware.bosch.BNO055IMU.Parameters
import com.qualcomm.hardware.rev.RevBlinkinLedDriver
import com.qualcomm.robotcore.hardware.*
import org.firstinspires.ftc.robotcore.external.Telemetry


class TurtleDozerTeleBot(hardwareMap: HardwareMap, val telemetry: Telemetry) {
    val tailHook: Servo? = hardwareMap.get(Servo::class.java, "tailhook")
    val blinkyLights: RevBlinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver::class.java, "blinkyLights")
    val kennethClawRight: Servo = hardwareMap.get(Servo::class.java, "kennethClawRight")
    val kennethClawLeft: Servo = hardwareMap.get(Servo::class.java, "kennethClawLeft")

     val kennethElevator: CRServo = hardwareMap.get(CRServo::class.java, "kennethElevator")
    private val rightFrontDrive: DcMotor? = hardwareMap.get(DcMotor::class.java, "rightFrontDrive")
    private val leftFrontDrive: DcMotor? = hardwareMap.get(DcMotor::class.java, "leftFrontDrive")
    private val rightRearDrive: DcMotor? = hardwareMap.get(DcMotor::class.java, "rightRearDrive")
    private val leftRearDrive: DcMotor? = hardwareMap.get(DcMotor::class.java, "leftRearDrive")
    private val elevatorLowerLimitSwitch: DigitalChannel = hardwareMap.get<com.qualcomm.robotcore.hardware.DigitalChannel
            >(com.qualcomm.robotcore.hardware.DigitalChannel::class.java,  "sensor_digital")

    val inertialMotionUnit: InertialMotionUnit = InertialMotionUnit(hardwareMap)
    var parameters: Parameters = Parameters()

    init {
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"
        parameters.loggingEnabled = true
        parameters.loggingTag = "IMU"
        parameters.accelerationIntegrationAlgorithm = com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator()
        kennethClawLeft.direction = Servo.Direction.REVERSE
        kennethElevator.direction = DcMotorSimple.Direction.REVERSE
        elevatorLowerLimitSwitch.setMode(DigitalChannel.Mode.INPUT)
    }

    var heading = 0.0
    val allMotors = listOf(rightFrontDrive, leftFrontDrive, rightRearDrive, leftRearDrive)

    val elevatorIsAtBottomLimit
            get() = !elevatorLowerLimitSwitch.state


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
    }

    fun deployHook() {
        if (tailHook != null) {
            tailHook.position = 0.0
        }
    }

    fun unlatchHook() {
        if (tailHook != null) {
            tailHook.position = 0.5
        }
    }

    fun stopAllMotors() {
        for (motor in allMotors) {
            motor!!.power = 0.0
        }
    }

}
