package org.firstinspires.ftc.teamcode

import com.qualcomm.hardware.rev.RevBlinkinLedDriver
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.util.ElapsedTime


@TeleOp(name = "TeleOp", group = "TurtleDozer.")
class TeleOp : OpMode() {
    lateinit var robot: TurtleDozerTeleBot
    private val dozerBladeNeutralPosition = 0.3
    private val dozerBladeLowerLimit = 0.125
    private val dozerBladeUpperLimit = 0.6
    private val timer = ElapsedTime()
    private var allianceIsBlue = false

    override fun init() {
        robot = TurtleDozerTeleBot(hardwareMap, telemetry)
//        robot.dozerBladePosition = dozerBladeNeutralPosition
        robot.blinkyLights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN)
        timer.reset()
    }

    override fun init_loop() {
        if (gamepad2.b) {
            robot.blinkyLights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED)
            allianceIsBlue = false
        }
        if (gamepad2.x) {
            robot.blinkyLights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE)
            allianceIsBlue = true
        }
        if (gamepad2.y) {
            robot.blinkyLights.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW)
            allianceIsBlue = false
        }
        if (gamepad2.a) {
            robot.blinkyLights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN)
            allianceIsBlue = false
        }
    }

    override fun loop() {

        if (gamepad2.dpad_down) {
            robot.deployHook()
        }

        if (gamepad2.dpad_up) {
            robot.unlatchHook()
        }
        if (gamepad2.b) {
            robot.blinkyLights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED)
            allianceIsBlue = false
        }
        if (gamepad2.x) {
            robot.blinkyLights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE)
            allianceIsBlue = true
        }
        if (gamepad2.y) {
            robot.blinkyLights.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW)
            allianceIsBlue = false
        }
        if (gamepad2.a) {
            robot.blinkyLights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN)
            allianceIsBlue = false
        }
        if (timer.seconds() > 90) {
            if (allianceIsBlue) robot.blinkyLights.setPattern(RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_BLUE)
            else robot.blinkyLights.setPattern(RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_RED)
        }

//        val joyStickInput = -gamepad2.right_stick_y.toDouble()
//        robot.dozerBladePosition = when (joyStickInput) {
//
//            0.0 -> dozerBladeNeutralPosition
//            in -1.0..0.0 -> dozerBladeNeutralPosition + joyStickInput * (dozerBladeNeutralPosition - dozerBladeLowerLimit)
//            in 0.0..1.0 -> dozerBladeNeutralPosition + joyStickInput * (dozerBladeUpperLimit - dozerBladeNeutralPosition)
//            else -> dozerBladeNeutralPosition
//
//        }
        robot.kennethElevator.power = gamepad2.left_stick_y.toDouble()

        val clawReductionFactor = 0.5
        val clawRestPosition = 0.35
        val clawPosition = clawRestPosition + gamepad2.right_trigger.toDouble() * clawReductionFactor
        robot.kennethClawRight.position = clawPosition
        robot.kennethClawLeft.position = clawPosition




        telemetry.addData("Joystick", gamepad2.right_trigger)
        telemetry.update()

        val rotation = gamepad1.left_stick_x.toDouble()
        val xScooch = gamepad1.right_stick_x.toDouble()
        val yScooch = -gamepad1.right_stick_y.toDouble()
        val heading = robot.inertialMotionUnit.getHeading()

        val driveCommand = DriveCommand(xScooch, yScooch, rotation * ROTATION_SPEED_ADJUST)
        driveCommand.rotate(heading + 3.1415f)
        robot.setDriveMotion(driveCommand)
    }
}

