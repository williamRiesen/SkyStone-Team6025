package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp


@TeleOp(name = "TeleOp3", group = "TurtleDozer.")
class TeleOp3 : OpMode() {
    lateinit var robot: TurtleDozerTeleBot3
    val dozerBladeNeutralPosition = 0.4
    val dozerBladeLowerLimit = 0.0
    val dozerBladeUpperLimit = 0.7

    override fun init() {
        robot = TurtleDozerTeleBot3(hardwareMap, telemetry)
        robot.dozerBladePosition = dozerBladeNeutralPosition
    }

    override fun loop() {

        if (gamepad2.dpad_down) {
            robot.deployHook()
        }

        if (gamepad2.dpad_up) {
            robot.unlatchHook()
        }


        val joyStickInput = -gamepad2.right_stick_y.toDouble()
        robot.dozerBladePosition = when (joyStickInput) {
            0.0 -> dozerBladeNeutralPosition
            in -1.0..0.0 -> dozerBladeNeutralPosition + joyStickInput * (dozerBladeNeutralPosition - dozerBladeLowerLimit)
            in 0.0..1.0 -> dozerBladeNeutralPosition + joyStickInput * (dozerBladeUpperLimit - dozerBladeNeutralPosition)
            else -> dozerBladeNeutralPosition
        }


        val rotation = gamepad1.left_stick_x.toDouble()
        val xScooch = gamepad1.right_stick_x.toDouble()
        val yScooch = -gamepad1.right_stick_y.toDouble()
        val heading = robot.inertialMotionUnit.getHeading()

//        telemetry.addData("Heading", heading)
        telemetry.addData("Rotation", rotation)
        telemetry.update()

        val driveCommand = DriveCommand(xScooch, yScooch, rotation * ROTATION_SPEED_ADJUST)
        driveCommand.rotate(heading + 3.1415f)
        robot.setDriveMotion(driveCommand)

    }
}

