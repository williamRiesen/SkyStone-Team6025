package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp


@TeleOp(name = "TeleOp3", group = "TurtleDozer.")
class TeleOp3 : OpMode() {
    lateinit var robot:TurtleDozer3

    override fun init() {
        robot = TurtleDozer3(hardwareMap,telemetry)
    }

    override fun loop() {
    }
}

