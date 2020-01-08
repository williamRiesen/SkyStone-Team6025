package org.firstinspires.ftc.teamcode


import com.qualcomm.hardware.rev.RevBlinkinLedDriver
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode

@Autonomous(name = "Blue Autonomous", group = "Holobot")
//@Disabled

class BlueAutonomous : LinearOpMode() {

    override fun runOpMode() {



        initialize(hardwareMap,telemetry, RevBlinkinLedDriver.BlinkinPattern.BLUE)
        checkList(telemetry,gamepad1,gamepad2, RevBlinkinLedDriver.BlinkinPattern.BLUE)

        waitForStart()

        go(telemetry)
    }


}