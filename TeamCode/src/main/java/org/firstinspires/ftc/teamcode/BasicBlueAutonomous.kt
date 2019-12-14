package org.firstinspires.ftc.teamcode


import com.qualcomm.hardware.rev.RevBlinkinLedDriver
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode

@Autonomous(name = "Basic Blue Autonomous", group = "Holobot")
//@Disabled

class BasicBlueAutonomous : LinearOpMode() {

    override fun runOpMode() {

        initialize(hardwareMap,telemetry, RevBlinkinLedDriver.BlinkinPattern.BLUE)

        waitForStart()
        val moveOver = Vector(12.0, 0.0,0.5,"Slide Over")
        robot.driveByEncoder(moveOver)
    }


}