package org.firstinspires.ftc.teamcode


import com.qualcomm.hardware.rev.RevBlinkinLedDriver
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode

@Autonomous(name = "Basic Forward Autonomous", group = "Holobot")
//@Disabled

class BasicForwardAutonomous : LinearOpMode() {

    override fun runOpMode() {

        initialize(hardwareMap,telemetry, RevBlinkinLedDriver.BlinkinPattern.GREEN)

        waitForStart()
        val moveForward = Vector(0.0, 12.0 ,0.5,"Move Forward")
        robot.driveByEncoder(moveForward)
    }


}