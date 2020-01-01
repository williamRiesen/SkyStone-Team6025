package org.firstinspires.ftc.teamcode


import com.qualcomm.hardware.rev.RevBlinkinLedDriver
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import kotlin.math.PI

@Autonomous(name = "Calibration Run ", group = "Holobot")
@Disabled

class CalibrationRun : LinearOpMode() {

    override fun runOpMode() {

        initialize(hardwareMap, telemetry, RevBlinkinLedDriver.BlinkinPattern.VIOLET)

        waitForStart()
        var slideSideways = useTileGrid(2.0,0.0)
        robot.bumpDrive(slideSideways)
        Thread.sleep(9000)
    }
}