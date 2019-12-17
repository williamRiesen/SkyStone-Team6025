package org.firstinspires.ftc.teamcode


import com.qualcomm.hardware.rev.RevBlinkinLedDriver
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode

@Autonomous(name = "Calibration Run ", group = "Holobot")
//@Disabled

class CalibrationRun : LinearOpMode() {

    override fun runOpMode() {

        initialize(hardwareMap, telemetry, RevBlinkinLedDriver.BlinkinPattern.VIOLET)

        waitForStart()
        val startingLocation = Vector(0.0, 0.0)
        val moveForward60 = Vector(0.0, 60.0, 0.5, "Move Forward")
        robot.driveByEncoder(moveForward60)
        val endXByReckoning = robot.xPosition
        val endYByReckoning = robot.yPosition
        val sightingOK = robot.updateSighting()
        val endXByVisual = robot.xPosition
        val endYByVisual = robot.yPosition
        telemetry.addData("Sighting successful", sightingOK)
        telemetry.addData("By reckoning", "$endXByReckoning, $endYByReckoning")
        telemetry.addData("By visual", "$endXByVisual,$endYByVisual")
        telemetry.update()
    }
}