package org.firstinspires.ftc.teamcode


import com.qualcomm.hardware.rev.RevBlinkinLedDriver
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor

@Autonomous(name = "Basic Red Autonomous", group = "Holobot")
//@Disabled

class BasicRedAutonomous : LinearOpMode() {

    override fun runOpMode() {

        moveToViewNavTarget = Vector(-24.0, -12.0, name = "Move to View NavTarget")
        alignWithFoundation.reflectOverXAxis()
        backUpToLatchFoundation.reflectOverXAxis()
        slideLeftToGoAroundFoundation.reflectOverXAxis()
        backUpAlongsideFoundation.reflectOverXAxis()
        pushFoundationToRight.reflectOverXAxis()
        parkUnderBridge.reflectOverXAxis()
        foundationGoalLine = -foundationGoalLine

        initialize(hardwareMap,telemetry,RevBlinkinLedDriver.BlinkinPattern.RED)

        waitForStart()

        val moveOver = Vector(-12.0, 0.0,0.5,"Slide Over")
        robot.driveByEncoder(moveOver)
    }
}