package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode

lateinit var robot: TurtleDozerAutoBot3

@Autonomous(name = "Autonomous3", group = "Holobot")
class Autonomous3 : LinearOpMode() {
    override fun runOpMode() {

        robot = TurtleDozerAutoBot3(hardwareMap, telemetry)
        val startingXPosition = 24.0
        val startingYPosition = 60.0
        val driveFromStartToByFoundation = Vector(36.0, -24.0)
        robot.xPosition = startingXPosition
        robot.yPosition = startingYPosition
        val moveToViewNavTarget = Vector(24.0, -12.0, name = "Move to View NavTarget")
        val alignWithFoundation = Vector(48.0, 36.0, name = "Align with foundation.")
        val backUpToLatchFoundation = Vector(48.0, 24.0, name = "Back up to latch foundation.")
        val slideLeftToGoAroundFoundation = Vector(24.0, 60.0, name = "Slide Left to Go Around Foundation.")
        val backUpAlongsideFoundation = Vector(24.0, 36.0, name = "Back up Alongside Foundation")
        val pushFoundationToRight = Vector(36.0, 36.0, name = "Push Foundation to Right.")
        val parkUnderBridge = Vector(0.0, 36.0, name = "Park Under Bridge.")
        robot.showStatus("Ready!")
        waitForStart()



        robot.driveByEncoder(moveToViewNavTarget)
        robot.updateSighting()
        robot.visuallyNavigateTo(alignWithFoundation)
        robot.deployHook()
        Thread.sleep(1000)
        robot.visuallyNavigateTo(backUpToLatchFoundation)
//
//        robot.dragWithVisualCorrection(60.0, telemetry)
//        robot.unlatchHook()
//        robot.visuallyNavigateTo(slideLeftToGoAroundFoundation)
//        robot.visuallyNavigateTo(backUpAlongsideFoundation)
//        robot.visuallyNavigateTo(pushFoundationToRight)
//        robot.visuallyNavigateTo(parkUnderBridge)
//        robot.showStatus("Autonomous completed.")
        sleep(20000)

    }
}