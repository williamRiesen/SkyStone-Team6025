package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode

lateinit var robot: TurtleDozer3

@Autonomous(name = "Autonomous3", group = "Holobot")
class Autonomous3 : LinearOpMode(){
    override fun runOpMode() {

        robot = TurtleDozer3(hardwareMap, telemetry)
        val startingXPosition = 24.0
        val startingYPosition = 60.0
        val driveFromStartToByFoundation = Vector(36.0,-24.0)
        robot.xPosition = startingXPosition
        robot.yPosition = startingYPosition
        val moveToViewNavTarget = Vector(24.0,-12.0,name = "Move to View NavTarget")
        val alignWithFoundation = Vector(48.0,36.0,name = "Align with foundation.")
        val backUpToLatchFoundation = Vector(48.0,24.0, name = "Back up to latch foundation.")
        robot.showStatus("Ready!")
        waitForStart()

        robot.driveByEncoder(moveToViewNavTarget)
        robot.updateSighting()
        robot.visuallyNavigateTo(alignWithFoundation)
        robot.deployHook()
        robot.visuallyNavigateTo(backUpToLatchFoundation)

        robot.dragWithVisualCorrection(60.0,telemetry)
        sleep(20000)
    }
}