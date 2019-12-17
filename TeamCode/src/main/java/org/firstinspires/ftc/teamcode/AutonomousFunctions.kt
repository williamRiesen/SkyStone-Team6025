package org.firstinspires.ftc.teamcode

import com.qualcomm.hardware.rev.RevBlinkinLedDriver
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry
import kotlin.math.PI


lateinit var robot: TurtleDozerAutoBot3
val startLocation = useTileGrid(0.0, 0.0)
var startHeading = PI / 2
var moveToViewNavTarget = useTileGrid(0.0,1.0)
val alignWithFoundation = useTileGrid(1.75,2.00)
val backUpToLatchFoundation = useTileGrid(1.25,1.5,0.3)
//1.75 - 1.25
val dragFoundation = useTileGrid(1.75,3.0)
val slideLeftToGoAroundFoundation = useTileGrid(1.0,2.5)
val backUpAlongsideFoundation = useTileGrid(1.0,2.0)
val pushFoundationToRight = useTileGrid(1.5,2.0)
val parkUnderBridge = Vector(0.0, 2.0)
var foundationGoalLine = 60.0

fun initialize(hardwareMap: HardwareMap, telemetry: Telemetry, lightPattern: RevBlinkinLedDriver.BlinkinPattern) {
    telemetry.addData("Status", "NOT READY! WAIT FOR SOLID LIGHT . . . ")
    telemetry.update()
    robot = TurtleDozerAutoBot3(hardwareMap, telemetry)
    robot.xPosition = startLocation.x
    robot.yPosition = startLocation.y
    robot.heading = robot.inertialMotionUnit.getHeading() + startHeading
    robot.blinkyLights.setPattern(lightPattern)
    robot.showStatus("Ready!")
}

fun useTileGrid(xTile: Double, yTile: Double, speed: Double = 0.75): Vector {
    return Vector(xTile * 24.0, yTile * 24.0, speed)
}

fun go(telemetry: Telemetry) {

    robot.blinkyLights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK)
    robot.navigateTo(moveToViewNavTarget)
//    robot.dozerBladePosition = 0.3
//    Thread.sleep(1000)
//    robot.navigateTo(alignWithFoundation)
//    robot.fixTheHeading()
//    robot.deployHook()
//    Thread.sleep(500)
//    robot.navigateTo(backUpToLatchFoundation)
//    robot.dragWithVisualGuidance(foundationGoalLine)
//    robot.unlatchHook()
//    Thread.sleep(500)
//    robot.navigateTo(slideLeftToGoAroundFoundation)
//    robot.navigateTo(backUpAlongsideFoundation)
//    robot.navigateTo(pushFoundationToRight)
//    robot.navigateTo(parkUnderBridge)
    robot.blinkyLights.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_WHITE)
Thread.sleep(9000)
}
