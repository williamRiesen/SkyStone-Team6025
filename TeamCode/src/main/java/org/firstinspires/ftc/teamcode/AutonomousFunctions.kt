package org.firstinspires.ftc.teamcode

import com.qualcomm.hardware.rev.RevBlinkinLedDriver
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.robotcore.external.Telemetry
import com.qualcomm.robotcore.eventloop.opmode.TeleOp

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.Gamepad
import kotlin.math.PI


lateinit var robot: TurtleDozerAutoBot3
val startLocation = useTileGrid(0.0, 0.0)
var startHeading = PI / 2
//var moveToViewNavTarget = useTileGrid(0.5, -1.0)
val alignWithFoundation = useTileGrid(0.5, -1.0)
val bumpAudienceWall = useTileGrid(2.0, 0.0)
val reboundToAlignWithFoundation = AutonomousStep(-16.0, 0.0, 0.5)
val backUpToLatchFoundation = useTileGrid(0.0, -0.35, 0.1)
val pullForwardToLatch = useTileGrid(0.0, 0.35, 0.1)
val dragFoundation = useTileGrid(0.0, 1.5, 0.3)
val slideLeftToGoAroundFoundation = useTileGrid(1.0, 2.5)
val shiftTowardAudience = useTileGrid(-1.25, 0.0)
val backUpToGoAroundFoundation = useTileGrid(1.9,0.0 ,0.75,"",PI)
val advanceAwayFromAudience = useTileGrid(0.0, 1.3,0.75,"", PI)
val pushFoundationHome = useTileGrid(-2.0, 0.0, 0.5,"", PI)
val backUpAlongsideFoundation = useTileGrid(1.0, 2.0)
val pushFoundationToRight = useTileGrid(1.5, 2.0)
val parkUnderBridge = useTileGrid(0.0, -2.0,0.75,"", PI)
var foundationGoalLine = 60.0
val pointA = useTileGrid(1.5, 2.6)

fun initialize(hardwareMap: HardwareMap, telemetry: Telemetry, lightPattern: RevBlinkinLedDriver.BlinkinPattern) {
    robot = TurtleDozerAutoBot3(hardwareMap, telemetry)
    with(robot) {
        xPosition = startLocation.x
        yPosition = startLocation.y
        blinkyLights.setPattern(lightPattern)
        showStatus("Ready!")
        kennethClawLeft.direction = Servo.Direction.REVERSE
    }
}

fun checkList(telemetry: Telemetry, gamepad1: Gamepad, gamepad2: Gamepad, lightPattern: RevBlinkinLedDriver.BlinkinPattern) {
    robot.blinkyLights.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP2_HEARTBEAT_SLOW)
    telemetry.addData("WAIT!", "Is tail hook servo vertical?")
    telemetry.addData("Press yellow if yes", "")
    telemetry.update()
    while (!gamepad1.y) {
        Thread.sleep(10)
    }
    telemetry.addData("WAIT!", "Are eyes of robot facing you?")
    telemetry.addData("Press green if yes", "")
    telemetry.update()
    while (!gamepad1.a) {
        Thread.sleep(10)
    }
    robot.blinkyLights.setPattern(lightPattern)
    telemetry.addData("Status", "READY!")
    telemetry.update()
}

fun useTileGrid(
        xTile: Double,
        yTile: Double,
        speed: Double = 0.75,
        name: String = "Unnamed Instruction Step",
        desiredHeading: Double = PI / 2.0): AutonomousStep {
    return AutonomousStep(xTile * 24.0, yTile * 24.0, speed, name, desiredHeading)
}

fun go(telemetry: Telemetry) {
    with(robot) {
        //        desiredHeading = PI / 2.0
        blinkyLights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK)
        drive(alignWithFoundation)
        bumpDrive(bumpAudienceWall)
        drive(reboundToAlignWithFoundation)
        deployHook()
        drive(backUpToLatchFoundation)
        drive(dragFoundation)
        unlatchHook()
        drive(shiftTowardAudience)
        drive(backUpToGoAroundFoundation)
        drive(advanceAwayFromAudience)
        drive(pushFoundationHome)
        drive(parkUnderBridge)
        kennethClawLeft.position = clawRestPosition
        kennethClawRight.position = clawRestPosition
        blinkyLights.setPattern(RevBlinkinLedDriver.BlinkinPattern.TWINKLES_PARTY_PALETTE)
    }
    Thread.sleep(1000)
}
