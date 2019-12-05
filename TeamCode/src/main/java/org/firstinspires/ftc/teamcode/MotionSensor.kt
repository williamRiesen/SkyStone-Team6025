package org.firstinspires.ftc.teamcode

import com.qualcomm.hardware.bosch.BNO055IMU
import com.qualcomm.hardware.bosch.BNO055IMU.AccelUnit
import com.qualcomm.hardware.bosch.BNO055IMU.Parameters
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.navigation.*


class MotionSensor(hwMap: HardwareMap)  {
    var imu: BNO055IMU
    val parameters = Parameters()

    init {
        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.

        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS
        parameters.accelUnit = AccelUnit.METERS_PERSEC_PERSEC
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"
        parameters.loggingEnabled = true
        parameters.loggingTag = "IMU"
        parameters.accelerationIntegrationAlgorithm = JustLoggingAccelerationIntegrator()
        imu = hwMap.get(BNO055IMU::class.java, "imu")
        imu.initialize(parameters)
        imu.startAccelerationIntegration(Position(), Velocity(), 1000)
    }

    fun getHeading(): Float {
        val angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS)
        return - angles.firstAngle
    }

    fun resetHeading(){
        imu.initialize(parameters)
    }
}


