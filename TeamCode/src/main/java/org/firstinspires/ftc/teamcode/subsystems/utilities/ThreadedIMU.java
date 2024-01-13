package org.firstinspires.ftc.teamcode.subsystems.utilities;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

public final class ThreadedIMU extends Thread {

    private final IMU imu;

    private double heading, angularVelo;

    private boolean run = true;

    public ThreadedIMU(HardwareMap hw, String name, RevHubOrientationOnRobot imuOrientation) {
        imu = hw.get(IMU.class, name);
        imu.resetDeviceConfigurationForOpMode();
        imu.resetYaw();
        imu.initialize(new IMU.Parameters(imuOrientation));

        start();
    }

    @Override
    public void run() {
        while (run) {
            heading = imu.getRobotYawPitchRollAngles().getYaw(RADIANS);
            angularVelo = imu.getRobotAngularVelocity(RADIANS).zRotationRate;
        }
    }

    public double getHeading() {
        return heading;
    }

    public double getAngularVelo() {
        return angularVelo;
    }

    @Override
    public void interrupt() {
        run = false;
    }
}