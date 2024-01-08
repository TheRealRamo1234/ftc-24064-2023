package org.firstinspires.ftc.teamcode.opmodes;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.A;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.B;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_DOWN;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_UP;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.LEFT_BUMPER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.RIGHT_BUMPER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.X;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.Y;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Trigger.LEFT_TRIGGER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Trigger.RIGHT_TRIGGER;
import static org.firstinspires.ftc.teamcode.opmodes.MainAuton.gamepadEx1;
import static org.firstinspires.ftc.teamcode.opmodes.MainAuton.gamepadEx2;
import static org.firstinspires.ftc.teamcode.opmodes.MainAuton.keyPressed;
import static org.firstinspires.ftc.teamcode.opmodes.MainAuton.mTelemetry;
import static org.firstinspires.ftc.teamcode.opmodes.MainAuton.robot;
import static java.lang.Math.atan2;
import static java.lang.Math.hypot;
import static java.lang.Math.max;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.centerstage.Robot;

@TeleOp(group = "24064 Main")
public final class MainTeleOp extends LinearOpMode {

    /**
     * OpMode that is shown in driver hub; Calls all the classes and objs
     */
    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize telemetry
        mTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Initialize gamepads
        gamepadEx1 = new GamepadEx(gamepad1);
        gamepadEx2 = new GamepadEx(gamepad2);

        // Instantiate robot
        robot = new Robot(hardwareMap);

        waitForStart();

        // Control loop:
        while (opModeIsActive()) {
            // Read sensors + gamepads:
            robot.readSensors();
            gamepadEx1.readButtons();
            gamepadEx2.readButtons();

            // Gamepad 1
            // Change the heading of the drivetrain in field-centric mode
            double x = gamepadEx1.getRightX();
            if (gamepadEx1.isDown(LEFT_BUMPER)) {
                double y = gamepadEx1.getRightY();
                if (hypot(x, y) >= 0.8) robot.drivetrain.setCurrentHeading(atan2(y, x));
                x = 0;
            }

            // Control drivetrain with control stick inputs:
            if (gamepadEx1.isDown(RIGHT_BUMPER)) {
                robot.drivetrain.run(
                        -gamepadEx1.getLeftX() * 0.5,
                        -gamepadEx1.getLeftY() * 0.5,
                        -x * 0.5
                );
            } else {
                robot.drivetrain.run(
                        -gamepadEx1.getLeftX(),
                        -gamepadEx1.getLeftY(),
                        -x
                );
            }

            // Gamepad 2
            if (keyPressed(2, DPAD_UP)) robot.lift.increment();
            if (keyPressed(2, DPAD_DOWN)) robot.lift.decrement();
            if (keyPressed(2, A)) robot.lift.updateTarget();
            if (keyPressed(2, B)) robot.arm.toggleFlap();
            if (keyPressed(2, X)) robot.launcher.setActivated(true);
            if (robot.lift.getSetPoint() >= 0) {
                if (keyPressed(2, Y)) robot.arm.toggleArm();
            }
            robot.climber.set(gamepadEx2.getRightY());

            // Shared
            // The intake power is set by the greatest trigger value
            robot.intake.set(max(
                    gamepadEx2.getTrigger(RIGHT_TRIGGER) - gamepadEx2.getTrigger(LEFT_TRIGGER),
                    gamepadEx1.getTrigger(RIGHT_TRIGGER) - gamepadEx1.getTrigger(LEFT_TRIGGER)
            ));

            robot.run();
            robot.printTelemetry();
            mTelemetry.update();
        }
    }
}