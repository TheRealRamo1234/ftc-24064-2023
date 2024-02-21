package org.firstinspires.ftc.teamcode.opmodes.centerstage;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.B;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_DOWN;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_UP;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.LEFT_BUMPER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.RIGHT_BUMPER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.X;

import static java.lang.Math.toRadians;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.subsystems.centerstage.Robot;
import org.firstinspires.ftc.teamcode.subsystems.centerstage.vision.AprilTagLocalization;
import org.firstinspires.ftc.teamcode.subsystems.centerstage.vision.PropSensor;

@Config
@Autonomous(group = "24064 Main", preselectTeleOp = "MainTeleOp")
public final class MainAuton extends LinearOpMode {
    static Robot robot;
    static PropSensor propSensor;
    static AprilTagLocalization aprilTag;
    public static MultipleTelemetry mTelemetry;
    public static GamepadEx gamepadEx1, gamepadEx2;

    public static Pose2d autonEndPose = null;

    public static boolean
            isRed = false,
            isTop = true;

    public static final double
            LEFT = toRadians(180),
            FORWARD = toRadians(90),
            RIGHT = toRadians(0),
            BACKWARD = toRadians(270);

    public static double
            BOTTOM_START_X = -37,
            BACKBOARD_X = 52;

    public static EditablePose
            // Bottom
            botStartRed = new EditablePose(BOTTOM_START_X, -61.788975, FORWARD),
            botLeftSpikeRed = new EditablePose(-46, -40, FORWARD),
            botCenterSpikeRed = new EditablePose(-41, -32, FORWARD),
            botRightSpikeRed = new EditablePose(-37, -35.5, FORWARD),
            botLeftPixelDodgeRed = new EditablePose(-34, -50, FORWARD),
            botCenterPixelDodgeRed = new EditablePose (-55, -38, FORWARD),
            botCenterPixelDodgeRed2 = new EditablePose(-55, -10, LEFT),
            botStageDoorRed = new EditablePose(-34, -10, LEFT),
            botTransitionRed = new EditablePose(35.5, -10, LEFT),
            botTransitionRed2 = new EditablePose(35.5, -35.5, LEFT),
            botLeftBackdropRed = new EditablePose(BACKBOARD_X, -27, LEFT),
            botCenterBackdropRed = new EditablePose(BACKBOARD_X, -36, LEFT),
            botRightBackdropRed = new EditablePose(BACKBOARD_X, -41, LEFT),
            // Top
            topLeftBackdropRed = new EditablePose(BACKBOARD_X, -27, LEFT),
            topCenterBackdropRed = new EditablePose(BACKBOARD_X, -36, LEFT),
            topRightBackdropRed = new EditablePose(BACKBOARD_X, -41, LEFT),
            topParkingRed = new EditablePose(47.5, -60, LEFT);

    private EditablePose prop, dodge, yellowPixel;

    public static boolean keyPressed(int gamepad, GamepadKeys.Button button) {
        return (gamepad == 2 ? gamepadEx2 : gamepadEx1).wasJustPressed(button);
    }

    @Override
    public void runOpMode() throws InterruptedException {

        mTelemetry = new MultipleTelemetry(telemetry);

        gamepadEx1 = new GamepadEx(gamepad1);

        robot = new Robot(hardwareMap);

        // Get gamepad 1 button input and save "right" and "red" booleans for autonomous configuration:
        while (opModeInInit() && !(gamepadEx1.isDown(RIGHT_BUMPER) && gamepadEx1.isDown(LEFT_BUMPER))) {
            gamepadEx1.readButtons();
            if (keyPressed(1, DPAD_UP)) isTop = true;
            if (keyPressed(1, DPAD_DOWN)) isTop = false;
            if (keyPressed(1, B)) isRed = true;
            if (keyPressed(1, X)) isRed = false;
            mTelemetry.addLine("| B - RED | X - BLUE |");
            mTelemetry.addLine("| D-pad-down - BOTTOM | D-pad-up - TOP |");
            mTelemetry.addLine();
            mTelemetry.addLine("Selected " + (isRed ? "RED" : "BLUE") + " " + (isTop ? "TOP" : "BOTTOM"));
            mTelemetry.addLine("Press both shoulder buttons to confirm!");
            mTelemetry.update();
        }

        TrajectorySequence[] trajectories = {getTrajectory(0), getTrajectory(1), getTrajectory(2)};
        TrajectorySequence trajectory = null;

        propSensor = new PropSensor(hardwareMap, isRed);

        while (!propSensor.getIsOpened()) {
            mTelemetry.addLine("Confirmed " + (isRed ? "RED" : "BLUE") + " " + (isTop ? "TOP" : "BOTTOM"));
            mTelemetry.addLine("Camera is not open");
            mTelemetry.update();
        }

        while (!isStarted() && !isStopRequested()) {
            int randomization = propSensor.propPosition();
            trajectory = trajectories[randomization];
            mTelemetry.addData("Predicted Prop Placement", randomization);
            mTelemetry.update();
            sleep(50);
        }

//        propSensor.getCamera().stopStreaming();
        propSensor.getCamera().closeCameraDeviceAsync(() -> {
            mTelemetry.addLine("Camera closed");
            mTelemetry.update();
        });

        robot.drivetrain.followTrajectorySequenceAsync(trajectory);

        while (opModeIsActive()) {
            if (!opModeIsActive()) {
                return;
            }

            robot.readSensors();

            robot.drivetrain.update();
            robot.run();
        }

        autonEndPose = robot.drivetrain.getPoseEstimate();
    }

    private TrajectorySequence getTrajectory(int randomization) {
        switch (randomization) {
            case 0:
                prop = isTop ? (isRed ? botRightSpikeRed : botLeftSpikeRed) : (isRed ? botLeftSpikeRed : botRightSpikeRed);
                dodge = botLeftPixelDodgeRed;
                yellowPixel = isTop ? (isRed ? topLeftBackdropRed : topRightBackdropRed) : (isRed ? botLeftBackdropRed : botRightBackdropRed);
                break;
            case 1:
                prop = botCenterSpikeRed;
                dodge = botCenterPixelDodgeRed;
                yellowPixel = isTop ? topCenterBackdropRed : botCenterBackdropRed;
                break;
            case 2:
                prop = isTop ? (isRed ? botLeftSpikeRed : botRightSpikeRed) : (isRed ? botRightSpikeRed : botLeftSpikeRed);
                dodge = botLeftPixelDodgeRed;
                yellowPixel = isTop ? (isRed ? topRightBackdropRed : topLeftBackdropRed) : (isRed ? botRightBackdropRed : botLeftBackdropRed);
                break;
        }

        Pose2d start = botStartRed.bySide().byAlliancePose2d();
        robot.drivetrain.setPoseEstimate(start);

        TrajectorySequenceBuilder builder = robot.drivetrain.trajectorySequenceBuilder(start);

        addPurplePixel(builder, randomization);
        addYellowPixel(builder, randomization);

        return builder.build();
    }

    private void addYellowPixel(TrajectorySequenceBuilder builder, int randomization) {
        if (!isTop) {
            if (isAudienceSide(randomization) || isBackboardSide(randomization))
                builder.lineTo(botStageDoorRed.byAllianceVec());
            else
                builder.lineToSplineHeading(botStageDoorRed.byAlliancePose2d());

            builder.lineToConstantHeading(botTransitionRed.byAllianceVec())
                    .lineToSplineHeading(botTransitionRed2.byAlliancePose2d());
        }

        // Scoring
        builder.lineToSplineHeading(yellowPixel.byAlliancePose2d())
                .addTemporalMarker(() -> robot.lift.setToAutonHeight(0)) // Lift and arm extend
                .waitSeconds(1)
                .addTemporalMarker(() -> robot.arm.setArm(true))
                .UNSTABLE_addTemporalMarkerOffset(1, () -> robot.arm.setFlap(false))
                .waitSeconds(2)
                .addTemporalMarker(() -> robot.lift.setToAutonHeight(400))
                .waitSeconds(2)
                .addTemporalMarker(() -> robot.arm.setArm(false))
                .UNSTABLE_addTemporalMarkerOffset(1, () -> robot.lift.retract());

        // Parking for top
        if (isTop)
            builder.forward(5)
                    .lineToSplineHeading(topParkingRed.byAlliancePose2d())
                    .back(13);
        else
            builder.waitSeconds(1);
    }

    private void addPurplePixel(TrajectorySequenceBuilder builder, int randomization) {
        builder.lineToSplineHeading(prop.bySide().byAlliancePose2d());

        if (!isTop) {
            if (isAudienceSide(randomization) || isCenter(randomization))
                builder.addTemporalMarker(() -> robot.rollers.intake(0.30))
                        .UNSTABLE_addTemporalMarkerOffset(0.2, () -> robot.rollers.intake(0))
                        .back(11)
                        .lineToSplineHeading(dodge.byAlliancePose2d());

            if (isCenter(randomization))
                builder.lineToSplineHeading(botCenterPixelDodgeRed2.byAlliancePose2d());

            if (isBackboardSide(randomization))
                builder.turn(toRadians(isRed ? -90 : 90))
                        .forward(3.5)
                        .addTemporalMarker(() -> robot.rollers.intake(0.30))
                        .UNSTABLE_addTemporalMarkerOffset(0.2, () -> robot.rollers.intake(0))
                        .waitSeconds(0.5)
                        .back(8)
                        .turn(toRadians(180));
        } else {
            if (isAudienceSide(randomization))
                builder.turn(toRadians(isRed ? 90 : -90))
                        .forward(3.5)
                        .addTemporalMarker(() -> robot.rollers.intake(0.30))
                        .UNSTABLE_addTemporalMarkerOffset(0.2, () -> robot.rollers.intake(0))
                        .waitSeconds(0.5);
            else
                builder.addTemporalMarker(() -> robot.rollers.intake(0.30))
                        .UNSTABLE_addTemporalMarkerOffset(0.2, () -> robot.rollers.intake(0))
                        .back(11);
        }
    }

    private boolean isCenter(int randomization) {
        return randomization == 1;
    }

    private boolean isAudienceSide(int randomization) {
        return isRed && randomization == 0 || !isRed && randomization == 2;
    }

    private boolean isBackboardSide(int randomization) {
        return isRed && randomization == 2 || !isRed && randomization == 0;
    }

    private static class EditablePose {
        public double x, y, heading;

        private EditablePose(double x, double y, double heading) {
            this.x = x;
            this.y = y;
            this.heading = heading;
        }

        // Switches *red* alliance coordinates to blue alliance
        private EditablePose byAlliance() {
            EditablePose pose = new EditablePose(x, y, heading);
            if (!isRed) {
                pose.y *= -1;
                pose.heading *= -1;
            }
            return pose;
        }

        private Pose2d byAlliancePose2d() {
            return byAlliance().toPose2d();
        }

        private Vector2d byAllianceVec() {
            return byAlliancePose2d().vec();
        }

        private Pose2d toPose2d() {
            return new Pose2d(x, y, heading);
        }

        private EditablePose bySide() {
            EditablePose pose = new EditablePose(x, y, heading);
            if (isTop) pose.x = -x - 23.5;
            return pose;
        }
//
//        private EditablePose byBoth() {
//            return byAlliance().bySide();
//        }
//
//        private EditablePose flipBySide() {
//            boolean isRight = MainAuton.isTop == isRed;
//            if (!isTop) heading = Math.PI - heading;
//            if (!isTop) x = (X_START_BOTTOM + X_START_TOP) / 2 - x;
//            return this;
//        }
    }
}