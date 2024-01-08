package org.firstinspires.ftc.teamcode.subsystems.centerstage.vision;

import static org.firstinspires.ftc.teamcode.opmodes.MainAuton.mTelemetry;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

public class PropSensor {
    private final OpenCvCamera camera;
    private final PropSensorPipeline pipeline;

    public PropSensor(HardwareMap hardwareMap, boolean isBlue) {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        WebcamName name = hardwareMap.get(WebcamName.class, "webcam");
        camera = OpenCvCameraFactory.getInstance().createWebcam(name, cameraMonitorViewId);
        pipeline = new PropSensorPipeline(isBlue);

        initializeCamera();
    }

    private void initializeCamera() {
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
                camera.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);
                camera.setPipeline(pipeline);
            }

            @Override
            public void onError(int errorCode) {
                // This will be called if the camera could not be opened
            }
        });
    }

    public int propPosition() {
        return pipeline.propPosition();
    }

    public void printTelemetry() {
        mTelemetry.addData("Prop position", propPosition());
    }

    public void printNumericalTelemetry() {
        mTelemetry.addData("FPS", camera.getFps());
    }
}
