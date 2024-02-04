package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.auto.BasicPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCameraRotation;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous (name = "autoRedFar")
public class autoRedFar extends LinearOpMode {

    protected DcMotor lf;
    protected DcMotor rf;
    protected DcMotor lb;
    protected DcMotor rb;
    protected DcMotor arm;
    protected DcMotor intake;
    protected Servo armServo;
    protected Servo trapdoor;
    OpenCvCamera camera;
    PipelineRed pipeline = new PipelineRed();

    //BasicPipeline pipeline = new BasicPipeline();

    enum propPosition {
        LEFT,
        MIDDLE,
        RIGHT
    }


    @Override
    public void runOpMode() throws InterruptedException {
        lf = hardwareMap.get(DcMotor.class, "left_front");
        rf = hardwareMap.get(DcMotor.class, "right_front");
        lb = hardwareMap.get(DcMotor.class, "left_back");
        rb = hardwareMap.get(DcMotor.class, "right_back");

        arm = hardwareMap.get(DcMotor.class, "arm");

        intake = hardwareMap.get(DcMotor.class, "intake");
        armServo = hardwareMap.get(Servo.class, "armServo");
        trapdoor = hardwareMap.get(Servo.class, "trapdoor");

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "camera"), cameraMonitorViewId);

        camera.setPipeline(pipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        armServo.setPosition(0.1);

        while (!isStarted() && !isStopRequested()) {
            telemetry.addData("Prop x value: ", pipeline.getJunctionPoint().x);
            telemetry.addData("Prop area: ", pipeline.getPropAreaAttr());

            if (pipeline.getPropAreaAttr() > 25000 && pipeline.getJunctionPoint().x > 500){
                telemetry.addLine("Box Placement: Right");
            }
            else if (pipeline.getPropAreaAttr() > 25000) { //Middle Path
                telemetry.addLine("Box Placement: Middle");
            }
            else{
                telemetry.addLine("Box Placement: Left");
            }
            telemetry.update();
        }

        double propX = pipeline.getJunctionPoint().x;
        double propArea = pipeline.getPropAreaAttr();

        //Full Battery Power Constant = 0.80
        //13.7 Battery Constant = 0.87
        //13.6 Battery Constant = 0.89
        //13.39 Battery Constant = 0.9
        //13.16 BatteryConstant = 0.91
        //12.96 Battery Constant = 0.92
        //12.85 Battery Constant = 0.93
        //12.78 = 0.94
        double batteryConstant = 0.931;

        if (propArea > 25000 && propX > 500){ //Right Path
            moveForward(-0.3, (int)(1775 * batteryConstant));
            moveTurning(-0.2, (int)(2750 * batteryConstant));
            sleep(200);
            moveForward(-0.3, (int)(250 * batteryConstant));
            sleep(100);
            moveForward(0.3, (int)(250 * batteryConstant));
            intake.setPower(-0.6);
            sleep((int)(1500 * batteryConstant));
            intake.setPower(0);
            sleep(200);
            moveForward(0.3, (int)(700 * batteryConstant));
            sleep(300);
        }

        else if (propArea > 25000){ //Middle Path
            moveForward(-0.3, (int)(2100 * batteryConstant));
            sleep(200);
            moveForward(0.3, (int)(200 * batteryConstant));
            intake.setPower(-0.6);
            sleep(1500);
            intake.setPower(0);
            sleep(200);
            moveForward(0.4, (int)(300 * batteryConstant));
            moveStrafing(-0.3, (int)(500 * batteryConstant)); //Strafe left
            moveTurning(-0.2, (int)(2600 * batteryConstant)); //Turn Left
        }

        else{ //Left Path
            moveForward(-0.3, (int)(1850 * batteryConstant));
            moveTurning(0.2, (int)(2750 * batteryConstant));
            sleep(200);
            moveForward(-0.3, (int)(290 * batteryConstant));
            sleep(100);
            moveForward(0.3, (int)(200 * batteryConstant));
            intake.setPower(-0.6);
            sleep((int)(1500 * batteryConstant));
            intake.setPower(0);
            sleep(200);
            moveForward(0.3, (int)(150 * batteryConstant));
            sleep(300);
        }

        sleep(100000);
    }

    public void moveTurning(double power, int time) {
        rf.setPower(power);
        lf.setPower(power);
        rb.setPower(power);
        lb.setPower(power);
        sleep(time);
        rf.setPower(0);
        lf.setPower(0);
        rb.setPower(0);
        lb.setPower(0);

    }

    public void moveStrafing(double power, int time){
        rf.setPower(-power);
        lf.setPower(-power);
        rb.setPower(power);
        lb.setPower(power);
        sleep(time);
        rf.setPower(0);
        lf.setPower(0);
        rb.setPower(0);
        lb.setPower(0);

    }

    public void moveForward(double power, int time) {
        rf.setPower(-power);
        lf.setPower(power);
        rb.setPower(-power);
        lb.setPower(power);
        sleep(time);
        rf.setPower(0);
        lf.setPower(0);
        rb.setPower(0);
        lb.setPower(0);
    }
}