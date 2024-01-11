package org.firstinspires.ftc.teamcode.teleop;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.Locale;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.Constants;
import org.firstinspires.ftc.teamcode.common.DriveConstants;
import org.firstinspires.ftc.teamcode.common.HardwareDrive;
import org.firstinspires.ftc.teamcode.common.Utility;

@TeleOp(name = "Base Drive Complete", group = "Drive")
public class BaseDriveComplete extends LinearOpMode {

    private final HardwareDrive robot = new HardwareDrive();
    private final ElapsedTime runtime = new ElapsedTime();

    double drivePower = 0.70;
    double servoTargetPosition = 0.312;
    double trapdoorTargetPos = 0.6;
    int upperLimit = 3100;
    int lowerLimit = -25;

    double upperLimTrapdoor = 0.65;
    double lowerLimTrapdoor = 0.332;


    @Override
    public void runOpMode() {
        composeTelemetry();
        robot.init(hardwareMap);
        telemetry.addData("Say", "Hello Driver");
        runtime.reset();
        //robot.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //robot.lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        //robot.imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
        while (opModeIsActive()) {
            //UpdateGripper();
            UpdateTelemetry();
            if (gamepad1.right_bumper) drivePower = 1;
            DriveTrainBase(drivePower);
            //DriveMicroAdjust();
            intakeDrive();
            //trapdoor();
        }
    }

    @Utility.Encapsulate
    private void DriveTrainBase(double drivePower) {
        double directionX = Math.pow(gamepad1.left_stick_x, 1); //Strafe
        double directionY = -Math.pow(gamepad1.left_stick_y, 1); //Forward
        double directionR = Math.pow(gamepad1.right_stick_x, 1); //Turn

        double armPower = Math.pow(gamepad2.right_stick_y, 1);

        if (gamepad1.left_stick_x < 0.2 && gamepad1.left_stick_x > -0.2) directionX = 0;
        if (gamepad1.left_stick_y < 0.2 && gamepad1.left_stick_y > -0.2) directionY = 0;

        robot.lf.setPower((directionY + directionR + directionX) * drivePower);
        robot.rf.setPower((directionY - directionR - directionX) * drivePower);
        robot.lb.setPower((directionY + directionR - directionX) * drivePower);
        robot.rb.setPower((directionY - directionR + directionX) * drivePower);

        double armPos = robot.arm.getCurrentPosition();
        if (armPos > upperLimit && armPower < 0){
            armPower = 0;
        }
        else if (armPos < lowerLimit && armPower > 0){
            armPower = 0;
        }
        robot.arm.setPower((armPower) * -0.6);

        //if ((armPos < DriveConstants.armMaxPos || armPower > 0) && (armPos > DriveConstants.armMinPos || armPower < 0)){
        //}

        /*if (gamepad2.dpad_down){
            if (servoTargetPosition >= 0.996){
                servoTargetPosition = 0;
            }
            else{
                servoTargetPosition += 0.004;
            }
        }

        if (gamepad2.dpad_up){
            if (servoTargetPosition <= 0.004){
                servoTargetPosition = 1;
            }
            else{
                servoTargetPosition -= 0.004;
            }
        }*/

        if (gamepad2.dpad_down && servoTargetPosition < 1){ // && servoTargetPosition < 0.8
            servoTargetPosition += 0.004;
        }
        else if (gamepad2.dpad_up && servoTargetPosition > 0){ //&& servoTargetPosition > 0
            servoTargetPosition -= 0.004;


        }
        telemetry.addData("servo tgt: ", servoTargetPosition);
        if (robot.armServo.getPosition() != servoTargetPosition){
            telemetry.addLine("Repositioning servo");
            robot.armServo.setPosition(servoTargetPosition);
        }

        if (gamepad2.x){
            robot.trapdoor.setPosition(0.35); //Open
        }
        if (gamepad2.y){
            robot.trapdoor.setPosition(0.6); //Close
        }

        if (gamepad1.y){
            upperLimit = 10000;
            lowerLimit = -10000;
        }

        if (gamepad1.dpad_down && trapdoorTargetPos < upperLimTrapdoor){ // && servoTargetPosition < 0.8
            trapdoorTargetPos += 0.001;
            if (robot.trapdoor.getPosition() != trapdoorTargetPos){
                telemetry.addLine("Repositioning servo");
                robot.trapdoor.setPosition(trapdoorTargetPos);
            }
        }
        else if (gamepad1.dpad_up && trapdoorTargetPos > lowerLimTrapdoor){ //&& servoTargetPosition > 0
            trapdoorTargetPos -= 0.001;
            if (robot.trapdoor.getPosition() != trapdoorTargetPos){
                telemetry.addLine("Repositioning servo");
                robot.trapdoor.setPosition(trapdoorTargetPos);
            }
        }
        telemetry.addData("trapdoor tgt: ", trapdoorTargetPos);



        if (gamepad2.y){
            robot.servoPlane.setPosition(0);
        }


        //0 - right_back
        //1 - left_forward
        //2 - right_front
        //3 - left_back

        /*if (armPos < Constants.elevatorPositionTop && gamepad2.right_stick_y < 0) {
            robot.arm.setPower((gamepad2.left_stick_y) * 0.1 - -0.001);
        }
        else if (armPos > Constants.elevatorPositionBottom && gamepad2.right_stick_y > 0) {
            robot.arm.setPower((gamepad2.left_stick_y) * 0.01);
        } else*/
    }

    private void DriveMicroAdjust() {
        if (gamepad1.dpad_up) {
            robot.lf.setPower(-0.4);
            robot.rf.setPower(+0.4);
            robot.lb.setPower(-0.4);
            robot.rb.setPower(+0.4);
        } else if (gamepad1.dpad_down) {
            robot.lf.setPower(+0.4);
            robot.rf.setPower(-0.4);
            robot.lb.setPower(+0.4);
            robot.rb.setPower(-0.4);
        } else if (gamepad1.dpad_right) {
            robot.lf.setPower(0.4);
            robot.rf.setPower(0.4);
            robot.lb.setPower(0.4);
            robot.rb.setPower(0.4);
        } else if (gamepad1.dpad_left) {
            robot.lf.setPower(-0.4);
            robot.rf.setPower(-0.4);
            robot.lb.setPower(-0.4);
            robot.rb.setPower(-0.4);
        }

        if (gamepad1.left_trigger == 1) {
            robot.lf.setPower(-0.4);
            robot.rf.setPower(0.4);
            robot.lb.setPower(-0.4);
            robot.rb.setPower(0.4);
        } else if (gamepad1.right_trigger == 1) {
            robot.lf.setPower(0.4);
            robot.rf.setPower(-0.4);
            robot.lb.setPower(0.4);
            robot.rb.setPower(-0.4);
        }
    }

    private void trapdoor(){
        if (gamepad2.x){
            robot.trapdoor.setPosition(0.1);
        }
        if (gamepad2.b){
            robot.trapdoor.setPosition(0.9);
        }
    }

    private void intakeDrive(){
        double directionY = Math.pow(gamepad2.left_stick_y, 1);

        if (directionY > 0.2  || directionY < 0.2){
            robot.intake.setPower(directionY/1.3);
        }
    }

    /*private void UpdateGripper() {
        if (gamepad2.left_trigger > 0.01) robot.serv0.setPower(0.22 * gamepad2.left_trigger - 0);
        else if (gamepad2.right_trigger > 0.01) robot.serv0.setPower(-0.16 * gamepad2.right_trigger + 0);
    }*/

    private void UpdateTelemetry() {
        double curPos = robot.arm.getCurrentPosition();
        telemetry.addData("g1.X", gamepad1.left_stick_x);
        telemetry.addData("g1.Y", -gamepad1.left_stick_y);
        telemetry.addData("g1.R", gamepad1.right_stick_x);
        telemetry.addData("Arm Position", curPos);
        //telemetry.addData("g2.L", gamepad2.right_stick_y);
        telemetry.addData("g2.arm", gamepad2.right_stick_y);
        telemetry.update();
    }

    public void composeTelemetry() {
        telemetry.addAction(() -> {
            //robot.angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            //robot.gravity = robot.imu.getGravity();
        });

        //telemetry.addLine().addData("status", () -> robot.imu.getSystemStatus().toShortString()).addData("calib", () -> robot.imu.getCalibrationStatus().toString());
        //telemetry.addLine().addData("heading", () -> formatAngle(robot.angles.angleUnit, robot.angles.firstAngle)).addData("roll", () -> formatAngle(robot.angles.angleUnit, robot.angles.secondAngle)).addData("pitch", () -> formatAngle(robot.angles.angleUnit, robot.angles.thirdAngle));
        //telemetry.addLine().addData("grvty", () -> robot.gravity.toString()).addData("mag", () -> String.format(Locale.getDefault(), "%.3f", Math.sqrt(robot.gravity.xAccel * robot.gravity.xAccel + robot.gravity.yAccel * robot.gravity.yAccel + robot.gravity.zAccel * robot.gravity.zAccel)));
    }
    String formatDegrees(double degrees) { return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees)); }
    String formatAngle(AngleUnit angleUnit, double angle) { return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle)); }
}

