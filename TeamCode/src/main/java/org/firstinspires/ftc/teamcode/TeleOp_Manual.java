/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.ArrayList;

@TeleOp
public class TeleOp_Manual extends LinearOpMode {

    //Define Wheel Biases to Manage Robot Drift
    static final double frontLeftBias = 0.96;
    static final double frontRightBias = 0.92;
    static final double backLeftBias = .92;
    static final double backRightBias = 0.96;

    //Define Timer Object
    private ElapsedTime runtime = new ElapsedTime();

    //Declare Motor Objects
    private DcMotor fL;
    private DcMotor fR;
    private DcMotor bL;
    private DcMotor bR;

    //Enter Run-Mode:
    @Override
    public void runOpMode() throws InterruptedException {

        //Initialize Telemetry Print:
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //Assign Motors
        fL = hardwareMap.get(DcMotor.class, "FrontLeft");
        fR = hardwareMap.get(DcMotor.class, "FrontRight");
        bL = hardwareMap.get(DcMotor.class, "BackLeft");
        bR = hardwareMap.get(DcMotor.class, "BackRight");

        bL.setDirection(DcMotor.Direction.REVERSE);

        bR.setDirection(DcMotor.Direction.FORWARD);

        fL.setDirection(DcMotor.Direction.REVERSE);

        fR.setDirection(DcMotor.Direction.FORWARD);

        bL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bL.setZeroPowerBehavior((DcMotor.ZeroPowerBehavior.BRAKE));
        bR.setZeroPowerBehavior((DcMotor.ZeroPowerBehavior.BRAKE));
        fL.setZeroPowerBehavior((DcMotor.ZeroPowerBehavior.BRAKE));
        fR.setZeroPowerBehavior((DcMotor.ZeroPowerBehavior.BRAKE));

        //Define TaskManipulator: new claw(wrist servo, beak servo, left lifter, right lifter, elbow);
        actuators gripper = new actuators(hardwareMap.get(Servo.class, "wrist"), hardwareMap.get(Servo.class, "rotationServo"), hardwareMap.get(Servo.class, "beak"), hardwareMap.get(DcMotor.class, "lifterLeft"), hardwareMap.get(DcMotor.class, "lifterRight"), hardwareMap.get(DcMotor.class, "elbow"), hardwareMap.get(DistanceSensor.class, "lifterHeightSensor"));

        //Wait for Driver to Start
        waitForStart();

        //Re-initialize Runtime
        runtime.reset();

        //Until the Match-End:
        while (opModeIsActive()) {
            telemetry.addData("WristPos:", gripper.getWristPosition());
            telemetry.addData("RotPos:", gripper.getRotationServoPosition());
            telemetry.addData("ElbowPos:", gripper.getElbowPos());
            telemetry.addData("LifterPos:", gripper.getLiftPos());
            telemetry.addData("Height", "CM: " + hardwareMap.get(DistanceSensor.class, "lifterHeightSensor").getDistance(DistanceUnit.CM));
            telemetry.addData("FrontDist", "CM: " + hardwareMap.get(DistanceSensor.class, "frontDistanceSensor").getDistance(DistanceUnit.CM));

            if(gamepad1.a){
                stopDriveBase();
                gripper.moveToInsertPosition();
            }
            else if(gamepad1.b){
                stopDriveBase();
                gripper.moveToPickupPosition();
            }
            else if(gamepad1.x){
                stopDriveBase();
                gripper.moveToSpecimenExtractPos();
            }
            else if(gamepad1.y){
                stopDriveBase();
                gripper.moveToHighBucketPosition();
            }
            else if(gamepad1.left_bumper){
                stopDriveBase();
                gripper.moveToTransportPosition();
            }
            else if(gamepad1.left_trigger == 1){
                stopDriveBase();
                gripper.moveToStartingPosition();
            }
            else if(gamepad1.right_bumper){
                stopDriveBase();
                gripper.moveToHangInsertPosition();
            }
            else if(gamepad1.right_trigger == 1){
                stopDriveBase();
                gripper.initializePosition();
            }
            else if(Math.abs(gamepad2.left_stick_y) == 1){
                if(gamepad2.left_stick_y == 1){
                    while(gamepad2.left_stick_y == 1){
                        gripper.lift(-1);
                        driveBase();
                    }
                    gripper.hold();
                }
                else{
                    while(gamepad2.left_stick_y == -1){
                        gripper.lift(1);
                        driveBase();
                    }
                    gripper.hold();
                }
            }
            else if(Math.abs(gamepad2.left_stick_x) == 1){

                stopDriveBase();

                if(gamepad2.left_stick_x == 1){
                    gripper.rotate(-0.03);
                }
                else{
                    gripper.rotate(0.03);
                }
            }
            else if(Math.abs(gamepad2.right_stick_y) == 1){

                stopDriveBase();

                if(gamepad2.right_stick_y == 1){
                    gripper.wristRotate(-0.03);
                }
                else{
                    gripper.wristRotate(0.03);
                }
            }
            else if(Math.abs(gamepad2.right_stick_x) == 1){
                if(gamepad2.right_stick_x == 1){
                    while(gamepad2.right_stick_x == 1){
                        gripper.elbowRotate(-0.5);
                        driveBase();
                    }
                    gripper.hold();
                }
                else{
                    while(gamepad2.right_stick_x == -1){
                        gripper.elbowRotate(0.5);
                        driveBase();
                    }
                    gripper.hold();
                }
            }
            else if(gamepad2.a){
                stopDriveBase();
                gripper.changeClawState();
            }
            else if(gamepad2.b){
                stopDriveBase();
                gripper.hang();
            }
            else if(gamepad2.x){
                stopDriveBase();
                if(!gripper.getCloseState()){gripper.changeClawState();}
                gripper.liftTo(1250);
            }
            else if(gamepad2.y){
                // Fill In
            }
            else if(gamepad2.left_bumper){
                stopDriveBase();
                gripper.rotateTo(-225);
            }
            else if(gamepad2.left_trigger == 1){
                stopDriveBase();
                gripper.rotateTo(-180);
            }
            else if(gamepad2.right_bumper){
                stopDriveBase();
                gripper.rotateTo(-135);
            }
            else if(gamepad2.right_trigger == 1){
                stopDriveBase();
                gripper.rotateTo(-90);
            }
            else{
                driveBase();
            }
        }
    }

    //Normal DriveBase Functionality
    public void driveBase() {

        //Define Fine-Control Scale
        double fineScale;
        if(Math.abs(gamepad1.left_stick_y) >= 0.7){fineScale = 0.4;}
        else{fineScale = 1;}

        double powX;
        double powY;
        double addLeft;
        double addRight;
        if(gamepad1.right_stick_x >= 0){
            powX = Math.pow(Math.abs(gamepad1.right_stick_x), 2);
        }
        else{
            powX = -1.0 * Math.pow(Math.abs(gamepad1.right_stick_x), 2);
        }
        if(gamepad1.right_stick_y >= 0){
            powY = -1 * Math.pow(Math.abs(gamepad1.right_stick_y), 2);
        }
        else{
            powY = Math.pow(Math.abs(gamepad1.right_stick_y), 2);
        }
        if(gamepad1.left_stick_x >= 0){
            addLeft = 0.5 * Math.pow(Math.abs(gamepad1.left_stick_x), 2);
            addRight = -0.5 * Math.pow(Math.abs(gamepad1.left_stick_x), 2);
        }
        else{
            addLeft = -0.5 * Math.pow(Math.abs(gamepad1.left_stick_x), 2);
            addRight = 0.5 * Math.pow(Math.abs(gamepad1.left_stick_x), 2);
        }

        // Send calculated power to wheels
        double a = (powX + powY) * (Math.pow(2, -0.5));
        double b = (-powX + powY) * (Math.pow(2, -0.5));
        fL.setPower((a + addLeft) * frontLeftBias * fineScale);
        fR.setPower((b + addRight) * frontRightBias * fineScale);
        bL.setPower((b + addLeft) * backLeftBias * fineScale);
        bR.setPower((a + addRight) * backRightBias * fineScale);

        // DriveBase Data Update (Via Telemetry)
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.update();
    }

    public void stopDriveBase(){
        fL.setPower(0);
        fR.setPower(0);
        bL.setPower(0);
        bR.setPower(0);
    }
}