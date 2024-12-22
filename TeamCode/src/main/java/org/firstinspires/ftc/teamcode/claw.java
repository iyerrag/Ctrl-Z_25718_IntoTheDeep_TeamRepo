package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

public class claw {
    private Servo wrist;
    private DcMotor elbow;
    private DcMotor leftSlide;
    private DcMotor rightSlide;
    private Servo leftTalon;
    private Servo rightTalon;
    private Servo beak;
    private boolean closeState;



    public claw(Servo wrist, Servo leftTalon, Servo rightTalon, Servo beakServo, DcMotor lifterLeft, DcMotor lifterRight, DcMotor elbow){
        this.wrist = wrist;
        this.elbow = elbow;
        this.leftTalon = leftTalon;
        this.rightTalon = rightTalon;
        this.beak = beakServo;
        leftSlide = lifterLeft;
        rightSlide = lifterRight;

        leftSlide.setDirection(DcMotor.Direction.FORWARD);
        leftSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightSlide.setDirection(DcMotor.Direction.REVERSE);
        rightSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        elbow.setDirection(DcMotor.Direction.FORWARD);
        elbow.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elbow.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        wrist.setDirection(Servo.Direction.FORWARD);
        closeState = true;
    }

    public boolean state(){
        return closeState;
    }

    public void liftTo(int targetPos){

        leftSlide.setTargetPosition(targetPos);
        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        rightSlide.setTargetPosition(targetPos);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        if((int) (Math.random() * 2) == 0){
            leftSlide.setPower(1);
            rightSlide.setPower(1);
        }
        else {
            rightSlide.setPower(1);
            leftSlide.setPower(1);
        }
    }

    public void hold(){
        leftSlide.setTargetPosition(leftSlide.getCurrentPosition());
        rightSlide.setTargetPosition(rightSlide.getCurrentPosition());
        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elbow.setTargetPosition(elbow.getCurrentPosition());
        elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        if((int) (Math.random() * 2) == 0){
            leftSlide.setPower(1);
            rightSlide.setPower(1);
        }
        else {
            rightSlide.setPower(1);
            leftSlide.setPower(1);
        }

        elbow.setPower(1);
    }

    public void lift(double liftPwr){
        leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if ((int) (Math.random() * 2) == 0) {
            leftSlide.setPower(liftPwr);
            rightSlide.setPower(liftPwr);
        } else {
            rightSlide.setPower(liftPwr);
            leftSlide.setPower(liftPwr);
        }
    }

    public void rotateElbow(double rotatePwr){
        elbow.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        elbow.setPower(rotatePwr);
    }

    public void wristRotateTo(double targetAngle) throws InterruptedException {
        // Insert Conversion Formula
        double targetPos = targetAngle;
        wrist.setPosition(targetPos);
        Thread.sleep(500);
    }

    /*public void intakeRotate_Hold(String direction) throws InterruptedException {
        // Insert Conversion Formula
        if(direction.equals("IN")){
            intakeServo.setPower(1);
        }
        else if(direction.equals("OUT")){
            intakeServo.setPower(-1);
        }
        else{
            intakeServo.setPower(0);
        }
    }

    public void stopIntakeRotation(){
        intakeServo.setPower(0);
    }

    public void intakeRotate_FixedDuration(String direction) throws InterruptedException {
        // Insert Conversion Formula
        if(direction.equals("IN")){
            intakeServo.setPower(1);
        }
        else if(direction.equals("OUT")){
            intakeServo.setPower(-1);
        }
        else{
            intakeServo.setPower(0);
        }
        Thread.sleep(500);
        intakeServo.setPower(0);
    }*/

    public void leftTalonRotateTo(double targetAngle) throws InterruptedException {
        // Insert Conversion Formula
        double targetPos = targetAngle;
        leftTalon.setPosition(targetPos);
    }

    public void rightTalonRotateTo(double targetAngle) throws InterruptedException {
        // Insert Conversion Formula
        double targetPos = targetAngle;
        rightTalon.setPosition(targetPos);
    }

    public void beakRotateTo(double targetAngle) throws InterruptedException {
        // Insert Conversion Formula
        double targetPos = targetAngle;
        beak.setPosition(targetPos);
    }


    public void changeClawState() throws InterruptedException {
        if(closeState){
            leftTalonRotateTo(0.3);
            rightTalonRotateTo(0.6);
            beakRotateTo(0.2);
            closeState = false;
        }

        else{
            leftTalonRotateTo(.43);
            rightTalonRotateTo(0.47);
            beakRotateTo(0.55);
            closeState = true;
        }
        Thread.sleep(500);
    }

    public boolean getCloseState(){
        return closeState;
    }

    /*public void changeIntakeState() throws InterruptedException {
        if(!holdState){
            holdState = true;
            intakeRotate_FixedDuration("IN");
        }
        else{
            holdState = false;
            intakeRotate_FixedDuration("OUT");
        }
        Thread.sleep(500);
    }*/

    public void moveToHighBucketPosition() throws InterruptedException {
         liftTo(5400);
         Thread.sleep(2000);
         elbowTo(-450, 1);
         wristRotateTo(0);
    }

    public void moveToInsertPosition() throws InterruptedException {
        wristRotateTo(0.12);
        elbowTo(-2750, 1);
        liftTo(0);
    }

    public void moveToPickupPosition() throws InterruptedException {
        wristRotateTo(0.12);
        elbowTo(-2600, 1);
        wristRotateTo(0.66);
        if(closeState){changeClawState();}
        liftTo(0);
    }

    public void moveToTransportPosition() throws InterruptedException {
        beakRotateTo(0.6);
        wristRotateTo(0.12);
        elbowTo(0, 1);
        liftTo(0);
        wristRotateTo(0.66);

    }

    public void moveToSpeciminExtractPos() throws InterruptedException {
        liftTo(1400);
        elbowTo(-4000, 0.5);
        wristRotateTo(1);
        elbowTo(0, 0.5);
        beakRotateTo(0.55);
    }

    // Add Extend & Rotate Functionality
    public void elbowTo(int targetPos, double maxPow){
        elbow.setTargetPosition(targetPos);
        elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elbow.setPower(maxPow);
    }


    public void elbowMove(String direction){
        elbow.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if (direction.equals("forward")){
            elbow.setPower(.5);
        }
        else if (direction.equals("backward")){
            elbow.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            elbow.setPower(-.5);
        }
        else if (direction.equals("hold")){
            elbow.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            elbow.setPower(0);
        }
        else{};
    }




    public double getLiftPos(){
        return (leftSlide.getCurrentPosition() + rightSlide.getCurrentPosition()) / 2.0;
    }

    public double getElbowPos(){
        return (elbow.getCurrentPosition());
    }

}
