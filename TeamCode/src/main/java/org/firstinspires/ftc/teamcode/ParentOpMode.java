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

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.hardware.bosch.BNO055IMU;

/**
 * Original FTC opmode header block
 *
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 **/

/** Parent OpMode Class
 * All Teleop and Autonomous OpModes should inherit from (extend) ParentOpMode.
 * Each child/subclass OpMode should have its own unique runOpMode() method that will
 * override the ParentOpMode runOpMode() method.
 **/

@TeleOp(name="Parent Opmode", group="Linear Opmode")
@Disabled
public class ParentOpMode extends LinearOpMode {

    // Declare OpMode members, hardware variables
    public ElapsedTime runtime = new ElapsedTime();

    private DcMotor rightFront = null;
    private DcMotor rightBack = null;
    private DcMotor leftFront = null;
    private DcMotor leftBack = null;
    // we named the motors

    private CRServo duckWheel = null;

    private CRServo intake = null;

    private Servo lift = null;

    // gyro
    BNO055IMU imu;
    Orientation angles = new Orientation();

    //Other Global Variables
    //put global variables here...
    double liftposition = .5;

    public void initialize() {
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        rightFront = hardwareMap.get(DcMotor.class, "rf_drive");
        rightBack = hardwareMap.get(DcMotor.class, "rb_drive");
        leftBack = hardwareMap.get(DcMotor.class, "lb_drive");
        leftFront = hardwareMap.get(DcMotor.class, "lf_drive");


        duckWheel = hardwareMap.get(CRServo.class, "duck_whel");

        intake = hardwareMap.get(CRServo.class, "cool_intake");

        lift = hardwareMap.get(Servo.class, "lift");
        //intakeServo = hardwareMap.get(CRServo.class, "intake_servo");
        //shooterFlipper = hardwareMap.get(Servo.class,"shooterFlipper_servo");

        //Set motor run mode (if using SPARK Mini motor controllers)
        //

        //Set Motor  and servo Directions
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        leftFront.setDirection(DcMotor.Direction.FORWARD);

        duckWheel.setDirection(CRServo.Direction.FORWARD);

        intake.setDirection(CRServo.Direction.REVERSE);

        lift.setDirection(Servo.Direction.REVERSE);
        //shooterFlipper.setDirection(Servo.Direction.REVERSE);


        //Set range for special Servos
        //wobbleLift.scaleRange(0.15,.85); //Savox PWM range is between 0.8 and 2.2 ms. REV Hub puts out 0.5-2.5ms.

        //Set brake or coast modes.
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);//BRAKE or FLOAT (Coast)
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        gyroInitialize();

        //Update Driver Station Status Message
        telemetry.addData("Status:", "Initialized");
        telemetry.update();
    }


    /**
     * runOpMode() will be overridden in child OpMode.
     * Basic structure should remain intact (init, wait for start, while(opModeIsActive),
     * Additionally, Emergency conditions should be checked during every cycle
     */
    @Override
    public void runOpMode() {

        initialize();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        //releaseLatch();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            //code here should never actually execute.
            // This function will be be overridden by child opmode classes


            //include emergency stop check in all runOpMode() functions/methods
            if (emergencyStopped()) {
                break;
            }

            telemetry.update();
        }
    }

    /*****************************/
    //Controls should be mapped here to avoid
    //CONTROLLER MAP
    //Thumbsticks
    public double left_sticky_x() {
        return gamepad1.left_stick_x;
    }

    public double left_sticky_y() {
        return -gamepad1.left_stick_y;
    }

    public double right_sticky_x() {
        return gamepad1.right_stick_x;
    }

    public double right_sticky_y() {
        return -gamepad1.right_stick_y;
    }

    //Buttons

    public boolean emergencyButtons() {
        return gamepad1.x && gamepad1.y;
    }

    public boolean duckWheelButton() {
        return gamepad1.b;
    }

    public boolean intakeButton() {
        return gamepad1.left_bumper ;
    }

    public boolean intakeReverseButton() {
        return gamepad1.right_bumper;
    }

    public boolean liftTheButtonUp() {
        return gamepad1.dpad_up;
    }

    public boolean liftTheButtonDown() {
        return gamepad1.dpad_down;
    }

    public boolean lifttheMONKEupbutton() {
        if (gamepad1.right_trigger >= 0.5) {
            return true;
        }
        else {
            return false;
        }
    }

    public boolean lifttheMONKEdownbutton() {
        if (gamepad1.left_trigger >= 0.5) {
            return true;
        }
        else {
            return false;
        }
    }

    public boolean restartGyroButton() {
        return gamepad1.back;
    }




   /* public boolean shootButton(){
        if((gamepad1.right_trigger>.25)||(gamepad2.right_trigger>.25)){
            return true;         // Converts analog triggers into digital button presses (booleans)
        }
        else{
            return false;
        }
    }
*/

    /****************************/
    // Emergency Stop Function
    public boolean emergencyStopped() {
        if (emergencyButtons()) {
            //stop all motors, servos, etc.
            stopDrive();

            return true;
        } else {
            return false;
        }
    }


    /*****************************/
    //Drive Methods

    // Assign left and right drive speed using arguments/parameters rather than hardcoding
    // thumb stick values inside function body. This will allow tank drive to be reused for
    // autonomous programs without additional work
    public void tankdrive(double left, double right) {

        rightFront.setPower(right);
        rightBack.setPower(right);
        leftFront.setPower(left);
        leftBack.setPower(left);

        telemetry.addData("lfspeed ",left);
        telemetry.addData("rfspeed ",right);
    }

    public void stopDrive() {
        tankdrive(0, 0);
    }

public void holonomicDrive(){
        double wheelVelocityFrontRight;
        double wheelVelocityBackRight;
        double wheelVelosityFrontLeft;
        double wheelVelosityBackLeft;
        double angleOffset  = 90;

        double robotSpeed = Math.hypot(left_sticky_y(), left_sticky_x());

        double robotAngle = Math.atan2(left_sticky_y(),left_sticky_x())+Math.toRadians(angleOffset);

        double speedOfRotation = -right_sticky_x();

        wheelVelocityFrontRight = robotSpeed*Math.sin(robotAngle+(Math.PI/4))-speedOfRotation;
        wheelVelosityFrontLeft = robotSpeed*Math.cos(robotAngle+(Math.PI/4))+speedOfRotation;
        wheelVelocityBackRight = robotSpeed*Math.cos(robotAngle+(Math.PI/4))-speedOfRotation;
        wheelVelosityBackLeft = robotSpeed*Math.sin(robotAngle+(Math.PI/4))+speedOfRotation;

        rightFront.setPower(wheelVelocityFrontRight);
        leftFront.setPower(wheelVelosityFrontLeft);
        rightBack.setPower(wheelVelocityBackRight);
        leftBack.setPower(wheelVelosityBackLeft);

        telemetry.addData("lfspeed ",wheelVelosityFrontLeft);
        telemetry.addData("lbspeed ",wheelVelosityBackLeft);
        telemetry.addData("rfspeed ", wheelVelocityFrontRight);
        telemetry.addData("rbspeed ",wheelVelocityBackRight);
}

    public void fieldCentric(){
        double wheelVelocityFrontRight;
        double wheelVelocityBackRight;
        double wheelVelosityFrontLeft;
        double wheelVelosityBackLeft;

        double angleOffset = getGyroAngle() - 90;

        double robotSpeed = Math.hypot(left_sticky_y(), left_sticky_x());

        double robotAngle = Math.atan2(left_sticky_y(),left_sticky_x())-Math.toRadians(angleOffset);

        double speedOfRotation = -right_sticky_x();

        if (restartGyroButton()){
            gyroInitialize();   //Re-initialize gyro. Tried using resetAngle(), but that didn't work.
        }

        wheelVelocityFrontRight = robotSpeed*Math.sin(robotAngle+(Math.PI/4))-speedOfRotation;
        wheelVelosityFrontLeft = robotSpeed*Math.cos(robotAngle+(Math.PI/4))+speedOfRotation;
        wheelVelocityBackRight = robotSpeed*Math.cos(robotAngle+(Math.PI/4))-speedOfRotation;
        wheelVelosityBackLeft = robotSpeed*Math.sin(robotAngle+(Math.PI/4))+speedOfRotation;

        rightFront.setPower(wheelVelocityFrontRight);
        leftFront.setPower(wheelVelosityFrontLeft);
        rightBack.setPower(wheelVelocityBackRight);
        leftBack.setPower(wheelVelosityBackLeft);

        telemetry.addData("LF speed ",wheelVelosityFrontLeft);
        telemetry.addData("RF speed ", wheelVelocityFrontRight);
        telemetry.addData("LB speed ",wheelVelosityBackLeft);
        telemetry.addData("RB speed ",wheelVelocityBackRight);

        telemetry.addData("angle", angleOffset);
    }


    /*****************************/
    //More Methods (Functions)

    public void duckWheelSpin(){
        if (duckWheelButton()) {
            duckWheel.setPower(1);

            telemetry.addData( "duckwheelspinner", "spinning");
        }
        else {
            duckWheel.setPower(0);

            telemetry.addData("duckwheelspinner","stopped");

        }
    }

    public void intakeEatr(){
        double intakeSpeed = .9420;
        if (intakeButton()) {
            intake.setPower(intakeSpeed);
        }
        else if  (intakeReverseButton()) {
            intake.setPower(-intakeSpeed);
        }
        else{
            intake.setPower(0);
        }
    }

    public void liftTheThreeUpandDown(){
        double wat = .000420;
        double lowerLimit = .25;
        double highLimit = .75;
        if (liftTheButtonUp()){
            liftposition = liftposition + wat;
        }
        if (liftTheButtonDown()){
            liftposition = liftposition - wat;
        }

        if (liftposition > highLimit){
            liftposition = highLimit;
        }
        if (liftposition < lowerLimit){
            liftposition = lowerLimit;
        }
        lift.setPosition(liftposition);
        telemetry.addData("liftposition", liftposition);

    }

    public void lifttheMONKE() {
        double lifttop = 1;
        double liftbottom = 0;
        if (lifttheMONKEupbutton()) {
            liftposition = lifttop;
        }
        if (lifttheMONKEdownbutton()) {
            lift.setPosition(liftbottom);
            liftposition = liftbottom;
        }
        lift.setPosition(liftposition);
    }


    /*****************************/
    //Gyro Functions

    public void gyroInitialize(){
        // JWN Referenced https://stemrobotics.cs.pdx.edu/node/7265 for IMU
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;  //this was the way we did it last year, my be slow and jerky
        // parameters.mode                = BNO055IMU.SensorMode.GYRO; //This should theoretically be faster (no wasted hardware cycles to get pitch and roll) if it works. Haven’t tested yet.
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        // Retrieve and initialize the IMU. IMU to be attached to an I2C port
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        // make sure the imu gyro is calibrated before continuing.
        while (!isStopRequested() && !imu.isGyroCalibrated())
        {
            sleep(50);
            idle();
        }
    }

    public double getGyroAngle(){
        // Z axis is returned as 0 to +180 or 0 to -180 rolling to -179 or +179 when passing 180
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double heading = angles.firstAngle;
        return heading;

    }

    public void resetAngle(){   //Need to do more research on how this function works
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    }


    /*****************************/
    //Encoder Functions
   /*
   // Example from 2020 season:
    public double getLeftVerticalEncoder(){
        return rightFront.getCurrentPosition();
    }
    */

    /*****************************/
    //Autonomous Functions

    public void holonomicDriveAuto(double robotSpeed, double robotAngle, double speedOfRotation){
        double wheelVelocityFrontRight;
        double wheelVelocityBackRight;
        double wheelVelosityFrontLeft;
        double wheelVelosityBackLeft;

        robotAngle = robotAngle + Math.toRadians(0);

        wheelVelocityFrontRight = robotSpeed*Math.sin(robotAngle+(Math.PI/4))-speedOfRotation;
        wheelVelosityFrontLeft = robotSpeed*Math.cos(robotAngle+(Math.PI/4))+speedOfRotation;
        wheelVelocityBackRight = robotSpeed*Math.cos(robotAngle+(Math.PI/4))-speedOfRotation;
        wheelVelosityBackLeft = robotSpeed*Math.sin(robotAngle+(Math.PI/4))+speedOfRotation;

        rightFront.setPower(wheelVelocityFrontRight);
        leftFront.setPower(wheelVelosityFrontLeft);
        rightBack.setPower(wheelVelocityBackRight);
        leftBack.setPower(wheelVelosityBackLeft);

        telemetry.addData("LF speed ",wheelVelosityFrontLeft);
        telemetry.addData("RF speed ", wheelVelocityFrontRight);
        telemetry.addData("LB speed ",wheelVelosityBackLeft);
        telemetry.addData("RB speed ",wheelVelocityBackRight);
    }

    public void rotateToAngle (double targetAngle){
        double speed = .4;

        if (targetAngle > 0) {
            //turn right
            while (getGyroAngle()<targetAngle){
                holonomicDriveAuto(0, 0, speed);

                telemetry.addData("Target Angle: ",targetAngle);
                telemetry.addData("Current Angle",getGyroAngle());
                telemetry.update();
            }
        }
        else {
            //turn left
            while (getGyroAngle()>targetAngle){
                holonomicDriveAuto(0, 0, -speed);

                telemetry.addData("Target Angle: ",targetAngle);
                telemetry.addData("Current Angle",getGyroAngle());
                telemetry.update();
            }
        }
        stopDrive();

    }

    public void autoRotisserie(){
        duckWheel.setPower(1);
    }

    public void autoRotisserieStop(){
        duckWheel.setPower(0);
    }




}



