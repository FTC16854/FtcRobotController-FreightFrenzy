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

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

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






@TeleOp(name="OMNI FieldCentric", group="Linear Opmode")
//@Disabled
public class ChildOpModeFieldCentricOmni extends ParentOpMode{

    private DcMotor rightFront = null;
    private DcMotor rightBack = null;
    private DcMotor leftFront = null;
    private DcMotor leftBack = null;


    @Override
    public void runOpMode() {

        initialize();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        //releaseLatch();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {


            liftTheThreeUpandDown();  //jog/nudge up and down
            lifttheMONKE();           //set position to top or bottom
            duckWheelSpin();
            intakeEatr();
            fieldCentricOmni();
            //include emergency stop check in all runOpMode() functions/methods
            if(emergencyStopped()){
                break;
            }

//            getRightDistanceCM();
//            getFrontDistanceCM();
//            getLeftDistanceCM();

            telemetry.update();
        }
    }

    public void fieldCentricOmni(){
        double wheelVelocityFrontRight;
        double wheelVelocityBackRight;
        double wheelVelosityFrontLeft;
        double wheelVelosityBackLeft;

        double angleOffset = getGyroAngle() - 90 + HeadingHolder.getOffsetOfTheHeading();

        double robotSpeed = Math.hypot(left_sticky_y(), left_sticky_x());

        double robotAngle = Math.atan2(left_sticky_y(),left_sticky_x())-Math.toRadians(angleOffset);

        double speedOfRotation = -right_sticky_x()*.75;

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



}


        //TODO
        //  Hardware map
        //  Controls map
        //  Tank Drive Function

        //Encoder Stuff
        //  Odometry Wheels
        //      9192 Counts per revolution


