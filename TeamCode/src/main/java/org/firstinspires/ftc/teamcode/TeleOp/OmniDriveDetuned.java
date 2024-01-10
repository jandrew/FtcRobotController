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

package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.libs.XYToAngle;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;

/*
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="OmniDriveDetuned", group="Basic")
//@Disabled
public class OmniDriveDetuned extends OpMode
{
    // Declare OpMode members.
    double detuningFactor = 2;
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeftDrive = null;
    private double frontLeftPower = 0;
    private DcMotor frontRightDrive = null;
    private double frontRightPower = 0;
    private DcMotor backLeftDrive = null;
    private double backLeftPower = 0;
    private DcMotor backRightDrive = null;
    private double backRightPower = 0;
    private XYToAngle inputController = new XYToAngle();
//    PowerToMotor frontLeftPower = new PowerToMotor(-45.0, "front_left_drive");
//    PowerToMotor frontRightPower = new PowerToMotor(45.0, "front_right_drive");
//    PowerToMotor backLeftPower = new PowerToMotor(-135.0, "back_left_drive");
//    PowerToMotor backRightPower = new PowerToMotor(135.0, "back_right_drive");
    Boolean motorsConnected = false;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        try {
            // Initialize the hardware variables. Note that the strings used here as parameters
            // to 'get' must correspond to the names assigned during the robot configuration
            // step (using the FTC Robot Controller app on the phone).
            frontLeftDrive = hardwareMap.get(DcMotor.class, "front_left_drive");
            frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_drive");
            backLeftDrive = hardwareMap.get(DcMotor.class, "back_left_drive");
            backRightDrive = hardwareMap.get(DcMotor.class, "back_right_drive");

            // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
            // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
            // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
            frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
            backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
            frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
            backRightDrive.setDirection(DcMotor.Direction.FORWARD);
            motorsConnected = true;
            // Tell the driver that initialization is complete.
            telemetry.addData("Status", "Motors connected");
        }catch (Exception e){
            System.out.println("Unable to connect motors: " + e);
        }

    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        // Setup a variable for each drive wheel to save power level for telemetry

        // Choose to drive using either Tank Mode, or POV Mode
        // Comment out the method that's not used.  The default below is POV.

        // POV Mode uses left stick to go forward, and right stick to turn.
        // - This uses basic math to combine motions and is easier to drive straight.
        double y_drive = scaleInput(-gamepad1.left_stick_y);
        double x_drive = scaleInput(gamepad1.left_stick_x);
        telemetry.addData("controller 1 left stick", "X: (%.2f), Y: (%.2f)", x_drive, y_drive);
        inputController.setXY(y_drive, x_drive);
        telemetry.addData("direction and power", "angle: (%.2f), power: (%.2f)",
                inputController.getAngle(), inputController.getPower());
//        telemetry.addData("Angle to motor", "front left: (%.2f)",
//                inputController.getAngle() - -45);
//        telemetry.addData("power to motor", "power: (%.2f)",
//                inputController.getPower());
        double turn  =  scaleInput(gamepad1.right_stick_x);
        telemetry.addData("Turn", "power: (%.2f)", turn);


        // Send calculated power to wheels
//        frontLeftPower.setPowerAngle(inputController.getPower(), inputController.getAngle());
//        frontRightPower.setPowerAngle(inputController.getPower(), inputController.getAngle());
//        backLeftPower.setPowerAngle(inputController.getPower(), inputController.getAngle());
//        backRightPower.setPowerAngle(inputController.getPower(), inputController.getAngle());
//        telemetry.addData("Motor speed",
//                "fl speed: (%.2f), fr speed: (%.2f)",
//                frontLeftPower.getPower(),
//                frontRightPower.getPower());
//        telemetry.addData("Motor speed",
//                "bl speed: (%.2f), br speed: (%.2f)",
//                backLeftPower.getPower(),
//                backRightPower.getPower());
        frontLeftPower = y_drive + x_drive + turn;
        frontRightPower = y_drive - x_drive - turn;
        backLeftPower = y_drive - x_drive + turn;
        backRightPower = y_drive + x_drive - turn;
        ArrayList<Double> numbers = new ArrayList<>(Arrays.asList(frontLeftPower, frontRightPower, backLeftPower, backRightPower));
        Double max = Collections.max(numbers);
        if (max > 1) {
            frontLeftPower = frontLeftPower/max;
            frontRightPower = frontRightPower/max;
            backLeftPower = backLeftPower/max;
            backRightPower = backRightPower/max;
        }
        // Detuning step
        frontLeftPower = frontLeftPower/detuningFactor;
        frontRightPower = frontRightPower/detuningFactor;
        backLeftPower = backLeftPower/detuningFactor;
        backRightPower = backRightPower/detuningFactor;

        telemetry.addData("Motor speed",
                "fl speed: (%.2f), fr speed: (%.2f)", frontLeftPower, frontRightPower);
        telemetry.addData("Motor speed",
                "bl speed: (%.2f), br speed: (%.2f)", backLeftPower, backRightPower);
        if (motorsConnected) {
            frontLeftDrive.setPower(frontLeftPower);
            frontRightDrive.setPower(frontRightPower);
            backLeftDrive.setPower(backLeftPower);
            backRightDrive.setPower(backRightPower);
        }

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
    /***
     * scale the input down (if scaleForSpeed) set so that the stick is more sensitive
     * @param input
     * @return
     */
    private Double scaleInput(Float input){
        Double output = input.doubleValue();
        int input_sense = 1;
        if (input < 0){
            input_sense = -1;
        }
        output = output * output * input_sense;
        return output;
    }

}
