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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.HardwareMecanum;

/**
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 * The code is structured as a LinearOpMode
 *
 * This particular OpMode executes a POV Game style Teleop for a PushBot
 * In this mode the left stick moves the robot FWD and back, the Right stick turns left and right.
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Mecanum: Teleop POV", group="Mecanum")
//@Disabled
public class MecanumTeleop_Linear extends LinearOpMode {

    /* Declare OpMode members. */
    HardwareMecanum robot           = new HardwareMecanum();   // Use our mecanum hardware
                                                               // could also use HardwarePushbotMatrix class.

    // *** added 11/11
    double          clawOffset      = 0;                       // Servo mid position
    final double    CLAW_SPEED      = 0.04 ;                   // sets rate to move servo
    // *** end added 11/11

    @Override
    public void runOpMode() {
        double left_front;
        double left_rear;
        double right_front;
        double right_rear;
        double drive;
        double strafe;
        double turn;
        double max;
        double arm_lift;

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Run wheels in POV mode (note: The joystick goes negative when pushed forwards, so negate it)
            // In this mode the Left stick moves the robot fwd and back, the Right stick turns left and right.
            // This way it's also easy to just drive straight, or just turn.
            drive = -gamepad1.left_stick_y;
            strafe = gamepad1.left_stick_x;
            turn  =  gamepad1.right_stick_x;

            // Combine drive and turn for blended motion.
            left_front  = drive + turn - strafe;
            right_front = drive - turn + strafe;
            left_rear = drive + turn + strafe;
            right_rear = drive - turn - strafe;

            // Normalize the values so neither exceed +/- 1.0
            max = Math.max(Math.max(Math.abs(left_front), Math.abs(left_rear)),
                           Math.max(Math.abs(right_front), Math.abs(right_rear)));
            if (max > 1.0)
            {
                left_front /= max;
                left_rear /= max;
                right_front /= max;
                right_rear /= max;
            }

            // Output the safe values to the motor drives.
            robot.leftFrontDrive.setPower(left_front);
            robot.rightFrontDrive.setPower(right_front);
            robot.leftRearDrive.setPower(left_rear);
            robot.rightRearDrive.setPower(right_rear);

            // Use gamepad right stick to open and close the claw
            if (gamepad2.right_stick_x>0)
                clawOffset += CLAW_SPEED;
            else if (gamepad2.right_stick_x<0)
                clawOffset -= CLAW_SPEED;

            // Move both servos to new position.  Assume servos are mirror image of each other.
            clawOffset = Math.max(-0.5,Math.min(clawOffset,+0.5));//Range.clip(clawOffset, -0.5, 0.5);
            robot.leftClaw.setPosition(robot.MID_SERVO + clawOffset);
            robot.rightClaw.setPosition(robot.MID_SERVO - clawOffset);



            // Use gamepad2 left stick to lift arm
            arm_lift = gamepad2.left_stick_y;
            if (Math.abs(arm_lift)>1.0) {
                arm_lift = Math.signum(arm_lift);
            }
            if (robot.armLowerStop.isPressed() && arm_lift < 0) {
                arm_lift = 0;
            }
            robot.liftMotor.setPower(arm_lift);


            // Send telemetry message to signify robot running;
            telemetry.addData("claw",  "Offset = %.2f", clawOffset);
            telemetry.addData("arm_lift",  "%.2f", arm_lift);
            telemetry.addData("left_front",  "%.2f", left_front);
            telemetry.addData("right_front", "%.2f", right_front);
            telemetry.addData("left_rear",  "%.2f", left_rear);
            telemetry.addData("right_rear", "%.2f", right_rear);
            telemetry.addData("armLowerStop", "Pressed = %d", robot.armLowerStop.isPressed());
            telemetry.update();

            // Pace this loop so jaw action is reasonable speed.
            sleep(20);
        }
    }
}
