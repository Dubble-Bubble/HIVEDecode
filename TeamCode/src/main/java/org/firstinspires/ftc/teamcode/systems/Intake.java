package org.firstinspires.ftc.teamcode.systems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class Intake {

    private DcMotor intake, transfer;
    private Servo flapL, flapR, pto, pto2;

    public static double shotDelay = 1.2, shotTransferAllowance = 0.4;
    public static double flapActuationTime = 0.8;

    private boolean intakeActivityFlag = false;
    private boolean reverseFlag = false;

    public static double flapUp = 0.77, flapDown = 0.93, ptoEngaged = 0.15, ptoDisengaged = 0;

    public Intake(HardwareMap hardwareMap) {
        intake = hardwareMap.dcMotor.get("intake");
        transfer = hardwareMap.dcMotor.get("transfer");
        flapL = hardwareMap.servo.get("lFlap");
        flapR = hardwareMap.servo.get("rFlap");
        flapR.setDirection(Servo.Direction.REVERSE);
    }

    public void setActive(boolean flag) {
        intakeActivityFlag = flag;
    }

    public void setReverse(boolean flag) {
        reverseFlag = flag;
    }

    public void setTransfer(boolean transfer) {
        if (transfer) {
            this.transfer.setPower(-1);
        } else {
            this.transfer.setPower(0);
        }
    }
    public void setTransferSlower(boolean transfer) {
        if (transfer) {
            this.transfer.setPower(-0.8);
        } else {
            this.transfer.setPower(0);
        }
    }

    ElapsedTime shotTimer = new ElapsedTime();
    private boolean wasShooting = false;

    int shotCount = 0;

    public void update() {
        if (intakeActivityFlag) {
            intake.setPower(1);
        } else if (!reverseFlag && !intakeActivityFlag) {
            intake.setPower(0);
        } else if (reverseFlag && !intakeActivityFlag){
            intake.setPower(-1);
            transfer.setPower(1);
        }
    }

    public void setIntake(double power) {
        intake.setPower(power);
    }

    public void setFlap(double pos) {
        flapR.setPosition(pos);
        flapL.setPosition(pos);
    }

    public void setPtoEngaged(boolean engaged) {
        if (engaged) {
            pto.setPosition(0.08);
            pto2.setPosition(0.96);
        } else {
            pto.setPosition(0.2);
            pto2.setPosition(0.85);
        }
    }

}
