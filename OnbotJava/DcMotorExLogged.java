package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;

public class DcMotorExLogged {
    
    public DcMotorEx motor = null;
    private DataLogger logger = null;
    
    // constructor
    public DcMotorExLogged (DcMotorEx m, String logFileName) {
        motor = m;
        logger = new DataLogger(logFileName);
        logger.addField("target");
        logger.addField("actual");
        logger.newLine();
    }
    
    public void setVelocity (double v) {
        motor.setVelocity(v);
        logger.addField(v);
        logger.addField(motor.getVelocity());
        logger.newLine();
    }

}
    
