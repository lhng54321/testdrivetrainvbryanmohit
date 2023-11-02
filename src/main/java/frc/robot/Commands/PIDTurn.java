package frc.robot.Commands;

import frc.robot.Robot;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class PIDTurn extends CommandBase{
    
    private PIDController pid;
    public static final double kP = 0.1;
    public static final double kI = 0;
    public static final double kD = 0.1;
    private double setpoint;

    public PIDTurn(double setpoint) {
       
        pid = new PIDController(kP, kI, kD);
        pid.setSetpoint(setpoint);
        this.setpoint = setpoint;
        pid.enableContinuousInput(-180,180);
        pid.setTolerance(0.5);

        SmartDashboard.putData("PID Controller", pid);

        addRequirements(Robot.getDrivebase());

    }

    @Override
    public void execute() {
        double output = pid.calculate((Robot.getNavX().getAngle()));
        Robot.getDrivebase().arcadeDrive(0, output);
        Robot.getDrivebase().updateTab(output, setpoint, setpoint-(Robot.getNavX().getAngle()%360));
    }

    @Override
    public boolean isFinished() {
        return pid.atSetpoint();
    }

}
