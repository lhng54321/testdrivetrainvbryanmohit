// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistribution;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap.ControllerConstants;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotMap.DrivebaseConstants;
import frc.robot.Subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import java.lang.ModuleLayer.Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.RobotMap;
import frc.robot.Commands.DefaultDrive;
import frc.robot.Commands.PIDTurn;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

import com.kauailabs.navx.frc.AHRS;

/**
 * This is a demo program showing the use of the DifferentialDrive class, specifically it contains
 * the code necessary to operate a robot with tank drive.
 */
public class Robot extends LoggedRobot {

	/* RoboRio Sensors */
	private static final AHRS navX = new AHRS();

	public static final Logger logger = Logger.getInstance(); 

	public static AHRS getNavX() {
		return navX;
	}

	private static final DriveSubsystem tank = new DriveSubsystem();	

	public static DriveSubsystem getDrivebase() {
			return tank;
	}

	public static final XboxController XBOX_CONTROLLER = new XboxController(0);

	public void robotInit() {
		
		tank.setDefaultCommand(new DefaultDrive());

		new Trigger(() -> {return XBOX_CONTROLLER.getXButtonPressed();}).onTrue(new InstantCommand(tank::resetPose));
		new Trigger(() -> {return XBOX_CONTROLLER.getYButtonPressed();}).onTrue(Commands.race(Commands.waitSeconds(3),new PIDTurn(90)));



		logger.recordMetadata("DifferentialDriveSim", "MyProject");

		if (isReal()) {
			logger.addDataReceiver(new WPILOGWriter(""));
			logger.addDataReceiver(new NT4Publisher());
			new PowerDistribution(1, ModuleType.kRev);
		} else {
			setUseTiming(false);
			String logPath = LogFileUtil.findReplayLog();
			logger.setReplaySource(new WPILOGReader(logPath));
			logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
		}

		logger.start();

	}

  	@Override
	public void robotPeriodic() {

		/*
		 * Runs the Scheduler. This is responsible for polling buttons, adding
		 * newly-scheduled
		 * commands, running already-scheduled commands, removing finished or
		 * interrupted commands,
		 * and running subsystem periodic() methods. This must be called from the
		 * robot's periodic
		 * block in order for anything in the Command-based framework to work.
		 */
		CommandScheduler.getInstance().run();
		//System.out.print("Left Joystick: "); System.out.println(-XBOX_CONTROLLER.getLeftY());
		//System.out.print("Right Joystick: "); System.out.println(-XBOX_CONTROLLER.getRightY());
		
	}
}
