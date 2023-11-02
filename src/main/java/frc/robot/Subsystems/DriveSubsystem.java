package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import frc.robot.RobotMap.DrivebaseConstants;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap.DrivebaseConstants;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.networktables.GenericEntry;
import frc.robot.Commands.PIDTurn;
import com.kauailabs.navx.frc.AHRS;



public class DriveSubsystem extends SubsystemBase{

    private final CANSparkMax m_frontLeftMotor;
    private final CANSparkMax m_frontRightMotor;
    private final CANSparkMax m_backLeftMotor;
    private final CANSparkMax m_backRightMotor;
    private final MotorController leftSideGroup;
    private final MotorController rightSideGroup;
    private final DifferentialDrive drive;
    private final DifferentialDriveOdometry dDriveOdometry; 
    private final RelativeEncoder left;
    private final RelativeEncoder right;
    private final ShuffleboardTab DBSTab = Shuffleboard.getTab("Drive Base Subsystem");
    private final GenericEntry setpoint;
    private final GenericEntry output;
    private final GenericEntry error;
    private final GenericEntry angle;


    private Pose2d m_pose; 

    public ShuffleboardTab getTab() {
        return DBSTab;
    }

    public DifferentialDrive getDrive() {
        return drive;
    }

    public DriveSubsystem()
     { 
        m_frontLeftMotor = new CANSparkMax(DrivebaseConstants.FRONT_LEFT_SPARK_ID, MotorType.kBrushed);
        m_frontRightMotor = new CANSparkMax(DrivebaseConstants.FRONT_RIGHT_SPARK_ID, MotorType.kBrushless);
        m_backLeftMotor = new CANSparkMax(DrivebaseConstants.BACK_LEFT_SPARK_ID, MotorType.kBrushless);
        m_backRightMotor = new CANSparkMax(DrivebaseConstants.BACK_RIGHT_SPARK_ID, MotorType.kBrushless);

        leftSideGroup = new MotorControllerGroup(m_frontLeftMotor, m_backLeftMotor);
        rightSideGroup = new MotorControllerGroup(m_frontRightMotor, m_backRightMotor); 
        
        m_frontLeftMotor.setIdleMode(DrivebaseConstants.BRAKE);
        m_frontRightMotor.setIdleMode(DrivebaseConstants.BRAKE);
        m_backLeftMotor.setIdleMode(DrivebaseConstants.BRAKE);
        m_backRightMotor.setIdleMode(DrivebaseConstants.BRAKE);

        drive = new DifferentialDrive(leftSideGroup, rightSideGroup);
        
        right = m_frontRightMotor.getEncoder();
        left = m_backLeftMotor.getEncoder();~

        right.setPositionConversionFactor(1/4.67);
        left.setPositionConversionFactor(1/4.67);

        dDriveOdometry = new DifferentialDriveOdometry(new Rotation2d(Robot.getNavX().getAngle()),
          left.getPosition(), 
          right.getPosition(), 
          new Pose2d(0, 0, new Rotation2d()));
        
        output = DBSTab.add("PID Output", 0).getEntry();
        setpoint = DBSTab.add("PID Setpoint", 0).getEntry(); 
        error = DBSTab.add("Error", 0).getEntry();
        angle = DBSTab.add("Angle", 0).getEntry();
     }

    public void updateTab(double output, double setpoint, double error) {
        this.output.setDouble(output);
        this.setpoint.setDouble(setpoint);
        this.error.setDouble(error);
    }

    public MotorController getLeftSideGroup() {
        return leftSideGroup;
    }

    public MotorController getRightSideGroup() {
        return rightSideGroup;
    }

    public void arcadeDrive(double arcadeDriveSpeed, double arcadeDriveRotations) {
        getDrive().arcadeDrive(arcadeDriveSpeed, arcadeDriveRotations);
    }


    public void tankDrive(double leftSpeed, double rightSpeed) {
        getDrive().tankDrive(leftSpeed, rightSpeed);
    }

    public void curvatureDrive(double xSpeed, double zRotations, boolean allowTurnInPlace) {
        getDrive().curvatureDrive(xSpeed, zRotations, allowTurnInPlace);
    }

    @Override
    public void periodic() {

        Rotation2d gyroAngle = new Rotation2d(Math.toRadians(Robot.getNavX().getAngle()));

        m_pose = dDriveOdometry.update(gyroAngle,
        left.getPosition(),
        right.getPosition());

        Robot.logger.recordOutput("Odometry", m_pose);
        
        angle.setDouble(Robot.getNavX().getAngle());

    }

    public void resetPose() {
        dDriveOdometry.resetPosition(Robot.getNavX().getRotation2d(), left.getPosition(), right.getPosition(), new Pose2d());
        Robot.getNavX().reset();
    }

}
