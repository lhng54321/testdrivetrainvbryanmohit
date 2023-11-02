// Copyright (c) 2022 Team 303

package frc.robot;
import com.revrobotics.CANSparkMax.IdleMode;


public final class RobotMap {

	public static final class DrivebaseConstants {

		/* CAN IDs of the Motors on the Drive Base */
		public static final int FRONT_LEFT_SPARK_ID = 18;
		public static final int FRONT_RIGHT_SPARK_ID = 5;
		public static final int BACK_LEFT_SPARK_ID = 6;
		public static final int BACK_RIGHT_SPARK_ID = 7;
		public static final boolean LEFT_SPARK_INVERTED = true;
		public static final boolean RIGHT_SPARK_INVERTED = true;
		public static final IdleMode BRAKE = IdleMode.kBrake;
    }

	public static final class ControllerConstants {
		public static final int CONTROLLER_ID = 1;
		public static final int LEFT_CONT_ID = 0;
		public static final int RIGHT_CONT_ID = 1;
	}


}
