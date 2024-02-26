package frc.robot.constants;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;

public class DrivebaseConstants {
    public static class BACK_LEFT {
        public static final double KS = 0.29966;
        public static final double KV = 1.7297;
        public static final double KA = 0.21697;
    }
    public static class BACK_RIGHT {
        public static final double KS = 0.44203;
        public static final double KV = 1.6083;
        public static final double KA = 0.23797;
    }
    public static class FRONT_LEFT {
        public static final double KS = 0.44112;
        public static final double KV = 1.6071;
        public static final double KA = 0.17817;
    }
    public static class FRONT_RIGHT {
        public static final double KS = 0.45984;
        public static final double KV = 1.6300;
        public static final double KA = 0.17909;
    }

    public static final SimpleMotorFeedforward BACK_LEFT_FEEDFORWARD = new SimpleMotorFeedforward(BACK_LEFT.KS, BACK_LEFT.KV, BACK_LEFT.KA);
    public static final SimpleMotorFeedforward BACK_RIGHT_FEEDFORWARD = new SimpleMotorFeedforward(BACK_RIGHT.KS, BACK_RIGHT.KV, BACK_RIGHT.KA);
    public static final SimpleMotorFeedforward FRONT_LEFT_FEEDFORWARD = new SimpleMotorFeedforward(FRONT_LEFT.KS, FRONT_LEFT.KV, FRONT_LEFT.KA);
    public static final SimpleMotorFeedforward FRONT_RIGHT_FEEDFORWARD = new SimpleMotorFeedforward(FRONT_RIGHT.KS, FRONT_RIGHT.KV, FRONT_RIGHT.KA);
}
