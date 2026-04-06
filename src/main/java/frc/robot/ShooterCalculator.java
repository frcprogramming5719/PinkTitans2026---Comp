package frc.robot;

/*
 * Model (cubic polynomial fit over minimum-RPS selections per distance):
 *
 *   angle(d) = c0 + c1*d + c2*d² + c3*d³   (R² = 0.758)
 *   rps(d)   = k0 + k1*d + k2*d² + k3*d³   (R² = 0.893)
 *
 * NOTE: The angle polynomial has moderate fit quality because the hood angle
 * resets lower each time a higher RPS band takes over at longer distances.
 * For highest accuracy, use calculateLookup() which linearly interpolates
 * directly from the calibration table.
 *
 * Valid range: 15–150 inches from the target.
 *
 * Calibration data (distance -> minimum viable RPS, hood angle):
 *   15 in  -> 34 RPS, 0.00°
 *   30 in  -> 43 RPS, 0.00°
 *   45 in  -> 43 RPS, 0.28°
 *   60 in  -> 43 RPS, 0.45°
 *   75 in  -> 43 RPS, 0.65°
 *   90 in  -> 43 RPS, 0.95°
 *  105 in  -> 52 RPS, 0.47°
 *  120 in  -> 52 RPS, 0.49°
 *  135 in  -> 52 RPS, 0.50°
 *  150 in  -> 61 RPS, 0.37°
 */
public class ShooterCalculator {

    // ---------------------------------------------------------------
    // Polynomial coefficients
    // ---------------------------------------------------------------

    // Hood angle: a(d) = c0 + c1*d + c2*d² + c3*d³
    private static final double ANGLE_C0 = -3.67333333e-01;
    private static final double ANGLE_C1 =  1.89821290e-02;
    private static final double ANGLE_C2 = -6.98005698e-05;
    private static final double ANGLE_C3 = -1.74393508e-07;

    // Shooter RPS: r(d) = k0 + k1*d + k2*d² + k3*d³
    private static final double RPS_K0 =  3.01000000e+01;
    private static final double RPS_K1 =  4.60722611e-01;
    private static final double RPS_K2 = -5.29137529e-03;
    private static final double RPS_K3 =  2.38280238e-05;

    // Valid calibration range (inches)
    public static final double MIN_DISTANCE_IN = 15.0;
    public static final double MAX_DISTANCE_IN = 150.0;

    // ---------------------------------------------------------------
    // Lookup table (for calculateLookup — higher accuracy)
    // Each row: { distanceInches, rps, angle }
    // ---------------------------------------------------------------
    private static final double[][] LOOKUP_TABLE = {
        {  15.0, 34.0, 0.00 },
        {  30.0, 43.0, 0.00 },
        {  45.0, 43.0, 0.28 },
        {  60.0, 43.0, 0.45 },
        {  75.0, 43.0, 0.65 },
        {  90.0, 43.0, 0.95 },
        { 105.0, 52.0, 0.47 },
        { 120.0, 52.0, 0.49 },
        { 135.0, 52.0, 0.50 },
        { 150.0, 61.0, 0.37 },
    };

    // ---------------------------------------------------------------
    // Polynomial API
    // ---------------------------------------------------------------

    /**
     * Calculates the required hood angle for a given distance using
     * a cubic polynomial fit.
     *
     * @param distanceInches  Distance to the target in inches.
     * @return                Hood angle (degrees, same units as calibration data).
     */
    public static double getAngle(double distanceInches) {
        double d = distanceInches;
        double angle = ANGLE_C0
                     + ANGLE_C1 * d
                     + ANGLE_C2 * d * d
                     + ANGLE_C3 * d * d * d;
        return Math.max(0.0, angle);
    }

    /**
     * Calculates the required shooter speed in rotations per second (RPS)
     * for a given distance using a cubic polynomial fit.
     *
     * @param distanceInches  Distance to the target in inches.
     * @return                Shooter speed (RPS).
     */
    public static double getRPS(double distanceInches) {
        double d = distanceInches;
        double rps = RPS_K0
                   + RPS_K1 * d
                   + RPS_K2 * d * d
                   + RPS_K3 * d * d * d;
        return Math.max(0.0, rps);
    }

    /**
     * Returns both the hood angle and shooter RPS for a given distance
     * using cubic polynomial models.
     *
     * @param distanceInches  Distance to the target in inches.
     * @return                ShooterState with angle and rps fields.
     */
    public static ShooterState calculate(double distanceInches) {
        return new ShooterState(getAngle(distanceInches), getRPS(distanceInches));
    }

    // ---------------------------------------------------------------
    // Lookup-table API (recommended for higher accuracy)
    // ---------------------------------------------------------------

    /**
     * Returns both the hood angle and shooter RPS for a given distance
     * by linearly interpolating between calibration data points.
     *
     * This method is more accurate than the polynomial {@link #calculate}
     * because it exactly reproduces calibration values and only interpolates
     * between adjacent measured points.
     *
     * @param distanceInches  Distance to the target in inches.
     * @return                ShooterState with angle and rps fields.
     */
    public static ShooterState calculateLookup(double distanceInches) {

        // Clamp to table bounds
        double d = Math.max(LOOKUP_TABLE[0][0],
                   Math.min(LOOKUP_TABLE[LOOKUP_TABLE.length - 1][0], distanceInches));

        // Find surrounding rows
        for (int i = 0; i < LOOKUP_TABLE.length - 1; i++) {
            double d0 = LOOKUP_TABLE[i][0];
            double d1 = LOOKUP_TABLE[i + 1][0];

            if (d >= d0 && d <= d1) {
                double t = (d - d0) / (d1 - d0);   // 0..1

                double rps   = lerp(LOOKUP_TABLE[i][1], LOOKUP_TABLE[i + 1][1], t);
                double angle = lerp(LOOKUP_TABLE[i][2], LOOKUP_TABLE[i + 1][2], t);

                return new ShooterState(angle, rps);
            }
        }

        // Exact match at last entry
        double[] last = LOOKUP_TABLE[LOOKUP_TABLE.length - 1];
        return new ShooterState(last[2], last[1]);
    }

    // ---------------------------------------------------------------
    // Helpers
    // ---------------------------------------------------------------

    private static double lerp(double a, double b, double t) {
        return a + t * (b - a);
    }

    // private static void warnIfOutOfRange(double distanceInches) {
    //     if (distanceInches < MIN_DISTANCE_IN || distanceInches > MAX_DISTANCE_IN) {
    //         System.err.printf(
    //             "[ShooterCalculator] WARNING: distance %.1f in is outside calibrated range [%.0f, %.0f].%n",
    //             distanceInches, MIN_DISTANCE_IN, MAX_DISTANCE_IN
    //         );
    //     }
    // }

    // ---------------------------------------------------------------
    // ShooterState
    // ---------------------------------------------------------------

    public static class ShooterState {
        /** Hood angle (degrees) */
        public final double angle;
        /** Shooter speed (rotations per second) */
        public final double rps;

        public ShooterState(double angle, double rps) {
            this.angle = angle;
            this.rps   = rps;
        }

        @Override
        public String toString() {
            return String.format("ShooterState { angle=%.4f deg, rps=%.4f RPS }", angle, rps);
        }
    }

/*
    public static void main(String[] args) {
        System.out.println("=== Polynomial model ===");
        System.out.printf("%-14s| %-10s| %s%n", "Distance (in)", "Angle (deg)", "RPS");
        System.out.println("--------------|-----------|--------");
        for (int d = 15; d <= 150; d += 15) {
            ShooterState s = calculate(d);
            System.out.printf("%-14d| %-11.4f| %.4f%n", d, s.angle, s.rps);
        }

        System.out.println("\n=== Lookup-table model ===");
        System.out.printf("%-14s| %-10s| %s%n", "Distance (in)", "Angle (deg)", "RPS");
        System.out.println("--------------|-----------|--------");
        for (int d = 15; d <= 150; d += 15) {
            ShooterState s = calculateLookup(d);
            System.out.printf("%-14d| %-11.4f| %.4f%n", d, s.angle, s.rps);
        }
    }
*/
}
