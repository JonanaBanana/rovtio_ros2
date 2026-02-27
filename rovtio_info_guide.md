# ROVTIO `.info` File — Complete Configuration Guide

---

## Syntax Rules

The `.info` file uses a **nested block** format. The rules are simple:

```
BlockName
{
    key   value;    This text after the semicolon is a comment
    
    NestedBlock
    {
        key   value;
    }
}
```

- Values are separated from keys by whitespace
- Everything after `;` on a line is a **comment** (the semicolon acts as comment delimiter, NOT a statement terminator)
- Blocks are opened with `{` and closed with `}`
- You can include another `.info` file with `#include "/path/to/file.info"`
- Unrecognised keys are silently ignored — safe to leave comments in

---

## Top-Level Block Structure

```
Common          { ... }         Global filter settings
Camera0         { ... }         Camera 0 intrinsics & extrinsics
Camera1         { ... }         Camera 1 intrinsics & extrinsics (if NCAM >= 2)
Init            { ... }         Initial state & covariance
ImgUpdate       { ... }         Visual/thermal feature tracking settings
Prediction      { ... }         IMU prediction noise
PoseUpdate      { ... }         External pose measurement (optional)
VelocityUpdate  { ... }         External velocity measurement (optional)
```

> Add one `CameraN` block per camera, matching your `-DROVIO_NCAM=N` build flag.

---

## `Common` Block

```
Common
{
    doVECalibration true;   Should the camera-IMU extrinsics be estimated online.
                            Set false only if you have a precise factory calibration.

    depthType 1;            Feature depth parametrization:
                            0 = normal (distance)
                            1 = inverse depth  (recommended, handles far features)
                            2 = log
                            3 = hyperbolic

    verbose false;          Print verbose filter output to stdout
}
```

---

## `CameraN` Blocks

Add one block per camera, numbered from 0. **N must match the build flag.**

```
Camera0
{
    CalibrationFile /path/to/cam0.yaml;     Absolute path to camera intrinsics YAML
                                            (ROS camera_info or OpenCV format) 

    ; Rotation: IMU frame → Camera frame (Hamilton quaternion, unit norm)
    qCM_x  0.0;
    qCM_y  0.0;
    qCM_z  0.0;
    qCM_w  1.0;

    ; Translation: vector from IMU origin to camera origin, expressed in IMU frame [m]
    MrMC_x  0.0;
    MrMC_y  0.0;
    MrMC_z  0.0;
}

Camera1
{
    CalibrationFile /path/to/cam1.yaml;

    qCM_x  0.0;
    qCM_y  0.0;
    qCM_z  0.0;
    qCM_w  1.0;

    MrMC_x  0.05;
    MrMC_y  0.0;
    MrMC_z  0.0;
}
```

### Extrinsics Notes

- **Hamilton convention** — same as ROS. If your calibration tool (e.g. Kalibr) outputs JPL, negate `x`, `y`, `z` and keep `w`.
- **`MrMC` is in the IMU body frame**, not the camera frame. It is the vector pointing from the IMU origin to the camera origin, written in IMU coordinates.
- If you only have a rough estimate, set `doVECalibration true` in `Common` and the filter will refine these values online.

---

## `Init` Block

Sets the starting state and how confident the filter is in it.

```
Init
{
    State
    {
        ; Initial position: world origin to IMU, in world frame [m]
        pos_0  0;
        pos_1  0;
        pos_2  0;

        ; Initial velocity: IMU body frame [m/s]
        vel_0  0;
        vel_1  0;
        vel_2  0;

        ; Initial accelerometer bias [m/s^2]
        acb_0  0;
        acb_1  0;
        acb_2  0;

        ; Initial gyroscope bias [rad/s]
        gyb_0  0;
        gyb_1  0;
        gyb_2  0;

        ; Initial attitude: IMU to world (Hamilton quaternion)
        att_x  0;
        att_y  0;
        att_z  0;
        att_w  1;
    }

    Covariance
    {
        ; Position uncertainty [m^2] — small if you know the start position
        pos_0  0.0001;
        pos_1  0.0001;
        pos_2  0.0001;

        ; Velocity uncertainty [m^2/s^2] — large: we don't know initial velocity
        vel_0  1.0;
        vel_1  1.0;
        vel_2  1.0;

        ; Accelerometer bias uncertainty [m^2/s^4]
        acb_0  4e-4;
        acb_1  4e-4;
        acb_2  4e-4;

        ; Gyroscope bias uncertainty [rad^2/s^2]
        gyb_0  3e-4;
        gyb_1  3e-4;
        gyb_2  3e-4;

        ; Attitude uncertainty [rad^2] — 0.1 ≈ ±18°, reasonable unless levelled
        att_0  0.1;
        att_1  0.1;
        att_2  0.1;

        ; Extrinsics uncertainty (used only if doVECalibration true)
        vep  0.0001;    Linear extrinsics [m^2]
        vea  0.01;      Rotational extrinsics [rad^2]
    }
}
```

---

## `ImgUpdate` Block

Controls feature detection, tracking, and the visual/thermal update step.

```
ImgUpdate
{
    ; ----- Update solver -----
    updateVecNormTermination  1e-4;     Convergence threshold for iterated EKF
    maxNumIteration           20;       Max IEKF iterations per update

    ; ----- Patch tracking -----
    doPatchWarping      true;           Warp patches to compensate for rotation/perspective
    useDirectMethod     true;           true = photometric error (recommended)
                                        false = reprojection error
    startLevel  2;                      Coarsest pyramid level used (0 = finest)
    endLevel    1;                      Finest pyramid level used

    ; ----- Visualisation -----
    doFrameVisualisation  true;         Overlay feature tracks on output image
    visualizePatches      false;        Show separate patch debug window

    ; ----- Update noise -----
    UpdateNoise
    {
        pix  2;     Reprojection noise [rad^2] (used when useDirectMethod false)
                    Roughly (1/focal_length)^2 ≈ 1/400^2 for a 400px focal length
        int  400;   Photometric noise [intensity^2] (used when useDirectMethod true)
                    Increase if images are noisy; decrease for clean images
    }

    ; ----- Feature initialisation -----
    initCovFeature_0  0.5;      Initial depth uncertainty (relative) [m^2/m^2]
    initCovFeature_1  1e-5;     Initial bearing uncertainty x [rad^2]
    initCovFeature_2  1e-5;     Initial bearing uncertainty y [rad^2]
    initDepth         0.5;      Initial inverse depth value

    ; ----- Feature detection -----
    nDetectionBuckets      100;     Grid cells for spatial feature spreading
    startDetectionTh       0.8;     Min relative Shi-Tomasi score to accept candidate (0–1)
    scoreDetectionExponent 0.25;    Score weighting exponent for bucketing
    penaltyDistance        100;     Distance within which candidates penalise each other [px]
    zeroDistancePenalty    100;     Penalty for zero-distance candidates
    minRelativeSTScore     0.75;    Min relative ST score for new patch extraction
    minAbsoluteSTScore     5.0;     Min absolute ST score for new patch extraction
    fastDetectionThreshold 5;       FAST corner threshold

    ; ----- Feature lifetime -----
    statLocalQualityRange         10;     Frames for local quality window
    statLocalVisibilityRange      100;    Frames for local visibility window
    statMinGlobalQualityRange     100;    Min frames for global quality estimate
    trackingUpperBound            0.9;    Quality threshold triggering new detections
    trackingLowerBound            0.8;    Quality threshold for forced new detections
    minTrackedAndFreeFeatures     0.75;   Min fraction of features tracked or free
    removalFactor                 1.1;    Aggressiveness of forced removal
    minTimeBetweenPatchUpdate     1.0;    Min seconds between patch re-extractions [s]
    patchRejectionTh              50.0;   Max mean intensity error before rejecting [intensity]

    ; ----- Alignment -----
    alignConvergencePixelRange    10;     Assumed convergence range per level [px]
    alignCoverageRatio            2;      Sigma coverage for adaptive alignment
    alignMaxUniSample             1;      Alignment seeds per side (total = 2n+1)
    alignmentHuberNormThreshold   10;     Huber norm threshold for intensity error
    alignmentGaussianWeightingSigma -1;   Gaussian weighting width (-1 = disabled)
    alignmentGradientExponent     0.0;    Gradient-based residual weighting exponent
    useIntensityOffsetForAlignment  true; Account for intensity offset between patches
    useIntensitySqewForAlignment    true; Account for intensity scale between patches

    ; ----- Multi-camera settings -----
    useCrossCameraMeasurements    true;   Use cross-camera feature measurements
    doStereoInitialization        true;   Use stereo match for feature depth init
    noiseGainForOffCamera         10.0;   Extra noise multiplier for non-primary camera

    MahalanobisTh                 9.21;   Outlier rejection threshold (chi^2, 2 dof, 99%)

    removeNegativeFeatureAfterUpdate true;
    maxUncertaintyToDepthRatioForDepthInitialization 0.3;
    discriminativeSamplingDistance   0.02;
    discriminativeSamplingGain       1.1;

    ; ----- Motion detection -----
    MotionDetection
    {
        isEnabled                       0;      1 = enable inertial motion detection
        rateOfMovingFeaturesTh          0.5;    Fraction of moving features for motion trigger
        pixelCoordinateMotionTh         1.0;    Per-feature motion threshold [px]
        minFeatureCountForNoMotionDetection 5;  Min features needed for detection
    }

    ; ----- Zero-velocity update -----
    ZeroVelocityUpdate
    {
        UpdateNoise
        {
            vel_0  0.01;    X zero-velocity noise [m^2/s^2]
            vel_1  0.01;    Y zero-velocity noise [m^2/s^2]
            vel_2  0.01;    Z zero-velocity noise [m^2/s^2]
        }
        MahalanobisTh0      7.69;   Outlier threshold
        minNoMotionTime     1.0;    Seconds of no-motion before update triggers [s]
        isEnabled           0;      1 = enable (requires MotionDetection.isEnabled 1)
    }
}
```

---

## `Prediction` Block

IMU noise values. These come from your **IMU datasheet** noise spectral densities.

```
Prediction
{
    PredictionNoise
    {
        ; Position process noise [m^2/s]
        pos_0  1e-4;
        pos_1  1e-4;
        pos_2  1e-4;

        ; Velocity process noise [m^2/s^3]
        ; = (accelerometer noise density)^2
        ; e.g. BMI088: 175 µg/√Hz → (175e-6 * 9.81)^2 ≈ 3e-6
        vel_0  4e-6;
        vel_1  4e-6;
        vel_2  4e-6;

        ; Accelerometer bias random walk [m^2/s^5]
        acb_0  1e-8;
        acb_1  1e-8;
        acb_2  1e-8;

        ; Gyroscope bias random walk [rad^2/s^3]
        ; = (gyroscope bias instability noise density)^2
        gyb_0  3.8e-7;
        gyb_1  3.8e-7;
        gyb_2  3.8e-7;

        ; Attitude process noise [rad^2/s]
        ; = (gyroscope noise density)^2
        att_0  7.6e-7;
        att_1  7.6e-7;
        att_2  7.6e-7;

        ; Extrinsics drift noise (keep small — extrinsics change slowly)
        vep  1e-8;    Linear extrinsics [m^2/s]
        vea  1e-8;    Rotational extrinsics [rad^2/s]

        ; Feature noise
        dep  0.0001;    Depth [m^2/s]
        nor  0.00001;   Bearing vector [rad^2/s]
    }

    MotionDetection
    {
        inertialMotionRorTh  0.1;   Rotational rate threshold [rad/s]
        inertialMotionAccTh  0.5;   Acceleration threshold [m/s^2]
    }
}
```

### How to get IMU noise values from a datasheet

| Parameter | Datasheet name | Units conversion |
|---|---|---|
| `vel` | Accelerometer noise density | (value in µg/√Hz × 9.81e-6)² |
| `att` | Gyroscope noise density | (value in mdeg/s/√Hz × π/180e-3)² |
| `acb` | Accelerometer bias instability | (value in µg × 9.81e-6)² |
| `gyb` | Gyroscope bias instability | (value in deg/hr × π/648000)² |

---

## `PoseUpdate` Block

Only needed if you feed external pose measurements (motion capture, GPS, etc.)
to the `/pose` or `/odometry` topics.

```
PoseUpdate
{
    UpdateNoise
    {
        pos_0  0.01;    Position measurement noise [m^2]
        pos_1  0.01;
        pos_2  0.01;
        att_0  0.01;    Attitude measurement noise [rad^2]
        att_1  0.01;
        att_2  0.01;
    }

    ; Initial and process covariance for the pose-sensor-to-world transform
    init_cov_IrIW  1;       Initial position uncertainty [m^2]
    init_cov_qWI   1;       Initial attitude uncertainty [rad^2]
    init_cov_MrMV  1;
    init_cov_qVM   1;
    pre_cov_IrIW   1e-4;
    pre_cov_qWI    1e-4;
    pre_cov_MrMV   1e-4;
    pre_cov_qVM    1e-4;

    MahalanobisTh0  12.65;

    ; Behaviour flags
    noFeedbackToRovio       true;   true = only use for frame alignment, not filter correction
    enablePosition          true;
    enableAttitude          true;
    doInertialAlignmentAtStart true;
    doVisualization         false;
    useOdometryCov          false;  Use covariance from Odometry message instead of fixed noise

    timeOffset  0.0;                Time offset added to pose timestamps [s]

    ; Transform between IMU and pose sensor body frame
    qVM_x  0;   qVM_y  0;   qVM_z  0;   qVM_w  1;
    MrMV_x 0;   MrMV_y 0;   MrMV_z 0;

    ; Transform between ROVTIO world frame and pose sensor inertial frame
    qWI_x  0;   qWI_y  0;   qWI_z  0;   qWI_w  1;
    IrIW_x 0;   IrIW_y 0;   IrIW_z 0;
}
```

---

## `VelocityUpdate` Block

Only needed if you feed external velocity measurements to `/abss/twist`.

```
VelocityUpdate
{
    UpdateNoise
    {
        vel_0  0.0001;
        vel_1  0.0001;
        vel_2  0.0001;
    }
    MahalanobisTh0  7.69;
    qAM_x  0;   qAM_y  0;   qAM_z  0;   qAM_w  1;
}
```

---

## Complete Minimal Example (2 Cameras)

```
; ================================================================
;  rovtio.info — 2-camera configuration
;  Build with: -DROVIO_NCAM=2 -DROVIO_NMAXFEATURE=25
; ================================================================

Common
{
    doVECalibration  true;
    depthType        1;
    verbose          false;
}

Camera0
{
    CalibrationFile  /path/to/cam0.yaml;
    qCM_x   0.0;   qCM_y   0.0;   qCM_z  -0.7021;   qCM_w  0.7121;
    MrMC_x -0.011; MrMC_y -0.057; MrMC_z  0.021;
}

Camera1
{
    CalibrationFile  /path/to/cam1.yaml;
    qCM_x   0.0;   qCM_y   0.0;   qCM_z  -0.7027;   qCM_w  0.7114;
    MrMC_x -0.009; MrMC_y  0.054; MrMC_z  0.019;
}

Init
{
    State
    {
        pos_0 0; pos_1 0; pos_2 0;
        vel_0 0; vel_1 0; vel_2 0;
        acb_0 0; acb_1 0; acb_2 0;
        gyb_0 0; gyb_1 0; gyb_2 0;
        att_x 0; att_y 0; att_z 0; att_w 1;
    }
    Covariance
    {
        pos_0 0.0001; pos_1 0.0001; pos_2 0.0001;
        vel_0 1.0;    vel_1 1.0;    vel_2 1.0;
        acb_0 4e-4;   acb_1 4e-4;   acb_2 4e-4;
        gyb_0 3e-4;   gyb_1 3e-4;   gyb_2 3e-4;
        att_0 0.1;    att_1 0.1;    att_2 0.1;
        vep 0.0001;
        vea 0.01;
    }
}

ImgUpdate
{
    updateVecNormTermination  1e-4;
    maxNumIteration           20;
    doPatchWarping            true;
    useDirectMethod           true;
    startLevel                2;
    endLevel                  1;
    doFrameVisualisation      true;
    visualizePatches          false;
    nDetectionBuckets         100;
    startDetectionTh          0.8;
    scoreDetectionExponent    0.25;
    penaltyDistance           100;
    zeroDistancePenalty       100;
    MahalanobisTh             9.21;
    initCovFeature_0          0.5;
    initCovFeature_1          1e-5;
    initCovFeature_2          1e-5;
    initDepth                 0.5;
    minRelativeSTScore        0.75;
    minAbsoluteSTScore        5.0;
    useCrossCameraMeasurements true;
    doStereoInitialization    true;
    noiseGainForOffCamera     10.0;
    UpdateNoise
    {
        pix  2;
        int  400;
    }
    MotionDetection
    {
        isEnabled  0;
    }
    ZeroVelocityUpdate
    {
        UpdateNoise { vel_0 0.01; vel_1 0.01; vel_2 0.01; }
        MahalanobisTh0   7.69;
        minNoMotionTime  1.0;
        isEnabled        0;
    }
}

Prediction
{
    PredictionNoise
    {
        pos_0 1e-4;  pos_1 1e-4;  pos_2 1e-4;
        vel_0 4e-6;  vel_1 4e-6;  vel_2 4e-6;
        acb_0 1e-8;  acb_1 1e-8;  acb_2 1e-8;
        gyb_0 3.8e-7; gyb_1 3.8e-7; gyb_2 3.8e-7;
        att_0 7.6e-7; att_1 7.6e-7; att_2 7.6e-7;
        vep  1e-8;
        vea  1e-8;
        dep  0.0001;
        nor  0.00001;
    }
    MotionDetection
    {
        inertialMotionRorTh  0.1;
        inertialMotionAccTh  0.5;
    }
}
```

---

## Common Mistakes

| Mistake | Symptom | Fix |
|---|---|---|
| Wrong number of `CameraN` blocks | Filter crashes or ignores cameras | Must match `-DROVIO_NCAM=N` at build time |
| JPL quaternion instead of Hamilton | Filter immediately diverges | Negate `x`, `y`, `z` components |
| `MrMC` in camera frame not IMU frame | Drift, wrong scale | Express vector in **IMU body frame** |
| Missing `CalibrationFile` path | Wrong intrinsics, blurry tracks | Always provide absolute path |
| Prediction noise too low | Overconfident filter, diverges | Increase `vel`, `att` noise values |
| Prediction noise too high | Sluggish, slow tracking | Decrease `vel`, `att` noise values |
| `isEnabled 0` in ZeroVelocityUpdate | Zero-velocity update silently disabled | Also requires `MotionDetection.isEnabled 1` |
| `noFeedbackToRovio true` in PoseUpdate | Pose not correcting odometry | Set to `false` if you want pose corrections fed back |
