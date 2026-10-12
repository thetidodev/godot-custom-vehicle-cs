using Godot;
using Godot.Collections;


public partial class Vehicle : RigidBody3D
{
    // Pedal position above which a pedal counts as pressed
    const float PedalDeadzone = 0.05f;

    [ExportGroup("Debug")]
    [Export] public Debug debug;
    [Export] public bool DB_forces_visible = false;

    [ExportGroup("Audio")]
    [Export] AudioStreamPlayer3D engineOff;
    [Export] AudioStreamPlayer3D engineOn;

    [ExportGroup("Vehicle")]
    [Export] ControlSettings Controls;
    [Export] float EngineDriveForce = 4f;
    enum EngineDriveForceBehaviorEnum { WheelsSynced, WheelsUnsynced }
    [Export] EngineDriveForceBehaviorEnum EngineDriveForceBehavior = EngineDriveForceBehaviorEnum.WheelsUnsynced;
    [Export] float EngineRevUpSpeed = 150f;
    [Export] float EngineRevDownSpeed = 50f;
    [Export] float EngineRevAlignmentRate = 50f;
    [Export] float EngineIdleRPM = 800f;
    [Export] float EngineMaxRPM = 7000f;
    [Export] float EngineStallPrevention = 2f;
    [Export] Curve EngineTorqueCurve;
    [Export] Curve EngineDeclineCurve;
    [Export] Array<float> GearBoxForwardGearRatios = [3.25f, 1.782f, 1.172f, 1.0f, 0.759f];
    [Export] float GearBoxFinalDriveRatio = 3.772f;
    [Export] Array<float> GearBoxReverseGearRatios = [3.0f];
    [Export] float GearBoxClutchNeeded = 0.25f;

    [ExportGroup("Automatic Gearbox")]
    // When enabled, the gearbox picks gears itself and the gearup/geardown inputs are ignored.
    [Export] bool AutomaticGearbox = false;
    // Engine RPM (as a fraction of the idle-to-redline range) above which the gearbox shifts up.
    [Export(PropertyHint.Range, "0,1,0.01")] float AutoUpshiftRPMFraction = 0.85f;
    // Engine RPM (as a fraction of the idle-to-redline range) below which the gearbox shifts down.
    [Export(PropertyHint.Range, "0,1,0.01")] float AutoDownshiftRPMFraction = 0.3f;
    // Engine RPM (as a fraction of idle RPM) below which the automatic gearbox drops to neutral to avoid stalling.
    [Export(PropertyHint.Range, "0,1,0.01")] float AutoStallRPMFraction = 0.5f;
    // Engine RPM the automatic gearbox must reach from a standstill before engaging first gear or reverse.
    [Export] float AutoLaunchRPM = 1500f;
    // How long the automatic gearbox keeps the clutch pressed after changing gear, in seconds.
    [Export] float AutoClutchHoldTime = 0.1f;
    // Clutch position (0 = engaged, 1 = pressed) the automatic gearbox slips to when pulling away from a standstill.
    [Export(PropertyHint.Range, "0,1,0.01")] float AutoLaunchClutchSlip = 0.5f;
    // Speed (m/s) at which the launch clutch slip has fully released.
    [Export] float AutoClutchLaunchSpeed = 5f;
    // Minimum time between automatic shifts, in seconds.
    [Export] float AutoShiftCooldown = 0.5f;
    // Forward speed (m/s) below which the automatic gearbox may switch between drive and reverse.
    [Export] float AutoDirectionChangeSpeed = 0.5f;

    [ExportGroup("Differentials")]
    [Export] float DIFF_Central_Locking_Preload = 0f;
    [Export] float DIFF_Central_Locking_Power = 0.0f;
    [Export] float DIFF_Central_Locking_Coast = 0.0f;

    [ExportGroup("Steering")]
    [Export] Node3D SteeringPivotNode;
    [Export] Marker3D CenterOfMassNode;
    [Export] float SteeringRadius = 4.0f;

    // Current engine RPM
    float rpm = 0f;
    Array<Wheel> wheels = [];
    Array<Wheel> steeringWheels = [];
    int wheelCount = 0;
    int gear = 0;
    // Total drivetrain weight (sum of DriveTrainInfluence across all driven wheels), cached from last frame
    float cachedDrivetrainWeight = 0f;

    // The Z position of the steering pivot point (rear axle reference for Ackermann steering)
    float steeringPivotZ = -1.151f;
    // Current throttle position (0 = closed, 1 = wide open)
    float throttle = 0;

    public PhysicsDirectBodyState3D physicsState;

    // Raw steering input from the player (-1 = full left, 1 = full right)
    float steeringInput = 0;
    // Final steering angle after assist and speed scaling is applied
    float steeringOutput = 0;
    // Smoothed accelerator pedal position (0-1)
    float analogAccelerate = 0;
    // Smoothed brake pedal position (0-1)
    float analogDecelerate = 0;
    // Pedal positions after mapping to engine throttle and brakes (differs from the raw pedals on the automatic gearbox)
    float throttleInput = 0;
    float brakeInput = 0;
    // Smoothed handbrake position (0-1)
    float analogHandbrake = 0;
    // Smoothed clutch pedal position (0-1)
    float analogClutching = 0;

    // Estimated maximum steering angle, used to scale steering assistance
    float maxSteerEstimate = 0;
    // Scales how strongly steering assistance corrects the vehicle heading
    float steeringAssistanceFactor = 0;
    // Average radius of all driven wheels, weighted by drivetrain influence
    float drivenWheelRadius = 0;
    // Vehicle velocity expressed in local (body) space
    Vector3 localVelocity;
    // Angular velocity expressed in local (body) space
    Vector3 localAngularVelocity;

    // The gear the driver has requested; actual gear change happens once clutch conditions are met
    int targetGear = 0;

    // Predicted linear velocity delta from pre-applied impulses this frame
    Vector3 predictedLinearVelocity;
    // Predicted angular velocity delta from pre-applied impulses this frame
    Vector3 predictedAngularVelocity;

    // Time left before the automatic gearbox is allowed to shift again (seconds)
    float autoShiftCooldownTimer = 0;
    // Gear the automatic gearbox will engage once the clutch is pressed (null when no shift is in progress)
    int? autoPendingGear = null;
    // Time left holding the clutch pressed after an automatic gear change (seconds)
    float autoClutchHoldTimer = 0;

    // Clutch pedal target, eased toward the input so keyboard presses ramp instead of jumping (manual gearbox)
    float desiredClutching = 0;

    // Public props for driving the vehicle
    public float Forward = 0;
    public float Backward = 0;
    public float SteerAxis = 0;
    public float Handbrake = 0;
    public float Clutch = 0;

    readonly TorqueData torqueData = new();

    // Returns the predicted velocity at a world-space position, accounting for impulses already applied this frame.
    public Vector3 PredictVelocityAtPosition(Vector3 worldPosition)
    {
        worldPosition += physicsState.Transform.Origin;
        return predictedLinearVelocity + predictedAngularVelocity.Cross(worldPosition - ToGlobal(CenterOfMass));
    }

    // Accumulates a future impulse so that other systems (e.g. wheels) can read the predicted post-impulse velocity.
    public void PredictImpulse(Vector3 applicationPoint, Vector3 impulse)
    {
        predictedLinearVelocity += impulse * physicsState.InverseMass;
        predictedAngularVelocity += GetInverseInertiaTensor() * (applicationPoint - CenterOfMass * GlobalTransform.Basis).Cross(impulse);
    }

    // Returns the velocity of a point on the rigid body due to both linear and rotational motion.
    public Vector3 GetPointVelocity(Vector3 point) =>
        LinearVelocity + AngularVelocity.Cross(point - GlobalPosition);

    public override void _Ready()
    {
        PrepareWheels();
        CenterOfMassMode = CenterOfMassModeEnum.Custom;
        CenterOfMass = CenterOfMassNode.Position;
        steeringPivotZ = SteeringPivotNode.Position.Z;
    }

    public void PrepareWheels()
    {
        foreach (var node in GetChildren())
        {
            if (node is not Wheel wheel) continue;
            wheelCount++;
            wheels.Add(wheel);
            if (wheel.useSteering) steeringWheels.Add(wheel);
        }
    }

    // Shifts the gearbox up (direction = 1) or down (direction = -1) by one gear.
    public void ShiftGear(int direction)
    {
        int gearLimit = direction > 0 ? GearBoxForwardGearRatios.Count : -GearBoxReverseGearRatios.Count;
        bool withinRange = direction > 0 ? targetGear < gearLimit : targetGear > gearLimit;
        if (!withinRange) return;

        if (analogClutching > GearBoxClutchNeeded)
        {
            // Manual: only shift if the driver has pressed the clutch far enough.
            targetGear += direction;
        }
    }

    // Requests a gear change. Neutral is selected immediately; any other gear waits until the clutch has been pressed
    // (see HandleAutomaticGearbox), the same way a manual shift needs the clutch.
    void AutoSelectGear(int newGear)
    {
        if (newGear == targetGear && autoPendingGear is null) return;
        autoShiftCooldownTimer = AutoShiftCooldown;
        if (newGear == 0)
        {
            targetGear = 0;
            autoPendingGear = null;
        }
        else if (newGear != targetGear) autoPendingGear = newGear;
    }

    // Clutch position the automatic gearbox wants: pressed while shifting or in neutral, then released gradually,
    // slipping at low speed so the engine isn't dragged down when pulling away.
    float AutoClutchTarget()
    {
        if (targetGear == 0 || autoPendingGear is not null || autoClutchHoldTimer > 0) return 1f;
        // Braking without throttle: disconnect the engine so it can't creep against the brakes and hold the vehicle above the direction-change speed.
        if (brakeInput > PedalDeadzone && throttleInput <= PedalDeadzone) return 1f;
        float launchFactor = 1f - Mathf.Clamp(Mathf.Abs(localVelocity.Z) / AutoClutchLaunchSpeed, 0f, 1f);
        return AutoLaunchClutchSlip * launchFactor;
    }

    // Chooses targetGear from throttle/brake input, vehicle speed and engine RPM.
    void HandleAutomaticGearbox(float delta)
    {
        autoShiftCooldownTimer = Mathf.Max(autoShiftCooldownTimer - delta, 0f);
        autoClutchHoldTimer = Mathf.Max(autoClutchHoldTimer - delta, 0f);

        // A gear change is in progress: wait for the clutch to be pressed, change gear, then hold the clutch briefly.
        if (autoPendingGear is not null)
        {
            if (analogClutching > GearBoxClutchNeeded)
            {
                targetGear = autoPendingGear.Value;
                autoPendingGear = null;
                autoClutchHoldTimer = AutoClutchHoldTime;
            }
            return;
        }

        float forwardSpeed = localVelocity.Z;
        bool moving = Mathf.Abs(forwardSpeed) >= AutoDirectionChangeSpeed;
        bool wantsForward = analogAccelerate > PedalDeadzone;
        bool wantsReverse = analogDecelerate > PedalDeadzone;

        float rpmRange = EngineMaxRPM - EngineIdleRPM;
        float upshiftRPM = EngineIdleRPM + rpmRange * AutoUpshiftRPMFraction;
        float downshiftRPM = EngineIdleRPM + rpmRange * AutoDownshiftRPMFraction;
        // Recovered = clearly above the stall threshold. Idle itself is too strict: stall prevention can hold RPM just under it.
        float recoveredRPM = EngineIdleRPM * (1f + AutoStallRPMFraction) * 0.5f;

        // Engine is about to stall: drop to neutral so it can recover. A slipping clutch protects the engine, so only check when mostly engaged.
        if (targetGear != 0 && analogClutching < GearBoxClutchNeeded && rpm < EngineIdleRPM * AutoStallRPMFraction)
        {
            AutoSelectGear(0);
            return;
        }

        // Below AutoDirectionChangeSpeed the pedals pick the direction: up is first gear, down is reverse.
        if (!moving)
        {
            // No pedal pressed while (nearly) stopped, e.g. after braking to a halt: neutral.
            int wanted = wantsForward ? 1 : wantsReverse ? -1 : 0;
            if (wanted == targetGear) return;

            // Change direction through neutral so the engine can rev up to AutoLaunchRPM before the new gear engages.
            if (targetGear != 0 || wanted == 0) AutoSelectGear(0);
            else if (autoShiftCooldownTimer <= 0 && rpm >= AutoLaunchRPM) AutoSelectGear(wanted);
            return;
        }

        // Moving: the gear follows the direction of travel (the pedals only accelerate or brake, see MapPedals).
        if (forwardSpeed < 0)
        {
            if (targetGear != -1 && autoShiftCooldownTimer <= 0 && (targetGear != 0 || rpm >= recoveredRPM)) AutoSelectGear(-1);
            return;
        }

        if (targetGear <= 0)
        {
            if (autoShiftCooldownTimer > 0 || (targetGear == 0 && rpm < recoveredRPM)) return;

            // Pick the lowest forward gear that keeps the engine below the upshift point.
            int engage = GearBoxForwardGearRatios.Count;
            for (int i = 0; i < GearBoxForwardGearRatios.Count; i++)
            {
                if (SpeedBasedRPM(forwardSpeed, GearBoxForwardGearRatios[i]) < upshiftRPM) { engage = i + 1; break; }
            }
            AutoSelectGear(engage);
            return;
        }

        if (autoShiftCooldownTimer > 0) return;

        // Decide from the RPM the vehicle's ground speed implies, not the engine RPM, so wheelspin and airtime don't trigger shifts.
        float currentSpeedRPM = SpeedBasedRPM(forwardSpeed, GearBoxForwardGearRatios[targetGear - 1]);

        if (targetGear < GearBoxForwardGearRatios.Count && currentSpeedRPM > upshiftRPM)
        {
            AutoSelectGear(targetGear + 1);
        }
        else if (targetGear > 1 && currentSpeedRPM < downshiftRPM)
        {
            // Only downshift if the lower gear would not immediately push the engine past the upshift point.
            float predictedRPM = SpeedBasedRPM(forwardSpeed, GearBoxForwardGearRatios[targetGear - 2]);
            if (predictedRPM < upshiftRPM) AutoSelectGear(targetGear - 1);
        }
    }

    // Engine RPM the given forward speed (m/s) would imply through the driven wheels in a gear with the given ratio.
    float SpeedBasedRPM(float forwardSpeed, float gearRatio)
    {
        // drivenWheelRadius is twice the weighted average radius (see the physics loop).
        float averageRadius = drivenWheelRadius / 2.0f;
        if (averageRadius <= 0.001f) return 0f;
        float wheelSpin = Mathf.Abs(forwardSpeed) / averageRadius;
        // The extra 2.0 matches the internal RPM scale used for combinedGearRatio in the physics loop.
        return Rads2Rpm(wheelSpin) * gearRatio * GearBoxFinalDriveRatio * 2.0f;
    }

    // Applies the current gear selection and returns the combined gear + final drive ratio.
    public float Gearbox()
    {
        gear = targetGear;

        float gearRatio = gear > 0 ? GearBoxForwardGearRatios[gear - 1]
                        : gear < 0 ? GearBoxReverseGearRatios[-gear - 1]
                        : 1f;
        return gearRatio * GearBoxFinalDriveRatio;
    }

    // Simulates the engine: updates throttle, advances RPM, and writes torque output to torqueData.
    public void HandleEngine(float delta, float revUpRate, float revDownRate)
    {
        float physicsStepsPerMinute = delta * 60.0f;
        throttle -= (throttle - throttleInput) * 0.5f;

        // The throttle position at which rev-up and rev-down forces exactly balance.
        float torqueBalancePoint = revDownRate / (revUpRate + revDownRate);
        // How far below idle the engine currently is (positive = RPM is below idle).
        float distanceBelowIdle = 1.0f - rpm / EngineIdleRPM;

        // Stall prevention: automatically add throttle when RPM is dangerously low.
        if (rpm < EngineIdleRPM + revDownRate)
            throttle = Mathf.Max(throttle, Mathf.Min(torqueBalancePoint * Mathf.Min(EngineStallPrevention, 1) + distanceBelowIdle * Mathf.Max(EngineStallPrevention - 1, 0), 1));
        // Rev limiter: cut throttle at the RPM ceiling.
        else if (rpm > EngineMaxRPM - revUpRate)
            throttle = 0.0f;

        float netTorque = throttle * revUpRate - revDownRate * (1.0f - throttle);
        rpm += netTorque * physicsStepsPerMinute;

        torqueData.torqueMeasureRads = Rpm2Rads(netTorque);
        torqueData.revDelta = Rpm2Rads(Mathf.Max(revUpRate, revDownRate));
    }

    public static float Rpm2Rads(float rpm) => rpm / 9.549297f;
    public static float Rads2Rpm(float rads) => rads * 9.549297f;

    // Smoothly ramps an analog value (e.g. throttle, brake) toward a target using separate attack/release rates.
    static float RampAnalog(float current, float desired, float onRate, float offRate, float scale)
    {
        if (current < desired - onRate * scale) return current + onRate * scale;
        if (current > desired + offRate * scale) return current - offRate * scale;
        return desired;
    }

    // Maps the up/down pedals onto the engine throttle and the brakes.
    // Manual: up is throttle, down is brake. Automatic: the direction of travel decides. The pedal matching it accelerates
    // and the opposite pedal brakes. Below AutoDirectionChangeSpeed each pedal accelerates in its own direction instead.
    void MapPedals()
    {
        if (!AutomaticGearbox)
        {
            throttleInput = analogAccelerate;
            brakeInput = analogDecelerate;
            return;
        }

        float forwardSpeed = localVelocity.Z;
        if (forwardSpeed >= AutoDirectionChangeSpeed)
        {
            throttleInput = analogAccelerate;
            brakeInput = analogDecelerate;
        }
        else if (forwardSpeed <= -AutoDirectionChangeSpeed)
        {
            throttleInput = analogDecelerate;
            brakeInput = analogAccelerate;
        }
        else
        {
            // Nearly stopped: the pressed pedal (up wins) picks the direction. Only drive once the matching gear (or neutral, to rev up) is selected.
            bool forward = analogAccelerate > PedalDeadzone || analogDecelerate <= PedalDeadzone;
            float pedal = forward ? analogAccelerate : analogDecelerate;
            bool gearMatches = targetGear == 0 || (targetGear > 0) == forward;
            throttleInput = gearMatches ? pedal : 0f;
            // No pedal pressed: hold the vehicle still instead of letting it roll in neutral.
            brakeInput = analogAccelerate > PedalDeadzone || analogDecelerate > PedalDeadzone ? 0f : 1f;
        }
    }

    public void HandleControls(float deltaScale)
    {
        localVelocity = LinearVelocity * GlobalTransform.Basis.Orthonormalized();
        localAngularVelocity = AngularVelocity * GlobalTransform.Basis.Orthonormalized();

        HandlePedals(deltaScale);
        HandleSteeringInput(deltaScale);
    }

    public void HandleInternalInputs()
    {
        Forward = Mathf.Min(Input.GetActionStrength("up"), 1);
        Backward = Mathf.Min(Input.GetActionStrength("down"), 1);
        Handbrake = Mathf.Min(Input.GetActionStrength("jump"), 1);
        Clutch = Mathf.MoveToward(desiredClutching, Input.GetActionStrength("sprint"), 0.2f);
        SteerAxis = Input.GetAxis("right", "left");

        if (Input.IsActionJustPressed("debug_key"))
            DB_forces_visible = !DB_forces_visible;

        if (!AutomaticGearbox)
        {
            if (Input.IsActionJustPressed("gearup")) ShiftGear(1);
            else if (Input.IsActionJustPressed("geardown")) ShiftGear(-1);
        }
    }

    void HandlePedals(float deltaScale)
    {
        float desiredAccelerate = Forward;
        float desiredDecelerate = Backward;
        float desiredHandbrake = Handbrake;

        desiredClutching = Clutch;
        desiredClutching = Mathf.Min(desiredClutching, 1);
        if (AutomaticGearbox) desiredClutching = AutoClutchTarget();

        analogAccelerate = RampAnalog(analogAccelerate, desiredAccelerate, Controls.OnThrottleRate, Controls.OffThrottleRate, deltaScale);
        analogDecelerate = RampAnalog(analogDecelerate, desiredDecelerate, Controls.OnBrakeRate, Controls.OffBrakeRate, deltaScale);
        analogHandbrake = RampAnalog(analogHandbrake, desiredHandbrake, Controls.OnHandbrakeRate, Controls.OffHandbrakeRate, deltaScale);
        analogClutching = RampAnalog(analogClutching, desiredClutching, Controls.OnClutchRate, Controls.OffClutchRate, deltaScale);

        MapPedals();
    }

    // Smooths the steering input and works out the final steering output including speed scaling and assistance.
    void HandleSteeringInput(float deltaScale)
    {
        float desiredSteering = SteerAxis;

        // Steering smoothing: pick a correction speed depending on whether the wheel is
        // turning into the desired direction, oversteering past it, or returning to centre.
        float steeringError = steeringInput - desiredSteering;
        float steeringCorrectionSpeed;

        if (desiredSteering > Controls.KeyboardSteerSpeed)
            steeringCorrectionSpeed = steeringInput > desiredSteering ? Controls.KeyboardReturnSpeed
                                    : Mathf.Abs(steeringError) > 1 ? Controls.KeyboardCompensateSpeed
                                                                        : Controls.KeyboardSteerSpeed;
        else if (desiredSteering < -Controls.KeyboardSteerSpeed)
            steeringCorrectionSpeed = steeringInput < desiredSteering ? Controls.KeyboardReturnSpeed
                                    : Mathf.Abs(steeringError) > 1 ? Controls.KeyboardCompensateSpeed
                                                                        : Controls.KeyboardSteerSpeed;
        else
            steeringCorrectionSpeed = Controls.KeyboardReturnSpeed;

        steeringCorrectionSpeed *= deltaScale;
        if (steeringCorrectionSpeed > 0) steeringInput -= steeringError / (Mathf.Abs(steeringError) / steeringCorrectionSpeed + 1);
        steeringInput = Mathf.Clamp(steeringInput, -1.0f, 1.0f);

        // Lateral speed (sideways sliding). Zero it out if the slide opposes the steer direction
        // so we don't reduce steering authority when the driver is already correcting oversteer.
        float lateralSpeed = Mathf.Abs(localVelocity.X);
        if (localVelocity.X > 0 && steeringInput < 0 || localVelocity.X < 0 && steeringInput > 0)
            lateralSpeed = 0.0f;

        steeringAssistanceFactor = 90.0f / Mathf.Max(maxSteerEstimate, 1f);

        // Blend from 0 (no assist at low speed) to 1 (full assist above threshold speed).
        float assistBlend = Mathf.Clamp((LinearVelocity.Length() - Controls.SteerAssistThreshold) / Mathf.Max(Controls.SteerAssistThreshold, 1), 0f, 1f);
        float forwardSpeed = Mathf.Max(localVelocity.Z / (lateralSpeed + 1.0f), 0f);
        // Reduces maximum steering angle at higher speeds to prevent spinning out.
        float maxSteerFraction = 1.0f / (forwardSpeed * assistBlend * (Controls.SteerAmountDecay / steeringAssistanceFactor) + 1.0f);

        steeringOutput = Mathf.Clamp(
            steeringInput * maxSteerFraction
            + localVelocity.Normalized().X * assistBlend * (Controls.SteeringAssistance * steeringAssistanceFactor)
            - localAngularVelocity.Y * (Controls.SteeringAssistanceAngular * steeringAssistanceFactor * assistBlend),
            -1f, 1f);
    }

    public override void _IntegrateForces(PhysicsDirectBodyState3D state)
    {
        HandleInternalInputs();

        physicsState = state;

        debug.AddOrUpdateDebug("RPM: ", rpm.ToString());
        debug.AddOrUpdateDebug("Gear: ", gear.ToString());
        debug.AddOrUpdateDebug("Forward speed: ", localVelocity.Z.ToString("0.00"));
        debug.AddOrUpdateDebug("Pending gear: ", autoPendingGear?.ToString() ?? "-");
        debug.AddOrUpdateDebug("km/h: ", (LinearVelocity.Length() * 3.6f).ToString());

        float delta = state.Step;
        float physicsStepsPerMinute = delta * 60.0f;
        float wheelDrivenRPM = 0;

        HandleControls(physicsStepsPerMinute);
        if (AutomaticGearbox) HandleAutomaticGearbox(delta);

        // Normalise current RPM to 0-1 between idle and redline, for torque/decline curve sampling.
        float rpmProgress = (rpm - EngineIdleRPM) / (EngineMaxRPM - EngineIdleRPM);
        float torqueRatio = EngineTorqueCurve is not null ? EngineTorqueCurve.SampleBaked(rpmProgress) : 1f;
        float declineRatio = EngineDeclineCurve is not null ? EngineDeclineCurve.SampleBaked(rpmProgress) : 1f;

        float revUpRate = EngineRevUpSpeed * torqueRatio;
        float revDownRate = EngineRevDownSpeed * declineRatio;
        // Multiply gear ratio by 2 to convert from wheel rad/s to the RPM scale used internally.
        float combinedGearRatio = Gearbox() * 2.0f;

        // Apply gravity manually so wheel impulses can interact with it correctly this frame.
        ApplyCentralImpulse(physicsState.TotalGravity * Mass * delta);

        // How strongly the engine RPM is pulled toward the wheel speed through the drivetrain.
        // Zero when in neutral or the clutch is fully pressed.
        float clutchLockStrength = gear == 0 ? 0f
            : (Mathf.Max(EngineRevUpSpeed, EngineRevDownSpeed) + EngineRevAlignmentRate) * (1.0f - analogClutching);
        clutchLockStrength *= physicsStepsPerMinute;

        float clutchLockStrengthRads = Rpm2Rads(clutchLockStrength);
        // Normalises drive torque per unit of drivetrain weight so heavier drivetrains don't receive excess torque.
        float drivetrainNormalisedScale = EngineRevUpSpeed / (EngineDriveForce * physicsStepsPerMinute);
        float engineSpeedRads = Rpm2Rads(rpm);

        HandleEngine(delta, revUpRate, revDownRate);

        cachedDrivetrainWeight = Mathf.Max(cachedDrivetrainWeight, 1);

        HandleSteering();

        // Accumulate weighted average spin rates across all wheels.
        float drivenWheelSpinSum = 0f;
        float allWheelSpinSum = 0f;
        float allWheelTravelledSum = 0f;
        float drivenWheelInfluence = 0f;

        foreach (Wheel wheel in wheels)
        {
            drivenWheelInfluence += wheel.drivetrainInfluence;
            drivenWheelSpinSum += wheel.spin * wheel.drivetrainInfluence;
            allWheelSpinSum += wheel.spin;
            allWheelTravelledSum += wheel.distanceTravelledAllWheelDrive;
        }

        float averageAllWheelSpin = allWheelSpinSum / wheelCount;
        float averageDrivenWheelSpin = drivenWheelSpinSum / drivenWheelInfluence;
        float averageAllWheelTravelled = allWheelTravelledSum / wheelCount;

        drivenWheelRadius = 0;
        // Ratio of coast (rev-down) to power (rev-up) rate — scales coast-side differential locking torque.
        float coastToTorqueRatio = revDownRate / revUpRate;

        predictedLinearVelocity = Vector3.Zero;
        predictedAngularVelocity = Vector3.Zero;

        foreach (var wheel in wheels)
        {
            wheelDrivenRPM += wheel.spin * wheel.drivetrainInfluence;

            // Estimate how much torque this wheel should receive from the engine through the gearbox.
            float wheelTorqueEstimate = torqueData.revDelta * wheel.drivetrainInfluence * combinedGearRatio / cachedDrivetrainWeight / drivetrainNormalisedScale;
            wheelTorqueEstimate = Mathf.Min(wheelTorqueEstimate, clutchLockStrengthRads * drivetrainNormalisedScale);

            // How much the wheel spins beyond what the clutch can hold (0 = no slip, >0 = wheelspin).
            float wheelOverdrive = Mathf.Max(wheelTorqueEstimate / (torqueData.revDelta / (combinedGearRatio / physicsStepsPerMinute)) - 1.0f, 0);

            // Maximum torque the clutch can deliver to this wheel, reduced when the wheel is overdriving.
            float clutchTorqueLimit = clutchLockStrengthRads / (1.0f / (wheelOverdrive / wheel.tireSpinResistanceRate + 1));
            // Clamp engine torque to what the clutch can physically deliver.
            float clampedEngineTorque = Mathf.Clamp(torqueData.torqueMeasureRads, -clutchTorqueLimit, clutchTorqueLimit);

            // Choose whether all driven wheels share a common speed or each tracks its own.
            float referenceWheelSpin = EngineDriveForceBehavior == EngineDriveForceBehaviorEnum.WheelsUnsynced
                ? averageDrivenWheelSpin : wheel.spin;

            float predictedWheelSpin;
            // How far the engine speed is from matching the wheel speed through the gearbox.
            float engineWheelSpeedError;

            if (gear < 0)
            {
                predictedWheelSpin = referenceWheelSpin + clampedEngineTorque / combinedGearRatio - wheel.aligningTorque;
                engineWheelSpeedError = (engineSpeedRads / combinedGearRatio + predictedWheelSpin);
            }
            else
            {
                predictedWheelSpin = referenceWheelSpin - clampedEngineTorque / combinedGearRatio - wheel.aligningTorque;
                engineWheelSpeedError = (engineSpeedRads / combinedGearRatio - predictedWheelSpin);
            }

            // Final torque applied to the wheel: engine torque corrected toward perfect engine-wheel speed alignment.
            float aligningTorque = Mathf.Clamp(engineWheelSpeedError * combinedGearRatio - 1.0f + clampedEngineTorque, -clutchTorqueLimit, clutchTorqueLimit);
            float wheelOutputTorque = aligningTorque * wheel.drivetrainInfluence * combinedGearRatio / cachedDrivetrainWeight / drivetrainNormalisedScale;

            wheel.drivetrainOverdrive = wheelOverdrive;
            wheel.drivetrainTorque = gear < 0 ? -wheelOutputTorque : wheelOutputTorque;

            if (wheel.differentialLockTarget is not null)
            {
                // ── Axle differential ──────────────────────────────────────────────────────────
                // A differential allows the two wheels on the same axle to spin at different
                // speeds (e.g. when cornering). Locking torque limits how different they can become.

                float axleAverageSpin = (wheel.spin + wheel.differentialLockTarget.spin) / 2.0f;
                float axleWheelTravelDelta = wheel.distanceTravelled - wheel.differentialLockTarget.distanceTravelled;
                float axleDampeningTorque = wheel.spin - axleAverageSpin;
                float axleSpinDifference = wheel.spin - wheel.differentialLockTarget.spin;
                // Approaches 1 as the axle spins faster; reduces correction authority at speed.
                float axleStabilisation = 1.0f - (1.0f / (Mathf.Abs(axleAverageSpin * delta * wheel.wheelRadius) + 1));

                float axleLockingTorque = wheel.differentialLockingPreload / 60.0f;
                if (wheelOutputTorque > 0) axleLockingTorque += wheelOutputTorque * wheel.differentialLockingPower * wheel.drivetrainInfluence;
                else axleLockingTorque -= wheelOutputTorque * wheel.differentialLockingCoast * coastToTorqueRatio * wheel.drivetrainInfluence;

                // Travel delta clamped to the locking range; anything outside is slip the diff cannot hold.
                float axleSlipDelta = axleWheelTravelDelta > 0
                    ? Mathf.Max(axleWheelTravelDelta - axleLockingTorque, 0.0f)
                    : Mathf.Min(axleWheelTravelDelta + axleLockingTorque, 0.0f);

                axleWheelTravelDelta = Mathf.Clamp(axleWheelTravelDelta, -axleLockingTorque, axleLockingTorque);
                axleDampeningTorque = Mathf.Clamp(axleDampeningTorque, -axleLockingTorque, axleLockingTorque);

                wheel.distanceTravelled -= axleWheelTravelDelta * axleStabilisation;
                wheel.drivetrainTorque -= axleWheelTravelDelta + axleDampeningTorque
                                 + axleSpinDifference * wheel.differentialViscosity * delta * (wheelOverdrive + 1);

                // ── Central / AWD differential ─────────────────────────────────────────────────
                // In an AWD system a central diff splits torque between front and rear axles.
                // This block limits how different the front/rear average speeds can become.

                float awdDampeningTorque = axleAverageSpin - averageAllWheelSpin;
                float awdPairAverageTravelled = (wheel.distanceTravelledAllWheelDrive + wheel.differentialLockTarget.distanceTravelledAllWheelDrive) / 2.0f;
                float awdTravelDelta = awdPairAverageTravelled - averageAllWheelTravelled;
                float awdStabilisation = 1.0f - (1.0f / (Mathf.Abs(averageAllWheelSpin * delta * wheel.wheelRadius) + 1));
                float awdLockingTorque = DIFF_Central_Locking_Preload / 60.0f;

                if (wheelOutputTorque > 0) awdLockingTorque += wheelOutputTorque * DIFF_Central_Locking_Power;
                else awdLockingTorque -= wheelOutputTorque * DIFF_Central_Locking_Coast * coastToTorqueRatio;

                float awdSlipDelta = awdTravelDelta > 0
                    ? Mathf.Max(awdTravelDelta - awdLockingTorque, 0.0f)
                    : Mathf.Min(awdTravelDelta + awdLockingTorque, 0.0f);

                awdTravelDelta = Mathf.Clamp(awdTravelDelta, -awdLockingTorque, awdLockingTorque);
                awdDampeningTorque = Mathf.Clamp(awdDampeningTorque, -awdLockingTorque, awdLockingTorque);

                wheel.distanceTravelledAllWheelDrive -= awdTravelDelta * awdStabilisation;
                wheel.drivetrainTorque -= awdTravelDelta + awdDampeningTorque;
            }

            // Braking torque = brake capacity x blended pedal input (foot brake + handbrake), scaled by ABS.
            wheel.drivetrainBraking = wheel.drivetrainMaxBrakeTorque
                * Mathf.Min(wheel.drivetrainBrakeBias * brakeInput + wheel.drivetrainHandbrakeBias * analogHandbrake, 1)
                * GetABSThreshold();

            ApplyImpulse(wheel.impulse.force, wheel.impulse.position);
            PredictImpulse(wheel.predictedImpulse.position, wheel.predictedImpulse.force / wheelCount);

            drivenWheelRadius += wheel.wheelRadius * wheel.drivetrainInfluence;
        }

        drivenWheelRadius /= drivenWheelInfluence / 2.0f;

        cachedDrivetrainWeight = drivenWheelInfluence;

        // Convert accumulated wheel spin to RPM space and scale by gear ratio to get drivetrain RPM.
        wheelDrivenRPM *= Rads2Rpm(1) / drivenWheelInfluence * combinedGearRatio;
        if (gear < 0) wheelDrivenRPM = -wheelDrivenRPM;

        // Pull engine RPM toward wheel-driven RPM, limited by how tightly the clutch is engaged.
        float rpmError = rpm - wheelDrivenRPM;
        rpm -= Mathf.Clamp(rpmError, -clutchLockStrength, clutchLockStrength);

        HandleAudio();
    }

    // Returns a 0-1 brake multiplier; ABS reduces it when a wheel is locking up relative to the fastest wheel.
    public float GetABSThreshold()
    {
        float fastestWheelSpin = 0f;
        float absMultiplier = 1f;

        foreach (var wheel in wheels)
            fastestWheelSpin = Mathf.Max(fastestWheelSpin, Mathf.Abs(wheel.spin));

        foreach (var wheel in wheels)
        {
            if (wheel.absEnabled)
                absMultiplier -= Mathf.Abs(Mathf.Abs(wheel.spin + wheel.aligningTorque) - fastestWheelSpin)
                    * wheel.absSensitivity * wheelCount / 100.0f;
        }

        // Handbrake bypasses ABS: lerp toward 1 (full braking) as the handbrake is pulled.
        return Mathf.Lerp(Mathf.Max(absMultiplier, 0.0f), 1.0f, analogHandbrake);
    }

    // Rotates each steered wheel to the correct Ackermann angle for the current steering output.
    public void HandleSteering()
    {
        if (Mathf.Abs(steeringOutput) == 0) return;

        // Turn radius of the vehicle centre: larger steeringOutput = tighter turn.
        float turnRadius = SteeringRadius / Mathf.Abs(steeringOutput);
        if (steeringOutput < 0) turnRadius *= -1.0f;

        foreach (var wheel in steeringWheels)
            wheel.RotationDegrees = wheel.RotationDegrees with
            {
                // Ackermann formula: each wheel steers to a slightly different angle so all four
                // wheel arcs share the same turning centre point.
                Y = Mathf.RadToDeg(Mathf.Atan((-steeringPivotZ + wheel.Position.Z) / (turnRadius - wheel.Position.X)))
            };
    }

    public void HandleAudio()
    {
        float pitchScale = Mathf.Max(rpm, EngineIdleRPM) / 6000.0f;

        engineOn.VolumeDb = engineOn.MaxDb = Mathf.Max(Mathf.LinearToDb(throttle), -80.0f);
        engineOff.VolumeDb = engineOff.MaxDb = Mathf.Max(Mathf.LinearToDb(1.0f - throttle), -80.0f);

        engineOn.PitchScale = pitchScale;
        engineOff.PitchScale = pitchScale;
    }
}

public class TorqueData
{
    // Engine torque expressed in radians/s - used to drive wheel spin each physics step.
    public float torqueMeasureRads = 0;
    // Maximum rev change per step (the larger of rev-up and rev-down rates), in rad/s.
    public float revDelta = 0;
}