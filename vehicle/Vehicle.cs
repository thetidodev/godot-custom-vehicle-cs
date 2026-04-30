using Godot;
using Godot.Collections;


public partial class Vehicle : RigidBody3D
{
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

    [Export] float DIFF_Central_Locking_Preload = 0f;
    [Export] float DIFF_Central_Locking_Power = 0.0f;
    [Export] float DIFF_Central_Locking_Coast = 0.0f;

    [Export] Node3D SteeringPivotNode;
    [Export] Marker3D CenterOfMassNode;
    [Export] float SteeringRadius = 4.0f;

    // How fast the engine is spinning based on wheel speed through the drivetrain (RPM)
    float wheelDrivenRPM = 0;
    // Current engine RPM
    float rpm = 0f;
    // Engine RPM from the previous physics frame
    float previousRPM = 0;
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

    // Countdown timer: automatic clutch stays engaged while this is above zero during a gear shift
    float autoClutchEngageTimer = 0;
    // Countdown timer: throttle is blipped/cut while this is above zero during a gear shift
    float autoThrottleBlipTimer = 0;

    // The gear the driver has requested; actual gear change happens once clutch conditions are met
    int targetGear = 0;
    // True when the vehicle is currently driving in reverse
    bool onReverse = false;

    // Predicted linear velocity delta from pre-applied impulses this frame
    Vector3 predictedLinearVelocity;
    // Predicted angular velocity delta from pre-applied impulses this frame
    Vector3 predictedAngularVelocity;

    // The gear ratio from one gear below the current gear (used to compare RPM across shifts)
    float previousGearRatio = 0;

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

        if (Controls.AST_shift_assistance > 0)
        {
            // Automatic shift assistance: trigger clutch and throttle blip timers, then shift immediately.
            if (targetGear != 0)
            {
                autoClutchEngageTimer = Controls.AST_shifting_clutch_out_time;
                autoThrottleBlipTimer = Controls.AST_shifting_off_throttle_time;
            }
            targetGear += direction;
        }
        else if (analogClutching > GearBoxClutchNeeded)
        {
            // Manual: only shift if the driver has pressed the clutch far enough.
            targetGear += direction;
        }
    }

    public override void _Input(InputEvent @event)
    {
        if (@event.IsActionPressed("gearup")) ShiftGear(1);
        else if (@event.IsActionPressed("geardown")) ShiftGear(-1);

        if (@event.IsActionPressed("debug_key"))
            DB_forces_visible = !DB_forces_visible;
    }

    // Applies the current gear selection and returns the combined gear + final drive ratio.
    public float Gearbox()
    {
        float gearRatio = 1f;

        if (Controls.AST_shift_assistance > 0)
        {
            // Commit the target gear once the clutch is engaged or we are in neutral.
            if (analogClutching > GearBoxClutchNeeded || gear == 0)
            {
                gear = targetGear;
                if (gear == 0)
                {
                    autoClutchEngageTimer = -1;
                    autoThrottleBlipTimer = -1;
                }
            }
            if (autoThrottleBlipTimer <= 0) onReverse = gear < 0;
        }
        else gear = targetGear;

        // Look up the ratio one gear below current, for cross-gear RPM comparisons.
        int previousGearIndex = Mathf.Abs(gear) - 2;
        previousGearRatio = 0.001f;

        if (gear > 0 && previousGearIndex >= 0) previousGearRatio = GearBoxForwardGearRatios[previousGearIndex];
        else if (gear < 0 && previousGearIndex >= 0) previousGearRatio = GearBoxReverseGearRatios[previousGearIndex];

        if (gear > 0) gearRatio = GearBoxForwardGearRatios[gear - 1];
        else if (gear < 0) gearRatio = GearBoxReverseGearRatios[Mathf.Abs(gear) - 1];

        previousGearRatio *= GearBoxFinalDriveRatio * 2.0f;
        return gearRatio * GearBoxFinalDriveRatio;
    }

    // Simulates the engine: updates throttle, advances RPM, and writes torque output to torqueData.
    public void HandleEngine(float delta, float revUpRate, float revDownRate)
    {
        float physicsStepsPerMinute = delta * 60.0f;
        throttle -= (throttle - analogAccelerate) * 0.5f;

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

    public void HandleControls(float deltaScale)
    {
        float desiredSteering = Input.GetAxis("right", "left");
        float desiredAccelerate = Mathf.Min(Input.GetActionStrength("up"), 1);
        float desiredDecelerate = Mathf.Min(Input.GetActionStrength("down"), 1);
        float desiredHandbrake = Mathf.Min(Input.GetActionStrength("jump"), 1);

        float desiredClutching = Controls.AST_shift_assistance > 0
            ? Input.GetActionStrength("jump")
            : Input.GetActionStrength("sprint");

        // Auto-clutch: compute how far the engine RPM is below the target engagement point.
        float clutchEngagementRPM = EngineIdleRPM + Controls.AST_clutch_in_rpm_offset;
        float distanceBelowEngagementRPM = (clutchEngagementRPM - rpm) / (clutchEngagementRPM - EngineIdleRPM);

        if (Controls.AST_shift_assistance > 0)
            desiredClutching += Mathf.Clamp(distanceBelowEngagementRPM, 0, 1);

        desiredClutching = Mathf.Min(desiredClutching, 1);

        // During an automatic gear shift, override throttle/clutch to match engine speed to the new gear.
        if (autoThrottleBlipTimer > 0)
        {
            if (rpm < wheelDrivenRPM) { desiredAccelerate = 1; autoClutchEngageTimer = autoThrottleBlipTimer; }
            else { desiredAccelerate = 0; autoThrottleBlipTimer -= deltaScale; }
        }
        if (autoClutchEngageTimer > 0)
        {
            desiredClutching = 1;
            autoClutchEngageTimer -= deltaScale;
        }

        analogAccelerate = RampAnalog(analogAccelerate, desiredAccelerate, Controls.OnThrottleRate, Controls.OffThrottleRate, deltaScale);
        analogDecelerate = RampAnalog(analogDecelerate, desiredDecelerate, Controls.OnBrakeRate, Controls.OffBrakeRate, deltaScale);
        analogHandbrake = RampAnalog(analogHandbrake, desiredHandbrake, Controls.OnHandbrakeRate, Controls.OffHandbrakeRate, deltaScale);
        analogClutching = RampAnalog(analogClutching, desiredClutching, Controls.OnClutchRate, Controls.OffClutchRate, deltaScale);

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

        localVelocity = LinearVelocity * GlobalTransform.Basis.Orthonormalized();
        localAngularVelocity = AngularVelocity * GlobalTransform.Basis.Orthonormalized();

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
        physicsState = state;

        debug.AddOrUpdateDebug("RPM: ", rpm.ToString());
        debug.AddOrUpdateDebug("Gear: ", gear.ToString());
        debug.AddOrUpdateDebug("km/h: ", (LinearVelocity.Length() * 3.6f).ToString());

        float delta = state.Step;
        float physicsStepsPerMinute = delta * 60.0f;
        float drivetrainWeight = 0;
        wheelDrivenRPM = 0;

        HandleControls(physicsStepsPerMinute);

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
            drivetrainWeight += wheel.drivetrainInfluence;
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
                * Mathf.Min(wheel.drivetrainBrakeBias * analogDecelerate + wheel.drivetrainHandbrakeBias * analogHandbrake, 1)
                * GetABSThreshold();

            ApplyImpulse(wheel.impulse.force, wheel.impulse.position);
            PredictImpulse(wheel.predictedImpulse.position, wheel.predictedImpulse.force / wheelCount);

            drivenWheelRadius += wheel.wheelRadius * wheel.drivetrainInfluence;
        }

        drivenWheelRadius /= drivetrainWeight / 2.0f;

        cachedDrivetrainWeight = drivetrainWeight;

        // Convert accumulated wheel spin to RPM space and scale by gear ratio to get drivetrain RPM.
        wheelDrivenRPM *= Rads2Rpm(1) / drivetrainWeight * combinedGearRatio;
        if (gear < 0) wheelDrivenRPM = -wheelDrivenRPM;

        // Pull engine RPM toward wheel-driven RPM, limited by how tightly the clutch is engaged.
        float rpmError = rpm - wheelDrivenRPM;
        rpm -= Mathf.Clamp(rpmError, -clutchLockStrength, clutchLockStrength);

        previousRPM = rpm;
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
        float pitchScale = Mathf.Max(rpm, 800f) / 6000.0f;

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