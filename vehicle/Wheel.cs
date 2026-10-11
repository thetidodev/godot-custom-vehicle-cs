using Godot;
using Godot.Collections;


public partial class Wheel : Node3D
{
    [ExportGroup("Skidmarks")]
    [Export] public GpuParticles3D skidMarkEmitter;
    [Export] public float skidMarkSensitivity = 100f;
    [Export] public bool leaveSkidMarks = true;

    [ExportGroup("Required Nodes")]
    [Export] public ShapeCast3D shapeCast;
    [Export] Array<MeshInstance3D> spinningMeshes;
    [Export] Array<MeshInstance3D> fixedMeshes;
    [Export] Marker3D spinMarker;
    [Export] Marker3D fixedMarker;
    [Export] Marker3D contactPointMarker;
    [Export] Sprite3D lateralForceArrow;
    [Export] Sprite3D longitudinalForceArrow;
    [Export] Sprite3D suspensionForceArrow;
    [Export] MeshInstance3D patchPositionMesh;
    [Export] public Wheel differentialLockTarget;
    [Export] public Marker3D hubMarker;
    [Export] Generic6DofJoint3D rimJoint;
    Vehicle car;


    [ExportGroup("Suspension")]
    [Export] float suspensionSpringStiffness = 50f;
    [Export] float suspensionDampening = 7f;
    [Export] float suspensionRestLength = 0.15f;
    [Export] float suspensionTireStiffness = 120;
    [Export] float suspensionTireDamping = 7;
    [Export] float suspensionTireHeight = 0.087f;


    [ExportGroup("WheelProperties")]
    [Export] TireModel tireModel;
    [Export] float camberAngle = 0;
    [Export] public bool useSteering = false;
    [Export] public float wheelRadius = 0.55f;
    [Export] float tireDeformationFactor = 5;
    [Export] public float tireSpinResistanceRate = 15;
    [Export] float tireFrictionMultiplier = 0.7f;
    [Export] float tireElasticity = 2000;
    [Export] float tireDampingStiffness = 35;

    [Export] public bool absEnabled = true;
    [Export] public float absSensitivity = 4;

    [Export] public float drivetrainInfluence = 1;
    [Export] public float drivetrainMaxBrakeTorque = 30;
    [Export] public float drivetrainHandbrakeBias = 30;
    [Export] public float drivetrainBrakeBias = 0.8f;


    [ExportGroup("Diff")]
    [Export] public float differentialLockingPreload = 10;
    [Export(PropertyHint.Range, "0,1")] public float differentialLockingPower = 0.2f;
    [Export(PropertyHint.Range, "0,1")] public float differentialLockingCoast = 0.2f;
    [Export] public float differentialViscosity = 0.0f;

    // Used if shapeCast misses a collision for a single tick
    Vector3 lastContactPoint = Vector3.Zero;
    Vector3 lastContactNormal = Vector3.Zero;
    Ground lastContactGround = new();


    public float spin = 0;

    float hubPosition;
    float hubPreviousPosition;

    bool isBrakeLocked = false;
    public float aligningTorque;
    public float distanceTravelled;
    public float distanceTravelledAllWheelDrive;

    public float drivetrainOverdrive;
    public float drivetrainTorque;
    float drivetrainSubstantialTorque;
    public float drivetrainBraking;

    // Converts spring compression to Newtons
    float springForceNewtonFactor = 0.0169946645619466f;

    public ImpulseData impulse = new();
    public ImpulseData predictedImpulse = new();

    Vector3 tirePatchOffset;

    public float outputSkidding;
    public float outputStressing;


    public override void _Ready()
    {
        car = GetParent<Vehicle>();
        shapeCast.AddException(car);

        foreach (var mesh in spinningMeshes)
        {
            RemoveChild(mesh);
            spinMarker.AddChild(mesh);
        }

        foreach (var mesh in fixedMeshes)
        {
            RemoveChild(mesh);
            fixedMarker.AddChild(mesh);
        }

        hubMarker.RotationDegrees = hubMarker.RotationDegrees with
        {
            Z = Position.X > 0 ? -camberAngle : camberAngle
        };

        if (rimJoint != null)
        {
            rimJoint.NodeA = GetParent().GetPath();
            float maxTravel = suspensionRestLength + suspensionTireHeight;
            rimJoint.SetParamY(Generic6DofJoint3D.Param.LinearLowerLimit, -suspensionRestLength);
            rimJoint.SetParamY(Generic6DofJoint3D.Param.LinearUpperLimit, -(suspensionRestLength - maxTravel));
        }
    }

    public override void _PhysicsProcess(double delta)
    {
        if (car.physicsState is null) return;

        contactPointMarker.Visible = car.DB_forces_visible;

        var deltaTime = (float)delta;
        aligningTorque = 0;

        float hertzScale = deltaTime * 60.0f;
        spin += drivetrainTorque / (drivetrainOverdrive + 1.0f);
        spin += drivetrainSubstantialTorque;
        aligningTorque += drivetrainSubstantialTorque;

        if (drivetrainBraking > 0)
        {
            float overdriveAdjustedBraking = drivetrainBraking / (drivetrainOverdrive + 1.0f);
            float spinReduction = spin / Mathf.Max(Mathf.Abs(spin) / overdriveAdjustedBraking, 1);
            spin -= spinReduction;
            aligningTorque -= spin / Mathf.Max(Mathf.Abs(spin) / overdriveAdjustedBraking, 1);
        }

        shapeCast.TargetPosition = shapeCast.TargetPosition with { X = -suspensionRestLength };

        if (isBrakeLocked) spinMarker.RotateX(spin * deltaTime * 2.0f);

        Vector3 contactPoint = GlobalPosition + GlobalBasis.Orthonormalized() * shapeCast.TargetPosition;
        Basis contactAxis = new();

        var isColliding = shapeCast.IsColliding();
        if (isColliding || lastContactPoint != Vector3.Zero)
        {
            Vector3 collisionNormal;
            Ground ground = new();

            if (isColliding)
            {
                collisionNormal = shapeCast.GetCollisionNormal(0);
                contactPoint = shapeCast.GetCollisionPoint(0);
                if (shapeCast.GetCollider(0) is Ground newGround) ground = newGround;
                lastContactNormal = collisionNormal;
                lastContactPoint = contactPoint;
                lastContactGround = ground;
            }
            else
            {
                collisionNormal = lastContactNormal;
                contactPoint = lastContactPoint;
                lastContactNormal = Vector3.Zero;
                lastContactPoint = Vector3.Zero;
                lastContactGround = new();
            }

            // Ignore contacts above the wheel (e.g. ceilings)
            if ((contactPoint - GlobalPosition).Dot(GlobalBasis.Y) > 0) return;

            contactAxis = new Basis(collisionNormal.Cross(GlobalBasis.Z), collisionNormal, GlobalBasis.X.Cross(collisionNormal)).Orthonormalized();

            Vector3 worldOffset = contactPoint - car.GlobalPosition;
            Vector3 tirePatchGlobalVelocity = car.physicsState.GetVelocityAtLocalPosition(worldOffset);
            Vector3 predictedPatchGlobalVelocity = car.PredictVelocityAtPosition(worldOffset);

            Vector3 tirePatchLocalVelocity = tirePatchGlobalVelocity * contactAxis.Orthonormalized();
            Vector3 predictedPatchLocalVelocity = predictedPatchGlobalVelocity * contactAxis.Orthonormalized();

            contactPointMarker.GlobalPosition = contactPoint;
            contactPointMarker.Position = contactPointMarker.Position with { X = 0 };
            contactPoint = contactPointMarker.GlobalPosition;
            contactPointMarker.GlobalBasis = contactAxis;

            hubPreviousPosition = hubPosition;

            float tireStiffnessLateral = tireElasticity;
            float tireDampingLateral = tireDampingStiffness;
            float tireStiffnessLongitudinal = tireStiffnessLateral * tireModel.AspectRatio;
            float tireDampingLongitudinal = tireDampingLateral * tireModel.AspectRatio;

            // Ratio of lateral slip to total wheel speed; blends between lateral and longitudinal tire behavior
            float slipAngleRatio = Mathf.Min(Mathf.Abs(tirePatchLocalVelocity.X) / (Mathf.Abs(spin * wheelRadius) + 1), 1);

            float suspensionDampingForce = suspensionDampening * 1000.0f;
            float suspensionStiffnessForce = suspensionSpringStiffness * springForceNewtonFactor * 1000.0f;
            float tireStiffnessForce = Mathf.Max(suspensionTireStiffness * springForceNewtonFactor * 1000.0f, 0.0001f);
            float tireDampingForce = suspensionTireDamping * 1000.0f;

            float tireComplianceRatio = 1;
            float tireDampingModifier = 1;

            if (tireStiffnessForce > 0 && suspensionStiffnessForce > 0)
            {
                tireComplianceRatio = tireStiffnessForce / (suspensionStiffnessForce + tireStiffnessForce);
                tireDampingModifier = Mathf.Max(tireDampingForce / (suspensionDampingForce + tireDampingForce), deltaTime);
            }

            float tireCompressionDepth = wheelRadius / 2.0f - GlobalPosition.DistanceTo(contactPoint);
            float hubTargetPosition = Mathf.Lerp(suspensionRestLength, -tireCompressionDepth, tireComplianceRatio);
            hubPosition = Mathf.Lerp(hubPosition, hubTargetPosition, tireDampingModifier);
            float tireSinkDepth = Mathf.Max(tireCompressionDepth + hubPosition - suspensionTireHeight, 0);
            hubPosition -= tireSinkDepth;
            hubPosition = Mathf.Max(hubPosition, -tireCompressionDepth);

            float axleVelocity = hubPreviousPosition - hubPosition;
            float suspensionCompression = Mathf.Max(suspensionRestLength - hubPosition, 0.0f);
            float springForce = Mathf.Max(0.0f, suspensionCompression * suspensionStiffnessForce + suspensionDampingForce * axleVelocity);

            float axleMaxExtension = suspensionRestLength;
            float axlePosition = Mathf.Lerp(0.0f, axleMaxExtension, tireComplianceRatio);
            float axleTireSinkAmount = Mathf.Max(axleMaxExtension - axlePosition - suspensionTireHeight, 0.0f);
            axlePosition += axleTireSinkAmount;

            float axleSuspensionCompression = Mathf.Max(axlePosition, 0.0f);
            float axleForceOnSuspension = Mathf.Max(axleSuspensionCompression * suspensionStiffnessForce, 0.0f);
            float axleGripReductionFactor = suspensionStiffnessForce / tireStiffnessForce / suspensionTireHeight;
            float axleMaxGrip = axleForceOnSuspension / (axleTireSinkAmount * axleGripReductionFactor + 1);

            float verticalApproachVelocity = -predictedPatchLocalVelocity.Y;

            float totalGrip = Mathf.Min(springForce * Mathf.Max(1.0f + verticalApproachVelocity, 0.001f), axleMaxGrip) * tireModel.Friction * tireFrictionMultiplier * ground.grip;
            float curvedGripLateral = totalGrip * (tireModel.PeakY * (1.0f - tireModel.Linear) + tireModel.Linear);
            float curvedGripLongitudinal = totalGrip * (tireModel.PeakX * (1.0f - tireModel.Linear) + tireModel.Linear);

            float blendedTireShape = Mathf.Lerp(tireModel.ShapeY, tireModel.ShapeX, slipAngleRatio);
            float blendedTirePeak = Mathf.Lerp(tireModel.PeakY, tireModel.PeakX, slipAngleRatio);
            float blendedPeakedGrip = totalGrip;
            float blendedCurvedGrip = Mathf.Lerp(curvedGripLateral, curvedGripLongitudinal, slipAngleRatio);

            // Difference between actual wheel angular speed and the speed implied by ground contact velocity
            float wheelSpeedDifference = spin - tirePatchLocalVelocity.Z / wheelRadius;
            float predictedLateralVelocity = tirePatchLocalVelocity.X;
            float predictedSlipAmount = 0;

            isBrakeLocked = drivetrainBraking > (totalGrip * (tireSpinResistanceRate * deltaTime * (drivetrainOverdrive + 1.0)));
            if (totalGrip > 0)
            {
                float spinResistanceModifier = tireSpinResistanceRate * deltaTime / (drivetrainOverdrive + 1.0f);
                predictedSlipAmount = Mathf.Max(new Vector2(wheelSpeedDifference, predictedLateralVelocity).Length() / spinResistanceModifier / curvedGripLateral - 1, 0f);
                float predictedPeakedAmount = 1.0f - (1.0f / (Mathf.Max(Mathf.Sqrt(Mathf.Pow(Mathf.Abs(wheelSpeedDifference), 2.0f) + Mathf.Pow(Mathf.Abs(predictedLateralVelocity), 2.0f)) / totalGrip / spinResistanceModifier - 1, 0) + 1.0f));

                if (!isBrakeLocked)
                {
                    float slipCorrectionFactor = predictedSlipAmount * (predictedPeakedAmount / tireModel.ShapeY * tireModel.PeakY * (1.0f - tireModel.Linear) + tireModel.Linear) + 1.0f;
                    spin -= wheelSpeedDifference / slipCorrectionFactor;
                    aligningTorque -= wheelSpeedDifference / slipCorrectionFactor;
                }
            }

            float blendedTireStiffness = Mathf.Lerp(tireStiffnessLateral, tireStiffnessLongitudinal, Mathf.Pow(slipAngleRatio, 0.5f));
            float blendedTireDamping = Mathf.Lerp(tireDampingLateral, tireDampingLongitudinal, Mathf.Pow(slipAngleRatio, 0.5f));

            float brakingForceLimit = drivetrainBraking / (blendedTireStiffness * deltaTime) / tireSpinResistanceRate;
            float torqueForceLimit = drivetrainTorque / (blendedTireStiffness * deltaTime) / tireSpinResistanceRate;

            tirePatchOffset.X -= tirePatchGlobalVelocity.X * deltaTime;
            tirePatchOffset.Z -= tirePatchGlobalVelocity.Z * deltaTime;
            tirePatchOffset -= contactAxis.Y * (tirePatchOffset * contactAxis).Y;

            Vector3 tirePatchDisplacement = tirePatchOffset * contactAxis;

            float standstillFactor = 1.0f;
            if (springForce > 0) standstillFactor /= Mathf.Abs(spin * tireDeformationFactor / 1.5f) / 40.0f + 1f;
            standstillFactor += verticalApproachVelocity / wheelRadius;
            standstillFactor = Mathf.Clamp(standstillFactor, deltaTime, 1f);

            tirePatchOffset -= contactAxis[0] * tirePatchDisplacement.X * (1.0f - standstillFactor);

            if (!isBrakeLocked)
            {
                float longitudinalClampExcess = Mathf.Max(Mathf.Abs(tirePatchDisplacement.Z) - brakingForceLimit, 0f);
                if (tirePatchDisplacement.Z < 0) tirePatchOffset += contactAxis[2] * longitudinalClampExcess;
                else tirePatchOffset -= contactAxis[2] * longitudinalClampExcess;
            }

            float patchSlipAmount = 0;
            float patchPeakedAmount = 0;
            if (totalGrip > 0)
            {
                patchSlipAmount = Mathf.Max(tirePatchOffset.Length() / (curvedGripLateral / blendedTireStiffness) - 1, 0);
                patchPeakedAmount = 1.0f - (1.0f / (Mathf.Max(tirePatchOffset.Length() / (blendedPeakedGrip / blendedTireStiffness) - 1, 0) + 1.0f));
            }

            tirePatchOffset /= patchSlipAmount * (patchPeakedAmount / blendedTireShape * blendedTirePeak * (1.0f - tireModel.Linear) + tireModel.Linear) + 1f;

            if (!isBrakeLocked) tirePatchOffset += contactAxis.Z * torqueForceLimit;

            Vector3 tirePatchDisplacementGlobal = tirePatchOffset * contactAxis;

            Vector3 tireForceVector = new(
                tirePatchLocalVelocity.X * blendedTireDamping * standstillFactor - tirePatchDisplacementGlobal.X * blendedTireStiffness,
                0,
                (tirePatchLocalVelocity.Z - spin * wheelRadius) * blendedTireDamping * standstillFactor - tirePatchDisplacementGlobal.Z * blendedTireStiffness
            );

            if (totalGrip > 0 && isBrakeLocked)
            {
                spin -= wheelSpeedDifference / (predictedSlipAmount + 1f);
                aligningTorque -= wheelSpeedDifference / (predictedSlipAmount + 1);
            }

            float slipAmount = 0;
            float peakedAmount = 0;
            if (totalGrip > 0)
            {
                slipAmount = Mathf.Max(tireForceVector.Length() / blendedCurvedGrip - 1, 0);
                peakedAmount = 1.0f - (1.0f / (Mathf.Max(tireForceVector.Length() / blendedPeakedGrip - 1, 0) + 1.0f));

                outputSkidding = Mathf.Max(slipAmount * blendedCurvedGrip * 0.025f * totalGrip + patchSlipAmount * blendedCurvedGrip * 0.05f * totalGrip, 0);
                outputStressing = tireForceVector.Length() * Mathf.Min(Mathf.Abs(spin), 1);
            }

            tireForceVector /= slipAmount * (peakedAmount / blendedTireShape * blendedTirePeak * (1.0f - tireModel.Linear) + tireModel.Linear) + 1;

            if (totalGrip > 0)
            {
                bool isSlipping = slipAmount > 0;
                float forceLength = tireForceVector.Length();
                UpdateDebugArrow(lateralForceArrow, tireForceVector.X, lateralForceArrow.Scale, forceLength, blendedCurvedGrip, isSlipping);
                UpdateDebugArrow(longitudinalForceArrow, tireForceVector.Z, lateralForceArrow.Scale, forceLength, blendedCurvedGrip, isSlipping);
            }

            suspensionForceArrow.Scale = suspensionForceArrow.Scale with { Y = springForce / 40.0f };

            Vector3 contactForces = contactAxis.Y * springForce;
            Vector3 predictedForces = Vector3.Zero;
            if (totalGrip > 0)
            {
                contactForces -= contactAxis.X * tireForceVector.X;
                contactForces -= contactAxis.Z * tireForceVector.Z;
                predictedForces -= contactAxis.Z * tireForceVector.Z;
            }

            Vector3 impulsePoint = contactPoint - car.GlobalPosition;

            predictedImpulse.SetData(impulsePoint, predictedForces * hertzScale);
            impulse.SetData(impulsePoint, contactForces * hertzScale);

            if (leaveSkidMarks)
            {
                var material = skidMarkEmitter.ProcessMaterial as ParticleProcessMaterial;
                material.Color = ground.skidColor;

                skidMarkEmitter.Emitting = outputSkidding > skidMarkSensitivity * ground.skidMarkSensitivity;
                skidMarkEmitter.GlobalPosition = contactPoint + Vector3.Up * 0.005f;
                var skidMarkLookDirection = skidMarkEmitter.GlobalPosition + car.GetPointVelocity(skidMarkEmitter.GlobalPosition).Normalized();
                if (skidMarkLookDirection.Dot(Vector3.Up) > 0.001f) skidMarkEmitter.LookAt(skidMarkLookDirection);
            }
        }
        else
        {
            skidMarkEmitter.Emitting = false;
            outputSkidding = 0;
            outputStressing = 0;
            contactPointMarker.GlobalPosition = contactPoint;
            impulse.SetData(Vector3.Zero, Vector3.Zero);
            predictedImpulse.SetData(Vector3.Zero, Vector3.Zero);
            tirePatchOffset = Vector3.Zero;
            hubPosition = suspensionRestLength;
        }

        patchPositionMesh.Position = patchPositionMesh.Position with { X = tirePatchOffset.X * contactAxis[0].X + tirePatchOffset.Y * contactAxis[0].Z };
        patchPositionMesh.Position = patchPositionMesh.Position with { Z = tirePatchOffset.X * contactAxis[2].X + tirePatchOffset.Y * contactAxis[2].Z };

        hubMarker.Position = hubMarker.Position with { Y = -hubPosition };
        if (!isBrakeLocked) spinMarker.RotateX(spin * deltaTime * 2.0f);

        distanceTravelled += spin;
        distanceTravelledAllWheelDrive += spin;
    }

    private static void UpdateDebugArrow(Sprite3D arrow, float value, Vector3 baseScale, float forceLength, float curvedGrip, bool isSlipping)
    {
        arrow.Scale = baseScale with { Y = value / 40.0f };
        arrow.RotationDegrees = arrow.RotationDegrees with { X = arrow.Scale.Y > 0 ? 91 : 89 };

        if (isSlipping)
        {
            arrow.Modulate = new Color(1, 0, 0);
        }
        else
        {
            float loadRatio = 1.0f + (forceLength - curvedGrip) / curvedGrip;
            arrow.Modulate = new Color(0, loadRatio, 1.0f - loadRatio);
        }
    }
}

public class ImpulseData
{
    public Vector3 position = new();
    public Vector3 force = new();

    public void SetData(Vector3 pos, Vector3 force)
    {
        position = pos;
        this.force = force;
    }
}
