using Godot;


[GlobalClass]
public partial class ControlSettings : Resource
{
    [Export] public float SteerAssistThreshold = 5.0f;
    [Export] public float KeyboardSteerSpeed = 0.05f;
    [Export] public float KeyboardReturnSpeed = 0.05f;
    [Export] public float KeyboardCompensateSpeed = 0.1f;

    [Export] public float SteerAmountDecay = 0.4f;
    [Export] public float SteeringAssistance = 0.0f;
    [Export] public float SteeringAssistanceAngular = 0.0f;

    [Export] public float OnThrottleRate = 0.2f;
    [Export] public float OffThrottleRate = 0.2f;

    [Export] public float OnBrakeRate = 0.05f;
    [Export] public float OffBrakeRate = 0.1f;

    [Export] public float OnHandbrakeRate = 0.2f;
    [Export] public float OffHandbrakeRate = 0.2f;

    [Export] public float OnClutchRate = 0.2f;
    [Export] public float OffClutchRate = 0.2f;

    public ControlSettings() { }
}
