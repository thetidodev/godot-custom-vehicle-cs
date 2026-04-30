using Godot;


[GlobalClass]
public partial class ControlSettings : Resource
{
    public enum ASTShiftAssistanceEnum
    {
        None = 0,
        AutoClutch = 1,
    }
    [Export] public ASTShiftAssistanceEnum AST_shift_assistance = ASTShiftAssistanceEnum.AutoClutch;
    [Export] public float AST_clutch_in_rpm_offset = 2000f;
    [Export] public float AST_upshift_threshold = -250f;
    [Export] public float AST_downshift_threshold = -500f;
    [Export] public float AST_standstill_threshold = -100f;
    [Export] public int AST_shifting_clutch_out_time = 20;
    [Export] public int AST_shifting_off_throttle_time = 21;
    [Export] public int AST_reverse_delay = 30;
    [Export] public bool AST_instant_reverse = false;

    [Export] public float SteerSensitivity = 1.0f;
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
