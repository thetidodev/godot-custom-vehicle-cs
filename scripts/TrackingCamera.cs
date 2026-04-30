using Godot;
using System;

public partial class TrackingCamera : Camera3D
{
    [Export] Node3D target;

    public override void _Process(double delta)
    {
        if (target is null) return;
        LookAt(target.GlobalPosition);
    }
}
