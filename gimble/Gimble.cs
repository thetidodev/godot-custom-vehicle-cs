using System;
using System.Collections.Generic;
using System.Linq;
using Godot;


public partial class Gimble : Node3D
{
    [Export] RigidBody3D car;
    [Export] Node3D target;
    [Export] Node3D rotationPoint;
    [Export] SpringArm3D springArm;
    [Export] Camera3D camera;
    [Export] float defaultFOV = 75f;
    [Export] float defaultSpringLength = 5f;

    List<Camera3D> cameras = [];
    int activeCam = 5;

    public override void _Ready()
    {
        cameras = [.. GetTree().GetNodesInGroup("Camera").Select(x => x as Camera3D)];
        SwitchCamera();
    }

    public void SwitchCamera()
    {
        if (activeCam == cameras.Count - 1) activeCam = 0;
        else activeCam++;

        foreach (var cam in cameras) cam.Current = false;
        cameras[activeCam].Current = true;
    }

    public override void _Input(InputEvent @event)
    {
        if (@event is InputEventMouseMotion mouseEvent && Input.MouseMode == Input.MouseModeEnum.Captured)
        {
            springArm.RotateX(mouseEvent.Relative.Y * 0.001f);
            rotationPoint.RotateY(-mouseEvent.Relative.X * 0.001f);
            springArm.Rotation = new Vector3(Mathf.Clamp(springArm.Rotation.X, Mathf.DegToRad(-80), Mathf.DegToRad(80)), 180, 0);
        }
    }

    public override void _PhysicsProcess(double delta)
    {
        if (Input.IsActionJustPressed("esc")) Input.MouseMode = Input.MouseMode == Input.MouseModeEnum.Captured ? Input.MouseModeEnum.Visible : Input.MouseModeEnum.Captured;
        if (Input.IsActionJustPressed("cam")) SwitchCamera();

        springArm.RotateX(Input.GetAxis("rUp", "rDown") * 0.04f);
        rotationPoint.RotateY(Input.GetAxis("rRight", "rLeft") * 0.04f);
        springArm.Rotation = new Vector3(Mathf.Clamp(springArm.Rotation.X, Mathf.DegToRad(-80), Mathf.DegToRad(80)), 180, 0);

        GlobalPosition = GlobalPosition.Lerp(target.GlobalPosition, 0.2f);
        GlobalRotation = GlobalRotation with { Y = target.GlobalRotation.Y };

        camera.Fov = defaultFOV * Mathf.Clamp((260f + car.LinearVelocity.Length() * 3.6f) / 300f, 1, 1.5f);
        springArm.SpringLength = defaultSpringLength * 1 / Mathf.Clamp((260f + car.LinearVelocity.Length() * 3.6f) / 300f, 1, 1.5f);
    }
}
