using System;
using UnityEngine;

// Executes one action. SimulationRunner owns lifecycle; Python owns the Jev API.
[RequireComponent(typeof(BoatDynamics2D))]
public sealed class BoatActionExecutor : MonoBehaviour
{
    public bool HasAction => action != null;
    private BoatDynamics2D boat;
    private DiscreteBoatAction action;
    private int remainingSteps;
    private float fixedDt;
    private float targetHeading;
    void Awake() { boat = GetComponent<BoatDynamics2D>(); }
    public void Cancel() { action = null; remainingSteps = 0; boat.ClearControl(); }
    public void AcceptAction(DiscreteBoatAction command, float actionDuration, float dt)
    {
        DiscreteAction.Validate(command);
        if (command.speed * DiscreteAction.MetresPerSecondPerKnot > boat.maxForwardSpeed + 0.00001f)
            throw new FormatException("Target speed exceeds the boat limit.");
        remainingSteps = DiscreteAction.Steps(actionDuration, dt);
        action = new DiscreteBoatAction {
            speed = command.speed, course = command.course };
        // Simulator actuator adapter; the decision layer never needs the bow angle.
        targetHeading = command.speed == 0 ? boat.HeadingDeg : 90f - command.course;
        fixedDt = dt;
    }
    public bool PrepareStep(float dt)
    {
        if (!HasAction) return false;
        if (Mathf.Abs(fixedDt - dt) > 1e-7f) throw new InvalidOperationException("Fixed timestep changed during execution.");
        boat.SetNavigationTarget(action.speed * DiscreteAction.MetresPerSecondPerKnot, targetHeading);
        return true;
    }
    public void FinishStep()
    {
        if (HasAction && --remainingSteps == 0) Cancel();
    }
}
