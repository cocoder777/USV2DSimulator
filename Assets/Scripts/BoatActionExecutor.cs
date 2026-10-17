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
    void Awake() { boat = GetComponent<BoatDynamics2D>(); }
    public void Cancel() { action = null; remainingSteps = 0; boat.ClearControl(); }
    public void AcceptAction(DiscreteBoatAction command, float actionDuration, float dt)
    {
        DiscreteAction.Validate(command);
        remainingSteps = DiscreteAction.Steps(actionDuration, dt);
        action = new DiscreteBoatAction {
            acceleration_index = command.acceleration_index, turn_index = command.turn_index };
        fixedDt = dt;
    }
    public bool PrepareStep(float dt)
    {
        if (!HasAction) return false;
        if (Mathf.Abs(fixedDt - dt) > 1e-7f) throw new InvalidOperationException("Fixed timestep changed during execution.");
        boat.SetControl(DiscreteAction.AccelerationFractions[action.acceleration_index] * boat.maxAcceleration,
            DiscreteAction.TurnFractions[action.turn_index] * boat.maxAngularAccelerationDeg);
        return true;
    }
    public void FinishStep()
    {
        if (HasAction && --remainingSteps == 0) Cancel();
    }
}
