using System;
using UnityEngine;

// Sequence execution only. SimulationRunner owns lifecycle; Python owns the Jev API.
[RequireComponent(typeof(BoatDynamics2D))]
public sealed class BoatActionSequence : MonoBehaviour
{
    public bool HasPlan => plan != null;
    private BoatDynamics2D boat;
    private DiscreteBoatAction[] plan;
    private int stepsPerAction, stepsExecuted;
    private float fixedDt;
    void Awake() { boat = GetComponent<BoatDynamics2D>(); }
    public void Cancel() { plan = null; boat.ClearControl(); }
    public void AcceptPlan(DiscreteBoatAction[] actions, float actionDuration, float dt)
    {
        DiscreteActionPlan.Validate(actions);
        stepsPerAction = DiscreteActionPlan.Steps(actionDuration, dt);
        plan = Array.ConvertAll(actions, a => new DiscreteBoatAction {
            acceleration_index = a.acceleration_index, turn_index = a.turn_index });
        fixedDt = dt; stepsExecuted = 0;
    }
    public bool PrepareStep(float dt)
    {
        if (!HasPlan) return false;
        if (Mathf.Abs(fixedDt - dt) > 1e-7f) throw new InvalidOperationException("Fixed timestep changed during execution.");
        var action = plan[stepsExecuted / stepsPerAction];
        boat.SetControl(DiscreteActionPlan.AccelerationFractions[action.acceleration_index] * boat.maxAcceleration,
            DiscreteActionPlan.TurnFractions[action.turn_index] * boat.maxAngularAccelerationDeg);
        return true;
    }
    public void FinishStep()
    {
        if (HasPlan && ++stepsExecuted == stepsPerAction * plan.Length)
        { plan = null; boat.ClearControl(); }
    }
}
