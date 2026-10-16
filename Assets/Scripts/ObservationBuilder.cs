using System;
using Newtonsoft.Json.Linq;
using UnityEngine;

[Serializable]
public sealed class MissionSnapshot
{
    public string route_id, status;
    public int route_revision, passed_count;
    public TargetRouteData.RoutePoint next_target;
    public float arrival_radius_m, stop_speed_mps, cruise_speed_mps;

    public static MissionSnapshot Capture(MissionManager mission)
    {
        if (mission == null || mission.Route == null) return null;
        return new MissionSnapshot { route_id = mission.Route.route_id, route_revision = mission.Revision,
            status = mission.Status, passed_count = mission.Progress.NextIndex,
            next_target = mission.Progress.Next, arrival_radius_m = mission.Route.arrival_radius_m,
            stop_speed_mps = mission.Route.stop_speed_mps, cruise_speed_mps = mission.Route.cruise_speed_mps };
    }
}

public static class ObservationBuilder
{
    // Model input intentionally omits transport IDs, watchdog flags, UI data and full path duplication.
    public static JObject ForModel(BoatDynamics2D boat, MissionManager mission, float dt, int actionCount = 1, float actionDuration = 0.4f)
    {
        if (mission?.Route == null) throw new InvalidOperationException("No task loaded.");
        var target = mission.Progress.Next;
        var futurePoints = new JArray();
        for (int i = mission.Progress.NextIndex + 1; i < mission.Route.waypoints.Length; i++)
            futurePoints.Add(JObject.FromObject(mission.Route.waypoints[i]));
        var samples = mission.Route.GetReferenceSamples();
        int nearest = 0;
        int searchStart = 0, searchEnd = samples.Length - 1;
        if (mission.Route.schema_version == 3 && target != null)
        {
            int next = mission.Progress.NextIndex;
            searchStart = mission.Route.path.waypoint_sample_indices[Math.Max(0, next - 1)];
            searchEnd = mission.Route.path.waypoint_sample_indices[next];
            nearest = searchStart;
        }
        float best = float.PositiveInfinity;
        for (int i = searchStart; i <= searchEnd; i++)
        {
            float distance = (new Vector2(samples[i].x, samples[i].y) - boat.Position).sqrMagnitude;
            if (distance < best) { nearest = i; best = distance; }
        }
        int targetSample = samples.Length - 1;
        if (target != null)
        {
            if (mission.Route.schema_version == 3) targetSample = searchEnd;
            else
            {
                float targetBest = float.PositiveInfinity;
                for (int i = 0; i < samples.Length; i++)
                {
                    float d = (new Vector2(samples[i].x - target.x, samples[i].y - target.y)).sqrMagnitude;
                    if (d < targetBest) { targetBest = d; targetSample = i; }
                }
            }
            nearest = Mathf.Min(nearest, targetSample); // Never look past an unpassed checkpoint.
        }
        int lookahead = nearest;
        float lookaheadDistance = Mathf.Clamp(Mathf.Abs(boat.Speed) * 1.2f, 1f, 2.5f), accumulated = 0;
        while (lookahead < targetSample && accumulated < lookaheadDistance)
        {
            accumulated += Vector2.Distance(new Vector2(samples[lookahead].x, samples[lookahead].y),
                new Vector2(samples[lookahead + 1].x, samples[lookahead + 1].y));
            lookahead++;
        }
        var lookaheadPoint = new Vector2(samples[lookahead].x, samples[lookahead].y);
        Vector2 toLookahead = lookaheadPoint - boat.Position;
        float goalHeading = Mathf.Atan2(toLookahead.y, toLookahead.x) * Mathf.Rad2Deg;
        float distanceToTarget = target == null ? 0 : Vector2.Distance(boat.Position, new Vector2(target.x, target.y));
        float speedGoal = mission.Route.cruise_speed_mps;
        if (target?.role == "end")
            speedGoal = Mathf.Min(speedGoal, Mathf.Sqrt(1.0f * Mathf.Max(0, distanceToTarget - 0.5f * mission.Route.arrival_radius_m)));
        var navigation = new JObject {
            ["lookahead_xy"] = new JArray(lookaheadPoint.x, lookaheadPoint.y),
            ["heading_error_deg"] = Mathf.DeltaAngle(boat.HeadingDeg, goalHeading),
            ["distance_to_next_target_m"] = distanceToTarget, ["speed_goal_mps"] = speedGoal,
            ["reference_distance_m"] = Vector2.Distance(boat.Position, new Vector2(samples[nearest].x, samples[nearest].y)),
            ["meaning"] = "Geometry/task goals only, not actuator commands. Positive heading error means target to left. Lookahead cannot skip an unpassed checkpoint. End speed goal decreases towards an inner stopping region." };
        float headingError = Mathf.DeltaAngle(boat.HeadingDeg, goalHeading);
        float yawGoal = Mathf.Clamp(1.5f * headingError, -boat.maxYawRateDeg, boat.maxYawRateDeg);
        if (speedGoal == 0) yawGoal = 0;
        if (Mathf.Abs(headingError) > 40) speedGoal = Mathf.Min(speedGoal, 0.4f);
        navigation["speed_goal_mps"] = speedGoal;
        navigation["yaw_rate_goal_deg_s"] = yawGoal;
        navigation["goal_rule"] = "Yaw-rate goal = clamp(1.5 * lookahead heading error, +/- max yaw rate). This is a geometric tracking setpoint, not an angular acceleration command.";
        var accelerationForecasts = new JArray(); var turnForecasts = new JArray();
        int forecastSteps = DiscreteActionPlan.Steps(actionDuration * actionCount, dt);
        for (int i = 0; i < DiscreteActionPlan.AccelerationFractions.Length; i++)
        {
            float command = DiscreteActionPlan.AccelerationFractions[i] * boat.maxAcceleration;
            float value = BoatScalarDynamics.Predict(boat.Speed, boat.ActualAccelerationValue, command,
                boat.accelerationTimeConstant, boat.linearDamping, dt, forecastSteps, -boat.maxReverseSpeed, boat.maxForwardSpeed);
            accelerationForecasts.Add(new JObject { ["index"] = i, ["input_mps2"] = command,
                ["predicted_speed_mps"] = Math.Round(value, 4), ["tracking_error"] = Math.Round(Mathf.Abs(value - speedGoal), 4) });
        }
        for (int i = 0; i < DiscreteActionPlan.TurnFractions.Length; i++)
        {
            float command = DiscreteActionPlan.TurnFractions[i] * boat.maxAngularAccelerationDeg;
            float value = BoatScalarDynamics.Predict(boat.YawRateDeg, boat.ActualAngularAccelerationDeg, command,
                boat.angularAccelerationTimeConstant, boat.angularDamping, dt, forecastSteps, -boat.maxYawRateDeg, boat.maxYawRateDeg);
            turnForecasts.Add(new JObject { ["index"] = i, ["input_deg_s2"] = command,
                ["predicted_yaw_rate_deg_s"] = Math.Round(value, 4), ["tracking_error"] = Math.Round(Mathf.Abs(value - yawGoal), 4) });
        }
        var localPath = new JArray();
        // At most 13 points, about 1 m apart on the current sampled curve, plus the explicit next target.
        if (mission.Route.schema_version == 3)
        {
            float distanceSinceSample = 0;
            localPath.Add(new JArray(samples[nearest].x, samples[nearest].y));
            for (int i = nearest + 1; i < samples.Length && localPath.Count < 13; i++)
            {
                distanceSinceSample += Vector2.Distance(new Vector2(samples[i - 1].x, samples[i - 1].y),
                    new Vector2(samples[i].x, samples[i].y));
                if (distanceSinceSample < 1f) continue;
                localPath.Add(new JArray(samples[i].x, samples[i].y));
                distanceSinceSample = 0;
            }
        }
        else
            for (int i = nearest; i < samples.Length && localPath.Count < 13; i += 10)
                localPath.Add(new JArray(samples[i].x, samples[i].y));
        var parameters = JObject.FromObject(BoatParameters.Capture(boat));
        parameters.Remove("fixed_dt");
        parameters["integration"] = "Every fixed_dt: actuator += (command-actuator)*(1-exp(-dt/time_constant)); v += (actuator_acceleration-linear_damping*v)*dt; yaw_rate += (actuator_angular_acceleration-angular_damping*yaw_rate)*dt; clamp speeds; heading += yaw_rate*dt; position += signed_speed * heading_unit_vector * dt. Zero time_constant means no lag.";
        return new JObject {
            ["units"] = "world XY metres; seconds; degrees; heading 0=E,90=N,counterclockwise positive",
            ["planning"] = new JObject { ["action_count"] = actionCount, ["action_duration_s"] = actionDuration,
                ["decision_period_s"] = actionCount * actionDuration, ["fixed_dt_s"] = dt, ["simulation_paused_while_deciding"] = true },
            ["actions"] = new JObject {
                ["acceleration_fractions"] = new JArray(DiscreteActionPlan.AccelerationFractions),
                ["turn_fractions"] = new JArray(DiscreteActionPlan.TurnFractions),
                ["semantics"] = "Signed longitudinal acceleration and angular acceleration COMMANDS. Positive turn=left/CCW; negative=right/CW. Zero does not stop motion. No heading servo, no automatic yaw braking." },
            ["boat"] = new JObject { ["x"] = boat.Position.x, ["y"] = boat.Position.y,
                ["heading_deg"] = boat.HeadingDeg, ["speed_mps"] = boat.Speed, ["yaw_rate_deg_s"] = boat.YawRateDeg,
                ["actuator_acceleration_mps2"] = boat.ActualAccelerationValue,
                ["actuator_angular_acceleration_deg_s2"] = boat.ActualAngularAccelerationDeg },
            ["dynamics"] = parameters,
            ["navigation"] = navigation,
            ["input_forecasts"] = new JObject { ["horizon_s"] = actionDuration * actionCount,
                ["acceleration"] = accelerationForecasts, ["turn"] = turnForecasts,
                ["assumption"] = "Each scalar input held constant for horizon. Independent speed/yaw equations include current actuator lag, damping and limits. No Cartesian product or preselected action." },
            ["mission"] = new JObject { ["status"] = mission.Status,
                ["next_target"] = target == null ? JValue.CreateNull() : JObject.FromObject(target),
                ["subsequent_targets"] = futurePoints, ["arrival_radius_m"] = mission.Route.arrival_radius_m,
                ["end_stop_speed_mps"] = mission.Route.stop_speed_mps,
                ["cruise_speed_mps"] = mission.Route.cruise_speed_mps,
                ["rules"] = "Visit next_target then subsequent_targets in order. Intermediate points do not require stopping. End requires distance AND abs(speed) within limits simultaneously." },
            ["reference_path_xy"] = localPath
        };
    }
}
