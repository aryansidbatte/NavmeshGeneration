using UnityEngine;
using System.Collections.Generic;
using TMPro;

public class SteeringBehavior : MonoBehaviour
{
    public Vector3 target;
    public KinematicBehavior kinematic;
    public List<Vector3> path;
    // you can use this label to show debug information,
    // like the distance to the (next) target
    public TextMeshProUGUI label;
    
    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        kinematic = GetComponent<KinematicBehavior>();
        target = transform.position;
        path = null;
        EventBus.OnSetMap += SetMap;
        EventBus.OnPath += SetPath;
    }

    // Update is called once per frame
    void Update()
    {
        // Calculate base directional information
        Vector3 direction = target - transform.position;
        Vector3 heading = transform.forward;
        float angleToTarget = Vector3.SignedAngle(heading, direction, Vector3.up);
        float distanceToTarget = direction.magnitude;
        direction.Normalize();

        // Path following logic
        if (path != null && path.Count > 0)
        {
            // Always update target to the current waypoint
            target = path[0];

            // Recalculate direction information with updated target
            direction = target - transform.position;
            distanceToTarget = direction.magnitude;
            angleToTarget = Vector3.SignedAngle(heading, direction, Vector3.up);
            direction.Normalize();

            // Check if we've reached the current waypoint
            if (distanceToTarget < 1.0f)
            {
                // Remove this waypoint from the path
                path.RemoveAt(0);

                // If we still have waypoints, update to the next one
                if (path.Count > 0)
                {
                    target = path[0];
                    // Recalculate for the new target
                    direction = target - transform.position;
                    distanceToTarget = direction.magnitude;
                    angleToTarget = Vector3.SignedAngle(heading, direction, Vector3.up);
                    direction.Normalize();
                }
            }

            // Set desired speed and rotational velocity based on distance to target
            kinematic.SetDesiredSpeed(Mathf.Clamp(distanceToTarget, 0.0f, kinematic.max_speed));
            kinematic.SetDesiredRotationalVelocity(angleToTarget * 2.0f);
        }
        // This is the single target behavior
        else if (distanceToTarget < 1.0f)
        {
            // If we are close to the target, stop
            kinematic.SetDesiredSpeed(0.0f);
            kinematic.SetDesiredRotationalVelocity(0.0f);
        }
        else
        {
            // Set desired speed and rotational velocity based on distance to target
            kinematic.SetDesiredSpeed(Mathf.Clamp(distanceToTarget, 0.0f, kinematic.max_speed));
            kinematic.SetDesiredRotationalVelocity(angleToTarget * 2.0f);
        }

        // Update label with distance to target
        if (label != null)
        {
            label.text = "Distance to target: " + distanceToTarget.ToString("F2") + " m";

            // Added path information to debug label
            if (path != null && path.Count > 0)
            {
                label.text += "\nWaypoints left: " + path.Count;
            }
        }
    }

    public void SetTarget(Vector3 target)
    {
        this.target = target;
        EventBus.ShowTarget(target);
    }

    public void SetPath(List<Vector3> path)
    {
        this.path = path;
    }

    public void SetMap(List<Wall> outline)
    {
        this.path = null;
        this.target = transform.position;
    }
}