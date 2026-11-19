using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.UrdfImporter.Control;

public class JacobianBallReach : MonoBehaviour
{
    [Header("References")]
    public Transform endEffector; // Assign gripper/TCP transform (optional: defaults to 7th arm joint)
    public string ballObjectName = "Ball";

    [Header("IK Tuning")]
    public float stepGain = -0.5f;        // How aggressively each step updates angles (radians)
    public float maxDeltaDegPerStep = 2.0f; // Caps per-FixedUpdate angle change for stability
    public float stopDistance = 0.02f;   // Stop stepping once within 2 cm of target
    public float hoverHeight = 0.08f;    // Hover slightly above ball center so it doesn't clip table

    private Controller robotController;
    private readonly List<ArticulationBody> armJoints = new List<ArticulationBody>();
    private Transform ballTransform;

    void Start()
    {
        robotController = GetComponent<Controller>();
        //if (robotController != null)
        //{
        //    robotController.autoControl = true;
        //}

        // Collect the 7 non-fixed arm joints (skips base/fixed links/fingers)
        var fullChain = GetComponentsInChildren<ArticulationBody>();
        foreach (var body in fullChain)
        {
            if (body.jointType != ArticulationJointType.FixedJoint && armJoints.Count < 7)
            {
                armJoints.Add(body);
            }
        }

        if (armJoints.Count < 7)
        {
            Debug.LogError($"JacobianBallReach: Expected 7 arm joints, found {armJoints.Count}.");
            enabled = false;
            return;
        }

        if (endEffector == null)
        {
            endEffector = armJoints[armJoints.Count - 1].transform;
        }

        GameObject ballObj = GameObject.Find(ballObjectName);
        if (ballObj != null)
        {
            ballTransform = ballObj.transform;
        }
        else
        {
            Debug.LogWarning($"JacobianBallReach: Could not find object named '{ballObjectName}'.");
        }
    }

    void FixedUpdate()
    {
        if (ballTransform == null || endEffector == null || armJoints.Count < 7)
            return;

        Vector3 targetPos = ballTransform.position + Vector3.up * hoverHeight;
        Vector3 eePos = endEffector.position;
        Vector3 error = targetPos - eePos;

        if (error.magnitude <= stopDistance)
            return;

        for (int i = 0; i < 7; i++)
        {
            ArticulationBody joint = armJoints[i];
            Vector3 jointPos = joint.transform.position;

            // Default revolute axis for URDF/ArticulationBody X-drive is local +X
            Vector3 axisWorld = joint.transform.TransformDirection(Vector3.right);
            Vector3 r = eePos - jointPos;

            // Column i of the 3x7 positional Jacobian: J_i = axis x r
            Vector3 jacobianCol = Vector3.Cross(axisWorld, r);

            // Jacobian transpose step (in radians)
            float deltaRad = stepGain * Vector3.Dot(jacobianCol, error);
            float deltaDeg = Mathf.Clamp(deltaRad * Mathf.Rad2Deg, -maxDeltaDegPerStep, maxDeltaDegPerStep);

            var drive = joint.xDrive;
            float currentTargetDeg = float.IsNaN(drive.target)
                ? joint.jointPosition[0] * Mathf.Rad2Deg
                : drive.target;

            float nextTargetDeg = currentTargetDeg + deltaDeg;
            if (drive.lowerLimit < drive.upperLimit)
            {
                nextTargetDeg = Mathf.Clamp(nextTargetDeg, drive.lowerLimit, drive.upperLimit);
            }

            drive.target = nextTargetDeg;
            joint.xDrive = drive;
        }
    }
}