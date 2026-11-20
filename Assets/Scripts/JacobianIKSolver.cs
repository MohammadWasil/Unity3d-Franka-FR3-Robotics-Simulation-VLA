using System.Collections.Generic;
using UnityEngine;

public class JacobianIKSolver : MonoBehaviour
{
    [Header("Target & End Effector")]
    public Transform ikTarget;          // Drag IK_Target here
    public Transform endEffectorTCP;    // Assign fr3_hand_tcp

    [Header("IK Solver Tuning")]
    [Range(0.01f, 0.5f)] public float stepGain = 0.2f;
    [Range(0.001f, 0.2f)] public float damping = 0.05f;
    public bool lockTopDownOrientation = true;

    private List<ArticulationBody> armJoints = new List<ArticulationBody>();

    void Start()
    {
        string[] linkNames = {
            "fr3_link1", "fr3_link2", "fr3_link3", "fr3_link4",
            "fr3_link5", "fr3_link6", "fr3_link7"
        };

        var bodyMap = new Dictionary<string, ArticulationBody>();
        foreach (var ab in GetComponentsInChildren<ArticulationBody>())
            bodyMap[ab.name] = ab;

        foreach (string linkName in linkNames)
        {
            if (bodyMap.TryGetValue(linkName, out ArticulationBody joint))
                armJoints.Add(joint);
            else
                Debug.LogError($"Missing FR3 link: {linkName}");
        }
    }

    void FixedUpdate()
    {
        if (ikTarget == null || endEffectorTCP == null || armJoints.Count < 7) return;

        // 1. Compute 6D task-space error (3 position + 3 orientation)
        Vector3 posError = ikTarget.position - endEffectorTCP.position;
        Vector3 rotError = Vector3.zero;

        if (lockTopDownOrientation)
        {
            Quaternion targetRot = Quaternion.LookRotation(Vector3.down, ikTarget.forward);
            Quaternion deltaRot = targetRot * Quaternion.Inverse(endEffectorTCP.rotation);
            deltaRot.ToAngleAxis(out float angleDeg, out Vector3 axis);
            if (angleDeg > 180f) angleDeg -= 360f;
            if (!float.IsNaN(axis.x) && Mathf.Abs(angleDeg) > 0.01f)
                rotError = axis.normalized * (angleDeg * Mathf.Deg2Rad);
        }

        float[] error6D = { posError.x, posError.y, posError.z, rotError.x, rotError.y, rotError.z };

        // 2. Build geometric 6x7 Jacobian directly from each revolute joint's world axis
        float[,] J = new float[6, 7];
        Vector3 tcpPos = endEffectorTCP.position;

        for (int c = 0; c < 7; c++)
        {
            ArticulationBody joint = armJoints[c];
            // URDF Importer aligns each revolute joint's rotation axis with its local X-axis
            Vector3 jointAxisWorld = joint.transform.TransformDirection(joint.anchorRotation * Vector3.right).normalized;
            Vector3 jointPosWorld = joint.transform.TransformPoint(joint.anchorPosition);

            Vector3 linearVel = Vector3.Cross(jointAxisWorld, tcpPos - jointPosWorld);
            Vector3 angularVel = jointAxisWorld;

            J[0, c] = linearVel.x;
            J[1, c] = linearVel.y;
            J[2, c] = linearVel.z;
            J[3, c] = angularVel.x;
            J[4, c] = angularVel.y;
            J[5, c] = angularVel.z;
        }

        // 3. Damped Least Squares transpose step: deltaTheta = alpha * J^T * error
        float[] jte = new float[7];
        for (int c = 0; c < 7; c++)
            for (int r = 0; r < 6; r++)
                jte[c] += J[r, c] * error6D[r];

        float jNormSq = damping * damping;
        for (int r = 0; r < 6; r++)
            for (int c = 0; c < 7; c++)
                jNormSq += J[r, c] * J[r, c];

        float alpha = stepGain / jNormSq;

        // 4. Apply delta joint angles (in degrees) to ArticulationBody xDrives
        for (int i = 0; i < 7; i++)
        {
            var drive = armJoints[i].xDrive;
            float deltaDeg = jte[i] * alpha * Mathf.Rad2Deg;
            drive.target = Mathf.Clamp(drive.target + deltaDeg, drive.lowerLimit, drive.upperLimit);
            armJoints[i].xDrive = drive;
        }
    }
}
