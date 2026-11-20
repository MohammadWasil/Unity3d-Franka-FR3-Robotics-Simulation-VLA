using System.Collections.Generic;
using UnityEngine;

public class OpenIKArticulationBridge : MonoBehaviour
{
    [Header("Ghost Rig Reference")]
    public Transform ghostRoot; // Drag fr3 (1) here

    private List<ArticulationBody> physJoints = new List<ArticulationBody>();
    private List<Transform> ghostJoints = new List<Transform>();
    private List<Quaternion> initialGhostRotations = new List<Quaternion>();
    private List<Vector3> localAxes = new List<Vector3>();

    void Start()
    {
        string[] linkNames = {
            "fr3_link1", "fr3_link2", "fr3_link3", "fr3_link4",
            "fr3_link5", "fr3_link6", "fr3_link7"
        };

        var physMap = new Dictionary<string, ArticulationBody>();
        foreach (var ab in GetComponentsInChildren<ArticulationBody>())
            physMap[ab.name] = ab;

        var ghostMap = new Dictionary<string, Transform>();
        foreach (var t in ghostRoot.GetComponentsInChildren<Transform>())
            ghostMap[t.name] = t;

        foreach (string name in linkNames)
        {
            if (physMap.TryGetValue(name, out var articulationBody) && ghostMap.TryGetValue(name, out var ghostTransform))
            {
                physJoints.Add(articulationBody);
                ghostJoints.Add(ghostTransform);
                initialGhostRotations.Add(ghostTransform.localRotation);

                // Match the exact rotation axis of this URDF link's ArticulationBody
                Vector3 axis = (articulationBody.anchorRotation * Vector3.right).normalized;
                localAxes.Add(axis);
            }
        }
    }

    void FixedUpdate()
    {
        for (int i = 0; i < physJoints.Count; i++)
        {
            Quaternion deltaRot = Quaternion.Inverse(initialGhostRotations[i]) * ghostJoints[i].localRotation;
            deltaRot.ToAngleAxis(out float angleDeg, out Vector3 rotAxis);

            if (angleDeg > 180f) angleDeg -= 360f;

            // Project rotation onto this specific link's URDF joint axis
            float signedAngle = 0f;
            if (!float.IsNaN(rotAxis.x) && Mathf.Abs(angleDeg) > 0.001f)
            {
                signedAngle = angleDeg * Vector3.Dot(rotAxis.normalized, localAxes[i]);
            }

            var drive = physJoints[i].xDrive;
            drive.target = Mathf.Clamp(signedAngle, drive.lowerLimit, drive.upperLimit);
            physJoints[i].xDrive = drive;
        }
    }
}
