using UnityEngine;
using Unity.Robotics.UrdfImporter;

public class DisableGhostPhysics : MonoBehaviour
{
    void Awake()
    {
        foreach (var ab in GetComponentsInChildren<ArticulationBody>(true))
            ab.enabled = false;

        foreach (var inertial in GetComponentsInChildren<UrdfInertial>(true))
            inertial.enabled = false;

        foreach (var joint in GetComponentsInChildren<UrdfJoint>(true))
            joint.enabled = false;

        // Hide the ghost robot's 3D meshes so only the real physics robot is visible
        foreach (var renderer in GetComponentsInChildren<MeshRenderer>(true))
            renderer.enabled = false;
    }
}
