using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class FR3BallPicker : MonoBehaviour
{
    [Header("Scene References")]
    public Rigidbody ballRigidbody;
    public Transform graspAttachPoint; // Assign fr3_hand_tcp or fr3_hand in Inspector

    [Header("Motion Settings")]
    public float moveDuration = 2.0f;
    public float gripperOpenPos = 0.04f;   // 40 mm per finger (80 mm total opening)
    public float gripperClosedPos = 0.0375f; // Adjust slightly smaller than ball radius. Radius is : 0.075/2 = 0.0375.

    private ArticulationBody[] armJoints = new ArticulationBody[7];
    private ArticulationBody leftFinger;
    private ArticulationBody rightFinger;
    private FixedJoint graspJoint;

    // Pre-computed joint angles (in degrees) for FR3 (7-DOF)
    private readonly float[] homePose = { 0f, -45f, 0f, -135f, 0f, 90f, 45f };
    private readonly float[] preGraspPose = { 0f, 15f, 0f, -110f, 0f, 125f, 45f };
    private readonly float[] graspPose = { 0f, 32f, 0f, -102f, 0f, 134f, 45f };
    private readonly float[] liftPose = { 0f, 5f, 0f, -115f, 0f, 120f, 45f };
    private readonly float[] placePose = { 45f, 25f, 0f, -105f, 0f, 130f, 45f };

    void Start()
    {
        MapFR3Joints();
        StartCoroutine(PickAndPlaceRoutine());
    }

    private void MapFR3Joints()
    {
        ArticulationBody[] allBodies = GetComponentsInChildren<ArticulationBody>();
        Dictionary<string, ArticulationBody> bodyMap = new Dictionary<string, ArticulationBody>();

        foreach (var body in allBodies)
        {
            bodyMap[body.name] = body;
        }

        // Standard FR3 URDF link names corresponding to joints 1-7
        string[] linkNames = {
            "fr3_link1", "fr3_link2", "fr3_link3", "fr3_link4",
            "fr3_link5", "fr3_link6", "fr3_link7"
        };

        for (int i = 0; i < 7; i++)
        {
            if (bodyMap.TryGetValue(linkNames[i], out ArticulationBody joint))
                armJoints[i] = joint;
            else
                Debug.LogError($"Missing FR3 link: {linkNames[i]}");
        }

        bodyMap.TryGetValue("fr3_leftfinger", out leftFinger);
        bodyMap.TryGetValue("fr3_rightfinger", out rightFinger);
    }

    private IEnumerator PickAndPlaceRoutine()
    {
        // 1. Move to Home & Open Gripper
        SetGripperTarget(gripperOpenPos);
        yield return MoveArmToPose(homePose, moveDuration);

        // 2. Approach above the ball
        yield return MoveArmToPose(preGraspPose, moveDuration);

        // 3. Lower onto the ball
        yield return MoveArmToPose(graspPose, moveDuration * 0.75f);

        // 4. Close fingers and secure ball
        SetGripperTarget(gripperClosedPos);
        yield return new WaitForSeconds(0.6f);
        AttachBall();

        // 5. Lift ball
        yield return MoveArmToPose(liftPose, moveDuration);

        // 6. Move to drop-off location
        yield return MoveArmToPose(placePose, moveDuration);

        // 7. Release ball
        ReleaseBall();
        SetGripperTarget(gripperOpenPos);
        yield return new WaitForSeconds(0.5f);

        // 8. Return to Home
        yield return MoveArmToPose(homePose, moveDuration);
    }

    private IEnumerator MoveArmToPose(float[] targetAnglesDeg, float duration)
    {
        float[] startAngles = new float[7];
        for (int i = 0; i < 7; i++)
        {
            startAngles[i] = armJoints[i].xDrive.target;
        }

        float elapsed = 0f;
        while (elapsed < duration)
        {
            elapsed += Time.deltaTime;
            float t = Mathf.SmoothStep(0f, 1f, elapsed / duration);

            for (int i = 0; i < 7; i++)
            {
                var drive = armJoints[i].xDrive;
                drive.target = Mathf.Lerp(startAngles[i], targetAnglesDeg[i], t);
                armJoints[i].xDrive = drive;
            }
            yield return null;
        }
    }

    private void SetGripperTarget(float targetMeters)
    {
        if (leftFinger != null)
        {
            var leftDrive = leftFinger.xDrive;
            leftDrive.target = targetMeters;
            leftFinger.xDrive = leftDrive;
        }

        if (rightFinger != null)
        {
            var rightDrive = rightFinger.xDrive;
            rightDrive.target = targetMeters;
            rightFinger.xDrive = rightDrive;
        }
    }

    private void AttachBall()
    {
        if (ballRigidbody == null || graspAttachPoint == null) return;

        // Check proximity before grasping
        if (Vector3.Distance(ballRigidbody.position, graspAttachPoint.position) < 0.08f)
        {
            ballRigidbody.velocity = Vector3.zero;
            ballRigidbody.angularVelocity = Vector3.zero;
            ballRigidbody.isKinematic = true;
            ballRigidbody.transform.SetParent(graspAttachPoint);
        }
    }

    private void ReleaseBall()
    {
        if (ballRigidbody == null) return;

        ballRigidbody.transform.SetParent(null);
        ballRigidbody.isKinematic = false;
    }
}