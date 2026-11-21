using System.Collections;
using UnityEngine;

public class EpisodeManager : MonoBehaviour
{
    [Header("Scene References")]
    public Transform ikTarget;              // The target GameObject driven by OpenIK
    public Transform endEffectorTCP;        // fr3_hand_tcp on the real physics fr3
    public Rigidbody ballRigidbody;         // Red Ball Rigidbody
    public Transform dropperPlatform;       // Green DropperPlatform
    public Transform spawnAreaBox;          // Drag your manual white cube here!
    public DataLogger dataLogger;           // Refer to Data Logger

    [Header("Gripper Articulation Drives (Real FR3)")]
    public ArticulationBody leftFinger;     // fr3_leftfinger
    public ArticulationBody rightFinger;    // fr3_rightfinger
    public float fingerOpenTarget = 0.04f;
    public float fingerClosedTarget = 0.0365f;
    public bool attachBallOnGrasp = true;

    [Header("Workspace & Trajectory Settings")]
    public Vector3 homeHoverPos = new Vector3(0.0f, 0.42f, 0.48f);
    public float minDistanceFromPlatform = 0.25f;
    public float preGraspHeight = 0.14f;
    public float liftHeight = 0.18f;
    public float placeHeightOffset = 0.085f;
    public float moveSpeed = 0.35f;
    public int totalEpisodes = 100;


    [Header("Runtime Status (Read-Only)")]
    public int currentEpisode = 0;
    public int successfulEpisodes = 0;
    public string currentStage = "Initializing";

    private float ballRestY;
    private Quaternion topDownRotation = Quaternion.Euler(180f, 0f, 0f);

    void Start()
    {
        if (ballRigidbody != null)
            ballRestY = ballRigidbody.transform.position.y;

        StartCoroutine(RunEpisodeLoop());
    }

    IEnumerator RunEpisodeLoop()
    {
        currentStage = "Initial Settle";
        ikTarget.rotation = topDownRotation;
        ikTarget.position = homeHoverPos;
        SetGripper(fingerOpenTarget);
        yield return new WaitForSeconds(2.0f);

        while (currentEpisode < totalEpisodes)
        {
            currentEpisode++;

            // 1. Reset & Randomize Ball Inside spawnAreaBox
            currentStage = "Reset & Spawn";
            ResetAndSpawnBall();
            SetGripper(fingerOpenTarget);
            yield return MoveTargetTo(homeHoverPos, 0.015f);
            yield return new WaitForSeconds(0.25f);

            // Start the recording
            if (dataLogger != null) dataLogger.BeginEpisode();

            // 2. Pre-Grasp (Hover directly above the ball)
            currentStage = "Pre-Grasp";
            Vector3 ballPos = ballRigidbody.transform.position;
            Vector3 preGraspPos = new Vector3(ballPos.x, ballRestY + preGraspHeight, ballPos.z);
            yield return MoveTargetTo(preGraspPos, 0.01f);

            // 3. Descend onto the Ball
            currentStage = "Descend";
            Vector3 graspPos = new Vector3(ballPos.x, ballRestY, ballPos.z);
            yield return MoveTargetTo(graspPos, 0.008f);
            yield return new WaitForSeconds(0.15f);

            // 4. Close Gripper & Grasp
            currentStage = "Grasp";
            SetGripper(fingerClosedTarget);
            yield return new WaitForSeconds(0.35f);

            float graspError = Vector3.Distance(endEffectorTCP.position, ballRigidbody.position);
            if (attachBallOnGrasp && graspError < 0.05f)
            {
                ballRigidbody.isKinematic = true;
                ballRigidbody.transform.SetParent(endEffectorTCP);
                ballRigidbody.transform.localPosition = Vector3.zero;
            }
            else if (graspError >= 0.05f)
            {
                Debug.LogWarning($"[Episode {currentEpisode}] Missed grasp! TCP error = {graspError:F3}m at ball pos {ballRigidbody.position}");
            }

            // 5. Lift Straight Up
            currentStage = "Lift";
            Vector3 liftPos = new Vector3(graspPos.x, ballRestY + liftHeight, graspPos.z);
            yield return MoveTargetTo(liftPos, 0.015f);

            // 6. Transport Above Green Platform
            currentStage = "Transport";
            Vector3 abovePlatformPos = new Vector3(
                dropperPlatform.position.x,
                dropperPlatform.position.y + liftHeight,
                dropperPlatform.position.z
            );
            yield return MoveTargetTo(abovePlatformPos, 0.015f);

            // 7. Lower to Release Height
            currentStage = "Lower to Place";
            Vector3 placePos = new Vector3(
                dropperPlatform.position.x,
                dropperPlatform.position.y + placeHeightOffset,
                dropperPlatform.position.z
            );
            yield return MoveTargetTo(placePos, 0.01f);

            // 8. Open Gripper & Release Ball
            currentStage = "Release";
            if (attachBallOnGrasp)
            {
                ballRigidbody.transform.SetParent(null);
                ballRigidbody.isKinematic = false;
            }
            SetGripper(fingerOpenTarget);
            yield return new WaitForSeconds(0.4f);

            // 9. Retreat Upward & Evaluate Success
            currentStage = "Retreat & Check";
            yield return MoveTargetTo(abovePlatformPos, 0.02f);
            yield return new WaitForSeconds(0.3f);

            bool success = CheckPlacementSuccess();
            if (dataLogger != null) dataLogger.EndEpisode(success);

            if (success)
            {
                successfulEpisodes++;
                Debug.Log($"[Episode {currentEpisode}/{totalEpisodes}] SUCCESS (Total: {successfulEpisodes})");
            }
            else
            {
                Debug.LogWarning($"[Episode {currentEpisode}/{totalEpisodes}] FAILED - Ball missed platform.");
            }
        }

        currentStage = "Completed";
    }

    IEnumerator MoveTargetTo(Vector3 destination, float arrivalThreshold)
    {
        ikTarget.rotation = topDownRotation;
        while (Vector3.Distance(ikTarget.position, destination) > arrivalThreshold)
        {
            ikTarget.position = Vector3.MoveTowards(
                ikTarget.position,
                destination,
                moveSpeed * Time.fixedDeltaTime
            );
            yield return new WaitForFixedUpdate();
        }
        ikTarget.position = destination;

        // Wait up to 1.5s for the real physics gripper (endEffectorTCP) to actually reach destination
        float timeout = 1.5f;
        while (Vector3.Distance(endEffectorTCP.position, destination) > 0.015f && timeout > 0f)
        {
            timeout -= Time.fixedDeltaTime;
            yield return new WaitForFixedUpdate();
        }
    }

    void SetGripper(float targetPos)
    {
        if (leftFinger != null)
        {
            var leftDrive = leftFinger.xDrive;
            leftDrive.target = targetPos;
            leftFinger.xDrive = leftDrive;
        }
        if (rightFinger != null)
        {
            var rightDrive = rightFinger.xDrive;
            rightDrive.target = targetPos;
            rightFinger.xDrive = rightDrive;
        }
    }

    void ResetAndSpawnBall()
    {
        ballRigidbody.transform.SetParent(null);
        ballRigidbody.velocity = Vector3.zero;
        ballRigidbody.angularVelocity = Vector3.zero;
        ballRigidbody.isKinematic = true; // Prevents rolling before grasp

        // Inset by ball radius (0.0375m) so the entire ball stays strictly inside the white box
        float ballRadius = 0.0375f;
        Vector3 worldScale = spawnAreaBox.lossyScale;
        float halfX = Mathf.Max(0f, 0.5f - (ballRadius / worldScale.x));
        float halfZ = Mathf.Max(0f, 0.5f - (ballRadius / worldScale.z));

        Vector3 candidateWorldPos = spawnAreaBox.position;
        for (int attempt = 0; attempt < 30; attempt++)
        {
            Vector3 localRandom = new Vector3(
                Random.Range(-halfX, halfX),
                0f,
                Random.Range(-halfZ, halfZ)
            );
            Vector3 worldPoint = spawnAreaBox.TransformPoint(localRandom);
            candidateWorldPos = new Vector3(worldPoint.x, ballRestY, worldPoint.z);

            float distToPad = Vector2.Distance(
                new Vector2(candidateWorldPos.x, candidateWorldPos.z),
                new Vector2(dropperPlatform.position.x, dropperPlatform.position.z)
            );

            if (distToPad >= minDistanceFromPlatform)
                break;
        }

        ballRigidbody.transform.position = candidateWorldPos;
        ballRigidbody.transform.rotation = Quaternion.identity;
    }


    bool CheckPlacementSuccess()
    {
        Vector3 b = ballRigidbody.transform.position;
        Vector3 p = dropperPlatform.position;
        float horizontalDist = Vector2.Distance(new Vector2(b.x, b.z), new Vector2(p.x, p.z));
        return horizontalDist < 0.14f && b.y >= p.y;
    }
}
