using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.UrdfImporter.Control;

public class AutoController : MonoBehaviour
{
    private Controller robotController;
    private List<ArticulationBody> armJoints = new List<ArticulationBody>();
    private Transform ballTransform;

    public float transitionDuration = 2.0f;

    void Start()
    {
        robotController = GetComponent<Controller>();
        if (robotController == null)
        {
            Debug.LogError("Controller component not found!");
            return;
        }

        // Take over control so arrow keys don't interfere
        //robotController.autoControl = true;

        // Find the 7 revolute arm joints automatically
        //var fullChain = robotController.GetArticulationChain();
        var fullChain = GetComponentsInChildren<ArticulationBody>();
        foreach (var body in fullChain)
        {
            if (body.jointType == ArticulationJointType.RevoluteJoint && armJoints.Count < 7)
            {
                armJoints.Add(body);
            }
        }

        // Automatically find the ball in the scene by name or tag
        GameObject ballObj = GameObject.Find("Ball"); // Make sure your ball GameObject is named "Ball"
        if (ballObj != null)
        {
            ballTransform = ballObj.transform;
        }
        else
        {
            Debug.LogError("Could not find an object named 'Ball' in the scene!");
        }

        StartCoroutine(TrackAndReachBall());
    }

    IEnumerator TrackAndReachBall()
    {
        yield return new WaitForSeconds(1.0f);

        if (armJoints.Count < 7 || ballTransform == null) yield break;

        // 1. Automatically calculate joint angles based on the ball's position
        Vector3 ballPos = ballTransform.position;

        // Calculate base rotation angle (Joint 1) to face the ball using Atan2
        float baseAngleRad = Mathf.Atan2(ballPos.x, ballPos.z);
        float baseAngleDeg = baseAngleRad * Mathf.Rad2Deg;

        // Calculate distance from robot base to ball for reaching
        float distanceToBall = new Vector3(ballPos.x, 0, ballPos.z).magnitude;

        // Dynamic target angles calculated automatically from the ball's location:
        // Joint 1: Turns to face the ball
        // Joints 2-4: Adjust reach and pitch based on distance
        float[] dynamicTargetAngles = new float[7]
        {
            baseAngleDeg,                          // Joint 1 (Base yaw)
            -45f - (distanceToBall * 30f),         // Joint 2 (Shoulder pitch)
            0f,                                    // Joint 3 (Arm rotation)
            -90f + (distanceToBall * 40f),         // Joint 4 (Elbow pitch)
            0f,                                    // Joint 5 (Wrist yaw)
            90f,                                   // Joint 6 (Wrist pitch)
            45f                                    // Joint 7 (Finger/Wrist rotation)
        };

        // 2. Smoothly move from current angles to the dynamic target angles
        float[] startAngles = new float[7];
        for (int i = 0; i < 7; i++)
        {
            startAngles[i] = armJoints[i].jointPosition[0] * Mathf.Rad2Deg;
        }

        float elapsedTime = 0f;
        while (elapsedTime < transitionDuration)
        {
            elapsedTime += Time.deltaTime;
            float t = Mathf.Clamp01(elapsedTime / transitionDuration);
            float smoothT = Mathf.SmoothStep(0f, 1f, t);

            for (int i = 0; i < 7; i++)
            {
                float currentTarget = Mathf.Lerp(startAngles[i], dynamicTargetAngles[i], smoothT);

                var drive = armJoints[i].xDrive;
                drive.target = Mathf.Clamp(currentTarget, drive.lowerLimit, drive.upperLimit);
                armJoints[i].xDrive = drive;
            }

            yield return null;
        }

        Debug.Log("Robot automatically reached toward the ball!");
    }
}