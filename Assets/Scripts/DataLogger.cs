using System;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using System.Text;
using UnityEngine;

public class DataLogger : MonoBehaviour
{
    [Header("Cameras (224x224 for Pi-0)")]
    public Camera wristCamera;              // Drag wrist_camera here
    public Camera exteriorCamera;           // Drag exterior_camera_1 here
    public int imageWidth = 224;
    public int imageHeight = 224;

    [Header("Robot Joints & Gripper (Real FR3)")]
    public Transform robotRoot;             // Drag your real physics fr3 here
    public ArticulationBody leftFinger;     // fr3_leftfinger
    public ArticulationBody rightFinger;    // fr3_rightfinger

    [Header("Dataset Output Settings")]
    public string outputDirectory = "D:/ML/RobotLearning/dataset_fr3_pick_place";
    public string languagePrompt = "pick up the red ball and place it on the green platform";
    public float recordHz = 15f;

    [Header("Runtime Status (Read-Only)")]
    public bool isRecording = false;
    public int recordedStepCount = 0;
    public int savedEpisodesCount = 0;

    private List<ArticulationBody> armJoints = new List<ArticulationBody>();
    private RenderTexture wristRT;
    private RenderTexture exteriorRT;
    private Texture2D readbackTex;

    private string currentTempEpisodeDir;
    private StringBuilder csvBuffer;
    private Coroutine recordCoroutine;

    void Start()
    {
        // Collect the 7 FR3 arm joints in exact kinematic order (link1 -> link7)
        string[] linkNames = {
            "fr3_link1", "fr3_link2", "fr3_link3", "fr3_link4",
            "fr3_link5", "fr3_link6", "fr3_link7"
        };

        var abMap = new Dictionary<string, ArticulationBody>();
        foreach (var ab in robotRoot.GetComponentsInChildren<ArticulationBody>())
            abMap[ab.name] = ab;

        foreach (string name in linkNames)
        {
            if (abMap.TryGetValue(name, out var ab))
                armJoints.Add(ab);
        }

        // Allocate reusable 224x224 RenderTextures for fast zero-GC image capture
        wristRT = new RenderTexture(imageWidth, imageHeight, 24, RenderTextureFormat.ARGB32);
        exteriorRT = new RenderTexture(imageWidth, imageHeight, 24, RenderTextureFormat.ARGB32);
        readbackTex = new Texture2D(imageWidth, imageHeight, TextureFormat.RGB24, false);

        Directory.CreateDirectory(outputDirectory);
        WriteDatasetMetadata();
    }

    public void BeginEpisode()
    {
        if (isRecording) return;

        recordedStepCount = 0;
        currentTempEpisodeDir = Path.Combine(outputDirectory, $"_temp_ep_{Guid.NewGuid():N}");
        Directory.CreateDirectory(Path.Combine(currentTempEpisodeDir, "wrist_image"));
        Directory.CreateDirectory(Path.Combine(currentTempEpisodeDir, "exterior_image"));

        csvBuffer = new StringBuilder();
        csvBuffer.AppendLine(
            "step,timestamp," +
            "state_j0,state_j1,state_j2,state_j3,state_j4,state_j5,state_j6,state_gripper," +
            "action_j0,action_j1,action_j2,action_j3,action_j4,action_j5,action_j6,action_gripper"
        );

        isRecording = true;
        recordCoroutine = StartCoroutine(RecordLoop());
    }

    public void EndEpisode(bool success)
    {
        if (!isRecording) return;

        isRecording = false;
        if (recordCoroutine != null)
            StopCoroutine(recordCoroutine);

        if (success && recordedStepCount > 10)
        {
            File.WriteAllText(Path.Combine(currentTempEpisodeDir, "trajectory.csv"), csvBuffer.ToString());

            string finalEpisodeDir = Path.Combine(outputDirectory, $"episode_{savedEpisodesCount:D5}");
            if (Directory.Exists(finalEpisodeDir))
                Directory.Delete(finalEpisodeDir, true);

            Directory.Move(currentTempEpisodeDir, finalEpisodeDir);
            savedEpisodesCount++;
            Debug.Log($"[Logger] Saved {finalEpisodeDir} ({recordedStepCount} steps at {recordHz}Hz)");
        }
        else
        {
            // Automatically discard failed episodes so dataset remains 100% clean
            if (Directory.Exists(currentTempEpisodeDir))
                Directory.Delete(currentTempEpisodeDir, true);
            Debug.LogWarning("[Logger] Discarded failed/short episode.");
        }
    }

    IEnumerator RecordLoop()
    {
        float interval = 1f / recordHz;
        float startTime = Time.time;

        while (isRecording)
        {
            CaptureStep(Time.time - startTime);
            yield return new WaitForSeconds(interval);
        }
    }

    void CaptureStep(float elapsedTime)
    {
        string stepName = $"frame_{recordedStepCount:D5}.png";

        // 1. Capture synchronized RGB frames from both cameras
        SaveCameraFrame(wristCamera, wristRT, Path.Combine(currentTempEpisodeDir, "wrist_image", stepName));
        SaveCameraFrame(exteriorCamera, exteriorRT, Path.Combine(currentTempEpisodeDir, "exterior_image", stepName));

        // 2. Read 8D Proprioceptive State (7 joint angles in radians + total gripper width in meters)
        float[] state = new float[8];
        float[] action = new float[8];

        for (int i = 0; i < 7; i++)
        {
            // Unity ArticulationBody jointPosition[0] is already in radians
            state[i] = armJoints[i].jointPosition[0];
            // Commanded xDrive.target is in degrees -> convert to radians for Pi-0
            action[i] = armJoints[i].xDrive.target * Mathf.Deg2Rad;
        }

        float leftPos = leftFinger.jointPosition[0];
        float rightPos = Mathf.Abs(rightFinger.jointPosition[0]);
        state[7] = leftPos + rightPos;

        float leftTarget = leftFinger.xDrive.target;
        float rightTarget = Mathf.Abs(rightFinger.xDrive.target);
        action[7] = leftTarget + rightTarget;

        // 3. Append row to trajectory CSV
        csvBuffer.AppendFormat(
            System.Globalization.CultureInfo.InvariantCulture,
            "{0},{1:F4}," +
            "{2:F6},{3:F6},{4:F6},{5:F6},{6:F6},{7:F6},{8:F6},{9:F6}," +
            "{10:F6},{11:F6},{12:F6},{13:F6},{14:F6},{15:F6},{16:F6},{17:F6}\n",
            recordedStepCount, elapsedTime,
            state[0], state[1], state[2], state[3], state[4], state[5], state[6], state[7],
            action[0], action[1], action[2], action[3], action[4], action[5], action[6], action[7]
        );

        recordedStepCount++;
    }

    void SaveCameraFrame(Camera cam, RenderTexture rt, string filePath)
    {
        RenderTexture prevTarget = cam.targetTexture;
        RenderTexture prevActive = RenderTexture.active;

        cam.targetTexture = rt;
        cam.Render();

        RenderTexture.active = rt;
        readbackTex.ReadPixels(new Rect(0, 0, imageWidth, imageHeight), 0, 0);
        readbackTex.Apply();

        cam.targetTexture = prevTarget;
        RenderTexture.active = prevActive;

        File.WriteAllBytes(filePath, readbackTex.EncodeToPNG());
    }

    void WriteDatasetMetadata()
    {
        string metaPath = Path.Combine(outputDirectory, "dataset_info.json");
        string json = "{\n" +
                      $"  \"fps\": {recordHz},\n" +
                      $"  \"image_size\": [{imageWidth}, {imageHeight}],\n" +
                      $"  \"state_dim\": 8,\n" +
                      $"  \"action_dim\": 8,\n" +
                      $"  \"prompt\": \"{languagePrompt}\"\n" +
                      "}";
        File.WriteAllText(metaPath, json);
    }

    void OnDestroy()
    {
        if (wristRT != null) wristRT.Release();
        if (exteriorRT != null) exteriorRT.Release();
    }
}
