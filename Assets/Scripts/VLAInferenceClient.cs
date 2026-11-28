using System;
using System.IO;
using System.Net.Sockets;
using System.Threading;
using UnityEngine;

public class VLAInferenceClient : MonoBehaviour
{
    [Header("Connection")]
    public string host = "127.0.0.1";
    public int port = 5005;
    public float controlHz = 15f;

    [Header("Cameras (Must Match Training 224x224)")]
    public Camera exteriorCamera;
    public Camera wristCamera;
    public int imageSize = 224;

    [Header("Franka FR3 Articulation Bodies (Visible fr3 Robot)")]
    public ArticulationBody[] armJoints = new ArticulationBody[7];
    public ArticulationBody leftFinger;
    public ArticulationBody rightFinger;
    public bool invertRightFingerTarget = true;

    [Header("Motor Drive Tuning (Matches Controller.cs)")]
    public float stiffness = 10000f;
    public float damping = 1000f;
    public float forceLimit = 1000f;
    public float jointFriction = 1f;

    private TcpClient _client;
    private NetworkStream _stream;
    private Thread _netThread;
    private volatile bool _running;

    private RenderTexture _extRT;
    private RenderTexture _wristRT;
    private Texture2D _extTex;
    private Texture2D _wristTex;
    private float _timer;
    private bool _needsReset = true;

    private readonly object _lock = new object();
    private byte[] _pendingPacket;
    private float[] _latestAction;
    private bool _hasNewAction;

    [Header("Ball & Grasp (Matches EpisodeManager)")]
    public Rigidbody ballRigidbody;
    public Transform endEffectorTCP;
    public bool attachBallOnGrasp = true;
    public float graspThreshold = 0.076f; // Midpoint between open (0.08) and closed (0.073)

    [Header("Spawn & Platform Bounds (Matches EpisodeManager)")]
    public Transform spawnAreaBox;
    public Transform dropperPlatform;
    public float ballRestY = 0.0375f;
    public float minDistanceFromPlatform = 0.15f;
    public float platformSuccessRadius = 0.12f; // Or use dropperPlatform Collider bounds
    public float settleDelayAfterRelease = 1.5f;
    public float episodeTimeout = 30f;

    private bool _isHoldingBall = false;
    private bool _wasGripperClosed = false;
    private float _episodeTimer = 0f;
    private float _releaseTimer = -1f;

    void Start()
    {
        if (ballRigidbody != null)
            ballRestY = ballRigidbody.transform.position.y;

        ResetAndSpawnBall();

        if (armJoints == null || armJoints.Length != 7 || armJoints[0] == null)
        {
            Debug.LogError("[VLA] Assign all 7 visible fr3_link1..fr3_link7 ArticulationBodies!");
            return;
        }

        // Configure ArticulationBody drives identically to Controller.cs
        for (int i = 0; i < 7; i++)
        {
            armJoints[i].jointFriction = jointFriction;
            var drive = armJoints[i].xDrive;
            drive.stiffness = stiffness;
            drive.damping = damping;
            drive.forceLimit = forceLimit;
            armJoints[i].xDrive = drive;
        }

        foreach (var finger in new[] { leftFinger, rightFinger })
        {
            if (finger == null) continue;
            finger.jointFriction = jointFriction;
            var fDrive = finger.xDrive;
            fDrive.stiffness = stiffness;
            fDrive.damping = damping;
            fDrive.forceLimit = forceLimit;
            finger.xDrive = fDrive;
        }

        _extRT = new RenderTexture(imageSize, imageSize, 24);
        _wristRT = new RenderTexture(imageSize, imageSize, 24);
        _extTex = new Texture2D(imageSize, imageSize, TextureFormat.RGB24, false);
        _wristTex = new Texture2D(imageSize, imageSize, TextureFormat.RGB24, false);

        _client = new TcpClient(host, port);
        _client.NoDelay = true;
        _stream = _client.GetStream();

        _running = true;
        _netThread = new Thread(NetworkLoop) { IsBackground = true };
        _netThread.Start();

        Debug.Log("[VLA] Motors initialized (stiffness=10000) & connected to Python bridge.");
    }

    void ResetAndSpawnBall()
    {
        _isHoldingBall = false;
        _wasGripperClosed = false;
        _episodeTimer = 0f;
        _releaseTimer = -1f;

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
                UnityEngine.Random.Range(-halfX, halfX),
                0f,
                UnityEngine.Random.Range(-halfZ, halfZ)
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

    public void NotifyEpisodeReset()
    {
        _needsReset = true;
    }

    private void NetworkLoop()
    {
        byte[] actionBytes = new byte[32];
        try
        {
            while (_running && _client != null && _client.Connected)
            {
                byte[] packetToSend = null;
                lock (_lock)
                {
                    if (_pendingPacket != null)
                    {
                        packetToSend = _pendingPacket;
                        _pendingPacket = null;
                    }
                }

                if (packetToSend == null)
                {
                    Thread.Sleep(2);
                    continue;
                }

                _stream.Write(packetToSend, 0, packetToSend.Length);

                int read = 0;
                while (read < 32 && _running)
                {
                    int n = _stream.Read(actionBytes, read, 32 - read);
                    if (n <= 0) throw new Exception("Bridge disconnected.");
                    read += n;
                }

                if (read == 32)
                {
                    float[] action = new float[8];
                    Buffer.BlockCopy(actionBytes, 0, action, 0, 32);
                    lock (_lock)
                    {
                        _latestAction = action;
                        _hasNewAction = true;
                    }
                }
            }
        }
        catch (Exception ex)
        {
            if (_running) Debug.LogWarning($"[VLA] Network thread stopped: {ex.Message}");
        }
    }

    void FixedUpdate()
    {
        if (!_running) return;

        float[] actionToApply = null;
        lock (_lock)
        {
            if (_hasNewAction)
            {
                actionToApply = _latestAction;
                _hasNewAction = false;
            }
        }
        // Debug.Log($"[VLA]: Action received: {actionToApply[0]:F3}");
        if (actionToApply != null)
        {
            string formatted = string.Join(", ", System.Array.ConvertAll(actionToApply, x => x.ToString("F3")));

            for (int i = 0; i < 7; i++)
            {
                var drive = armJoints[i].xDrive;
                drive.target = actionToApply[i] * Mathf.Rad2Deg;
                armJoints[i].xDrive = drive;
            }

            float halfGripper = Mathf.Clamp(actionToApply[7] * 0.5f, 0f, 0.04f);
            var leftDrive = leftFinger.xDrive;
            leftDrive.target = halfGripper;
            leftFinger.xDrive = leftDrive;

            var rightDrive = rightFinger.xDrive;
            rightDrive.target = invertRightFingerTarget ? -halfGripper : halfGripper;
            rightFinger.xDrive = rightDrive;

            float commandedGripperWidth = actionToApply[7]; // 0.08 = open, 0.073 = closed
            bool isGripperClosed = commandedGripperWidth < graspThreshold;

            // Trigger grasp on open -> closed transition
            if (isGripperClosed && !_wasGripperClosed && !_isHoldingBall && ballRigidbody != null && endEffectorTCP != null)
            {
                float graspError = Vector3.Distance(endEffectorTCP.position, ballRigidbody.position);
                if (attachBallOnGrasp && graspError < 0.05f)
                {
                    ballRigidbody.isKinematic = true;
                    ballRigidbody.transform.SetParent(endEffectorTCP);
                    ballRigidbody.transform.localPosition = Vector3.zero;
                    _isHoldingBall = true;
                    _releaseTimer = -1f;
                    Debug.Log("[VLA] Ball grasped and snapped to TCP.");
                }
                else if (graspError >= 0.05f)
                {
                    Debug.LogWarning($"[VLA] Missed grasp! TCP error = {graspError:F3}m at ball pos {ballRigidbody.position}");
                }
            }
            // Trigger release on closed -> open transition
            else if (!isGripperClosed && _isHoldingBall)
            {
                ballRigidbody.transform.SetParent(null);
                ballRigidbody.isKinematic = false;
                _isHoldingBall = false;
                _releaseTimer = 0f;
                Debug.Log("[VLA] Ball released.");
            }

            _wasGripperClosed = isGripperClosed;
        }

        _timer += Time.fixedDeltaTime;
        if (_timer < 1f / controlHz) return;
        _timer = 0f;

        lock (_lock)
        {
            if (_pendingPacket != null) return;
        }

        float[] state = new float[8];
        for (int i = 0; i < 7; i++)
        {
            state[i] = armJoints[i].jointPosition[0];
        }
        state[7] = leftFinger.jointPosition[0] + Mathf.Abs(rightFinger.jointPosition[0]);

        byte[] extPng = CaptureCameraPng(exteriorCamera, _extRT, _extTex);
        byte[] wristPng = CaptureCameraPng(wristCamera, _wristRT, _wristTex);

        using (var ms = new MemoryStream())
        using (var bw = new BinaryWriter(ms))
        {
            bw.Write((byte)(_needsReset ? 1 : 0));
            _needsReset = false;
            for (int i = 0; i < 8; i++) bw.Write(state[i]);
            bw.Write((uint)extPng.Length);
            bw.Write((uint)wristPng.Length);
            bw.Write(extPng);
            bw.Write(wristPng);

            lock (_lock)
            {
                _pendingPacket = ms.ToArray();
            }
        }

        // reset the ball spawn position
        _episodeTimer += Time.fixedDeltaTime;

        if (!_isHoldingBall && _releaseTimer >= 0f)
        {
            _releaseTimer += Time.fixedDeltaTime;
        }

        // Grasping the ball and dropping over the greenplatform.
        bool landedOnPlatform = false;
        if (!_isHoldingBall && _releaseTimer >= settleDelayAfterRelease && ballRigidbody != null && dropperPlatform != null)
        {
            float distToPad = Vector2.Distance(
                new Vector2(ballRigidbody.position.x, ballRigidbody.position.z),
                new Vector2(dropperPlatform.position.x, dropperPlatform.position.z)
            );
            landedOnPlatform = distToPad <= platformSuccessRadius;
        }

        // respawn the ball location after 25 seconds, if the arm failed to pick it up.
        if (landedOnPlatform || _episodeTimer >= episodeTimeout)
        {
            Debug.Log(landedOnPlatform
                ? "[VLA] Placed on green platform! Respawning ball..."
                : "[VLA] Episode timeout. Respawning ball...");

            ResetAndSpawnBall();
            NotifyEpisodeReset();
        }
    }

    private byte[] CaptureCameraPng(Camera cam, RenderTexture rt, Texture2D tex)
    {
        RenderTexture prev = RenderTexture.active;
        cam.targetTexture = rt;
        cam.Render();
        RenderTexture.active = rt;
        tex.ReadPixels(new Rect(0, 0, imageSize, imageSize), 0, 0);
        tex.Apply();
        cam.targetTexture = null;
        RenderTexture.active = prev;
        return tex.EncodeToPNG();
    }

    void OnDestroy()
    {
        _running = false;
        _stream?.Close();
        _client?.Close();
        if (_netThread != null && _netThread.IsAlive) _netThread.Join(200);
    }

}
