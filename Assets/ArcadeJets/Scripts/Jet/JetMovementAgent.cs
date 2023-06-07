using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Policies;
using Random = UnityEngine.Random;

using ArcadeJets;

public class JetMovementAgent : Agent
{
    public GameObject jet;
    public bool useVecObs;
    
    Vector3 goalPosition;
    Rigidbody fighterjetRB;

    EnvironmentParameters m_ResetParams;

    Vector3 initPos;
    Quaternion initRot;
    Vector3 initGoalPosition;

    private float episode_start_time;

    public override void Initialize()
    {
        fighterjetRB = GetComponent<Rigidbody>();
        goalPosition = GameObject.Find("Goal").transform.position;

        initGoalPosition = goalPosition;
      
        initPos = gameObject.transform.position;
        initRot = gameObject.transform.rotation;

        SetResetParameters();

        Time.timeScale = 100f;
        // Vector3(-1814.80005,72.3000031,-2119)
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        if (useVecObs)
        {
            sensor.AddObservation(transform.position.x);
            sensor.AddObservation(transform.position.y);
            sensor.AddObservation(transform.position.z);

            sensor.AddObservation(transform.rotation.x);
            sensor.AddObservation(transform.rotation.y);
            sensor.AddObservation(transform.rotation.z);

            // sensor.AddObservation(fighterjetRB.velocity.x);
            // sensor.AddObservation(fighterjetRB.velocity.y);
            // sensor.AddObservation(fighterjetRB.velocity.z);

            sensor.AddObservation(GetComponent<StickInput>().Throttle);
            sensor.AddObservation(GetComponent<StickInput>().Pitch);
            sensor.AddObservation(GetComponent<StickInput>().Roll);
            sensor.AddObservation(GetComponent<StickInput>().Yaw);

            // sensor.AddObservation(1f);
            // sensor.AddObservation(1f);
            // sensor.AddObservation(1f);

            sensor.AddObservation((goalPosition - transform.position).x);
            sensor.AddObservation((goalPosition - transform.position).y);
            sensor.AddObservation((goalPosition - transform.position).z);
        }
    }

    public override void OnActionReceived(ActionBuffers actionBuffers)
    {
        float throttleTarget = Mathf.Clamp(actionBuffers.ContinuousActions[0], 0f, 1f);
        float stickInputX = Mathf.Clamp(actionBuffers.ContinuousActions[1], -1f, 1f);
        float stickInputY = Mathf.Clamp(actionBuffers.ContinuousActions[2], -1f, 1f);
        float stickInputZ = Mathf.Clamp(actionBuffers.ContinuousActions[3], -1f, 1f);

        GetComponent<StickInput>().Throttle = throttleTarget;
        // GetComponent<StickInput>().Throttle = 1f;
        GetComponent<StickInput>().Pitch = stickInputX;
        GetComponent<StickInput>().Roll =  stickInputY;
        GetComponent<StickInput>().Yaw =  stickInputZ;
        
        float distToGoal = Vector3.Distance(transform.position, goalPosition);

        // [(CL STEP 2) HIT TARGET!]
        // Vector3(-1872.30005,58.0999985,-2132.6001)  
        if (transform.position.y < 10){
            Debug.Log("Hit the Ground :(");
            SetReward(-1f);
            EndEpisode();
        } else if (distToGoal < 50) {
            Debug.Log("Reached Goal!");
            SetReward(1f);
            EndEpisode();
        } else {
            // speed + not hitting ground
            float reward = 0;
            reward -= 0.0001f * distToGoal;
            // reward += 0.00001f;
            if ((GetCumulativeReward() + reward) < -1f) {
                SetReward(-1f);
                EndEpisode();
            } else {
                SetReward(reward);
            }
        }
        
        // [(CL STEP 1) JUST FLY!]
        // if (transform.position.y < 10){
        //     Debug.Log("Hit the Ground :(");
        //     SetReward(-1f);
        //     EndEpisode();
        // } else {
        //     // speed + not hitting ground
        //     float reward = 0;
        //     reward += 0.0001f;
        //     SetReward(reward);
        // }
    }
    
    public override void OnEpisodeBegin()
    {
        //Reset the parameters when the Agent is reset.
        SetResetParameters();
    }

    public void SetResetParameters()
    {
        episode_start_time = Time.realtimeSinceStartup;
        fighterjetRB.velocity = new Vector3(0, 0, 0);
        transform.position = initPos;
        transform.rotation = initRot;
        
        // [(CL STEP 3) HIT RANDOM TARGET!]
        float newX = Random.Range(-30.0f, 30.0f) + initGoalPosition.x;
        float newY = Mathf.Clamp(Random.Range(-30.0f, 30.0f) + initGoalPosition.y, 40f, float.MaxValue);
        float newZ = Random.Range(-30.0f, 30.0f) + initGoalPosition.z;
        goalPosition = new Vector3(newX, newY, newZ);
        GameObject.Find("Goal").transform.position = goalPosition;
    }
}