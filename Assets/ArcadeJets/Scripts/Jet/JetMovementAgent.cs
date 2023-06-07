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

        Time.timeScale = 5f;

        SetResetParameters();

        // bool isInference = GetComponent<BehaviorParameters>().BehaviorType == BehaviorType.InferenceOnly;
        // if (isInference) {
        //     Time.timeScale = 1f;
        // } else {
        //     Time.timeScale = 10f;
        // }

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

        // Working Reward Function --> no_grounding_speed_distance
        // if ((Time.realtimeSinceStartup - episode_start_time) > 10f){
        //     Debug.Log("Took too long :(");
        //     // AddReward(-30000f);
        //     // EndEpisode();
        // } else 
        
        // if (throttleTarget < 0) {
        //     AddReward(-500f);
        //     EndEpisode();
        //     return;
        // }
        
        // if ((Mathf.Abs(stickInputX) > 0.5) || (Mathf.Abs(stickInputY) > 0.5) || (Mathf.Abs(stickInputZ) > 0.5)) {
        //     EndEpisode();
        //     return;
        // }

        // if (Mathf.Abs(transform.rotation.y) > 90) {
        //     AddReward(-500f);
        //     EndEpisode();
        //     return;
        // }

        // [HIT TARGET!]
        // Vector3(-1872.30005,72.3000031,-2044.40002)
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
            reward -= 0.001f * distToGoal;
            reward += 0.001f;
            SetReward(reward);
        }
        
        // [JUST FLY!]
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

        // if (reward == 1) {
        //     EndEpisode();
        //     return;
        // }

        // // not working but just_fly_plus_highweight_target
        // if ((Time.realtimeSinceStartup - episode_start_time) > 10f){
        //     Debug.Log("Took too long :(");
        //     // AddReward(-30000f);
        //     EndEpisode();
        // } else if (distToGoal < 50) {
        //     Debug.Log("Reached Goal!");
        //     AddReward(5000000f);
        //     EndEpisode();
        // } else if (transform.position.y < 10){
        //     Debug.Log("Hit the Ground :(");
        //     AddReward(-5000000f);
        //     EndEpisode();
        // } else {
        //     // speed + not hitting ground
        //     float reward = 0;
        //     // reward += 1f * fighterjetRB.velocity.magnitude;
        //     // reward += 5000f * (1 / distToGoal);
        //     // reward -= 500f * distToGoal;
        //     // if (transform.position.y < 20) {
        //     //     reward -= 500;
        //     // }
        //     // reward += 500f * transform.position.y;
        //     reward -= 1;
        //     AddReward(reward);
        // }

        // Debug.Log((goalPosition - transform.position).x);
        // Debug.Log((goalPosition - transform.position).y);
        // Debug.Log((goalPosition - transform.position).z);        

        // if (Mathf.Abs(transform.rotation.x) > 0.5 || Mathf.Abs(transform.rotation.y) > 0.5 || Mathf.Abs(transform.rotation.z) > 0.5) {
        //     Debug.Log("Too Much Rotation");
        //     SetReward(-50000f);
        //     EndEpisode();            
        // }

        // float distToGoal = Vector3.Distance(transform.position, goalPosition);
        // if ((Time.realtimeSinceStartup - episode_start_time) > 5f){
        //     Debug.Log("Took too long :(");
        //     AddReward(-500f);
        //     EndEpisode();
        // } else if (transform.position.y < 10){
        //     Debug.Log("Hit the Ground :(");
        //     AddReward(-500f);
        //     EndEpisode();
        // } else if (distToGoal < 2) {
        //     Debug.Log("Reached Goal!");
        //     AddReward(500f);
        //     EndEpisode();
        // } else {
        //     float reward = -0.1f * Mathf.Pow(distToGoal, 2);
        //     // reward += (1000 * transform.position.y);
        //     // reward += (10 * fighterjetRB.velocity.magnitude);
        //     // if (transform.rotation.y > 150) {
        //     //    SetReward(-4000f); 
        //     // }
        //     // float reward = 0f;
        //     SetReward(reward);
        // }
    }
    
    // public override void OnActionReceived(ActionBuffers actionBuffers)
    // {
    //     float throttleTarget = Mathf.Clamp(actionBuffers.ContinuousActions[0], 0f, 1f);
    //     float stickInputX = Mathf.Clamp(actionBuffers.ContinuousActions[1], -1f, 1f);
    //     float stickInputY = Mathf.Clamp(actionBuffers.ContinuousActions[2], -1f, 1f);
    //     float stickInputZ = Mathf.Clamp(actionBuffers.ContinuousActions[3], -1f, 1f);

    //     GetComponent<StickInput>().Pitch = stickInputX;
    //     GetComponent<StickInput>().Roll =  stickInputY;
    //     GetComponent<StickInput>().Yaw =  stickInputZ;
    //     GetComponent<StickInput>().Throttle = throttleTarget;

    //     float distToGoal = Vector3.Distance(transform.position, goalPosition);
    //     if (transform.position.y < 5){
    //         Debug.Log("Hit the Ground :(");
    //         SetReward(-6000f);
    //         EndEpisode();
    //     } else if (distToGoal < 2) {
    //         Debug.Log("Reached Goal!");
    //         SetReward(1000f);
    //         EndEpisode();
    //     } else if (transform.rotation.y > 150) {
    //         Debug.Log("Too much rotation :(");
    //         SetReward(-4000f);
    //         EndEpisode();
    //     } else {
    //         float distance_reward = -10f * Vector3.Distance(transform.position, goalPosition);
    //         float velocity_bonus;
    //         if ((fighterjetRB.velocity.x > 80 || fighterjetRB.velocity.z > 80) && (fighterjetRB.velocity.y < 10) && (fighterjetRB.velocity.y >= 0)) {
    //             velocity_bonus = 200f;
    //         } else if ((fighterjetRB.velocity.y > 100) || ((fighterjetRB.velocity.y < -10) && (transform.position.y < 30))) {
    //             velocity_bonus = -500f;
    //         } else {
    //             velocity_bonus = 0f
    //         }
    //         float reward = distance_reward + velocity_bonus;
    //         SetReward(reward);
    //     }
    // }

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
        

        float newX = Random.Range(-80.0f, 80.0f) + initGoalPosition.x;
        float newY = Mathf.Clamp(Random.Range(-80.0f, 80.0f) + initGoalPosition.y, 40f, float.MaxValue);
        float newZ = Random.Range(-80.0f, 80.0f) + initGoalPosition.z;

        GameObject.Find("Goal").transform.position = new Vector3(newX, newY, newZ);

        // bool isInference = GetComponent<BehaviorParameters>().BehaviorType == BehaviorType.InferenceOnly;
        // if (isInference) {
        //     Time.timeScale = 1f;
        // } else {
        //     Time.timeScale = 10f;
        // }

        // Time.timeScale = 1f;
    }
}