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

    private float episode_start_time;

    public override void Initialize()
    {
        fighterjetRB = GetComponent<Rigidbody>();
        goalPosition = GameObject.Find("Goal").transform.position;
      
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

        // Time.timeScale = 100f;
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

            sensor.AddObservation(fighterjetRB.velocity.x);
            sensor.AddObservation(fighterjetRB.velocity.y);
            sensor.AddObservation(fighterjetRB.velocity.z);

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

        GetComponent<StickInput>().Pitch = stickInputX;
        GetComponent<StickInput>().Roll =  stickInputY;
        GetComponent<StickInput>().Yaw =  stickInputZ;
        GetComponent<StickInput>().Throttle = throttleTarget;
        // GetComponent<StickInput>().Throttle = 1f;
        
        float distToGoal = Vector3.Distance(transform.position, goalPosition);


        // Working Reward Function
        if ((Time.realtimeSinceStartup - episode_start_time) > 10f){
            Debug.Log("Took too long :(");
            // AddReward(-30000f);
            EndEpisode();
        } else if (transform.position.y < 10){
            Debug.Log("Hit the Ground :(");
            AddReward(-50000f);
            EndEpisode();
        } else {
            // speed + not hitting ground
            float reward = 0;
            reward += 0.2f * fighterjetRB.velocity.magnitude;
            reward -= 0.3f * distToGoal;
            AddReward(reward);
        }
        

        // if ((Time.realtimeSinceStartup - episode_start_time) > 10f){
        //     Debug.Log("Took too long :(");
        //     // AddReward(-30000f);
        //     EndEpisode();
        // } else if (distToGoal < 100) {
        //     Debug.Log("Reached Goal!");
        //     AddReward(50000f);
        //     EndEpisode();
        // } else if (transform.position.y < 10){
        //     Debug.Log("Hit the Ground :(");
        //     AddReward(-500f);
        //     // EndEpisode();
        // } else {
        //     // speed + not hitting ground
        //     float reward = 0;
        //     // reward += 1f * fighterjetRB.velocity.magnitude;
        //     // reward -= 1f * distToGoal;
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

        // bool isInference = GetComponent<BehaviorParameters>().BehaviorType == BehaviorType.InferenceOnly;
        // if (isInference) {
        //     Time.timeScale = 1f;
        // } else {
        //     Time.timeScale = 10f;
        // }

        // Time.timeScale = 1f;
    }
}