﻿using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
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

    public override void Initialize()
    {
        fighterjetRB = GetComponent<Rigidbody>();
        goalPosition = GameObject.Find("Goal").transform.position;
      
        initPos = gameObject.transform.position;
        initRot = gameObject.transform.rotation;

        SetResetParameters();
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

        float distToGoal = Vector3.Distance(transform.position, goalPosition);
        if (transform.position.y < 5){
            Debug.Log("Hit the Ground :(");
            SetReward(-500000f);
            EndEpisode();
        } else {
            float reward = -0.000001f * distToGoal * distToGoal;
            // float reward = 0f;
            SetReward(reward);
        }
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
        fighterjetRB.velocity = new Vector3(0, 0, 0);
        transform.position = initPos;
        transform.rotation = initRot;
    }
}