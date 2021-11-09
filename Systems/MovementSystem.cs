using System;
using Boo.Lang;
using ECS.Components;
using ECS.MonoBehaviours;
using ECS.Structs;
using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;
using Unity.Transforms;
using UnityEngine;
using UnityEngine.Analytics;

namespace ECS.Systems
{
    /* Handles changing the agents' positions based on their final velocities calculated by the LinearProgrammingSystem. */
    public class MovementSystem : SystemBase
    {
        protected override void OnUpdate()
        {
            /* Reset the velocity obstacle data. This is to make sure that the agents which aren't updated in this frame
             * won't be treated as still moving due to old data. */
            ResetVelocities();
            
            MoveAgents(Time.DeltaTime);
        }

        private void MoveAgents(float deltaTime)
        {
            Entities
                .WithAll<MovementTag>()
                .ForEach(
                    (ref VelocityObstacle velocityObstacle, ref Translation translation,
                        in CollisionFreeVelocity collisionFreeVelocity, in DesiredVelocity desiredVelocity, in AgentMovement agentMovement) =>
                    {
                        if (float.IsNaN(collisionFreeVelocity.value.x * collisionFreeVelocity.value.y * collisionFreeVelocity.value.z))
                        {
                            Debug.LogError("Velocity is invalid - it contains a NaN. ");
                        }
                        else
                        {
                            float3 targetPosition = translation.Value + collisionFreeVelocity.value * deltaTime;
                            translation.Value = targetPosition;
                            velocityObstacle.velocity = collisionFreeVelocity.value;
                        }
                        
                    })
                .ScheduleParallel();
        }

        private void ResetVelocities()
        {
            Entities
                .WithNone<MovementTag>()
                .ForEach((ref VelocityObstacle vo) => { vo.velocity = new float3(0, 0, 0); })
                .ScheduleParallel();
        }
    }
}