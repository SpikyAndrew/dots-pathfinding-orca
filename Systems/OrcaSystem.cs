using System;
using AI;
using ECS.Components;
using ECS.MonoBehaviours;
using ECS.Structs;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Transforms;
using UnityEditor;
using UnityEngine;
using static Unity.Mathematics.math;
using float3 = Unity.Mathematics.float3;
using RigidTransform = Unity.Mathematics.RigidTransform;

// This file contains some source code derived from the RVO2 Library, available under the Apache 2 License at:
// https://gamma.cs.unc.edu/RVO2/
// The code was modified to utilize the Unity DOTS.

/* Calculates Velocity Obstacles for agent avoidance, then creates linear constraints for the LinearProgrammingSystem. */
[UpdateAfter(typeof(SectorSystem))] 
[UpdateAfter(typeof(DesiredVelocitySystem))]
public class OrcaSystem : SystemBase
{
    /* How far into the future (in seconds) should the agents look for collisions with other agents. */
    private const float TimeHorizon = 2f;
    private const float  InvTimeHorizon = 1 / TimeHorizon;
    /* How far into the future (in seconds) should the agents look for collisions when dealing with static obstacles such as walls. */
    private const float StaticTimeHorizon = 0.3f;
    private const float  StaticInvTimeHorizon = 1 / StaticTimeHorizon;
    /* Time within which agents should resolve collisions. */
    private const float CollisionResolveTime = 1/60f;

    
    protected override void OnUpdate()
    {
        CalculateVelocityConstraints();
        DebugDraw();
    }

    private void CalculateVelocityConstraints()
    {
        var sectorMultiHashMap = SectorSystem.SectorMultiHashMap;
        var navmesh = DotsNavmesh.Singleton;

        float3 center = navmesh.Center;
        float nodeSize = navmesh.NodeSize;
        int width = navmesh.Width;
        int depth = navmesh.Depth;
        NativeArray <NavmeshNode> navmeshNodes = navmesh.NodeArray;
        
        Entities.WithReadOnly(sectorMultiHashMap).WithReadOnly(navmeshNodes).WithAll<MovementTag>().ForEach((
            Entity entity,
            ref DynamicBuffer<LinearConstraint> linearConstraints,
            in CollisionFreeVelocity collisionFreeVelocity,
            in Translation translation,
            in AgentMovement agentMovement,
            in VelocityObstacle velocityObstacle,
            in DesiredVelocity desiredVelocity
        ) =>
        {
            /* Remove outdated constraints from previous frame. */
            linearConstraints.Clear();
            
            NativeList<Entity> nearbyAgents = SectorSystem.GetNearbyEntities(translation.Value, sectorMultiHashMap);
            var optimalVelocity = new float2(velocityObstacle.velocity.x, velocityObstacle.velocity.z);

            int2 agentNode = DotsNavmesh.WorldPositionToNode(center, nodeSize, width, depth, translation.Value);
            
            foreach (var offset in DotsNavmesh.NeighbourOffsets)
            {
                var node = navmeshNodes[DotsNavmesh.CalculateIndex(agentNode + offset, width)];
                if (node.isWalkable == false)
                {
                    /* Node is unwalkable, treat it as a static obstacle. */
                    var staticObstacle = new VelocityObstacle
                    {
                        velocity = float3.zero,
                        radius = nodeSize*0.5f
                    };
                    var obstacleTranslation = new Translation
                    {
                        Value = node.position
                    };

                    var constraint = CalculateLinearConstraint(translation, optimalVelocity, velocityObstacle,
                        obstacleTranslation, staticObstacle, true);
                    linearConstraints.Add(constraint);
                }
            }

            for (int i = 0; i < nearbyAgents.Length; i++)
            {
                var otherEntity = nearbyAgents[i];
                if (otherEntity != entity)
                {
                    var otherTranslation = GetComponent<Translation>(otherEntity);
                    var otherVelocityObstacle = GetComponent<VelocityObstacle>(otherEntity);
                    var line = CalculateLinearConstraint(translation, optimalVelocity, velocityObstacle, otherTranslation, otherVelocityObstacle, !HasComponent<MovementTag>(otherEntity));
                    linearConstraints.Add(line);
                }
            }
        }).ScheduleParallel();
    }

    private static LinearConstraint CalculateLinearConstraint(Translation translation, float2 optimalVelocity,
        VelocityObstacle velocityObstacle, Translation otherTranslation, VelocityObstacle otherVelocityObstacle, bool isStatic)
    {
        var line = new LinearConstraint();
        float2 u;

        var otherOptimalVelocity = new float2(otherVelocityObstacle.velocity.x, otherVelocityObstacle.velocity.z);

        var relativePosition3d = otherTranslation.Value - translation.Value;
        var relativePosition = new float2(relativePosition3d.x, relativePosition3d.z);
        var relativeVelocity = optimalVelocity - otherOptimalVelocity;

        float2 w = relativeVelocity - (isStatic ? StaticInvTimeHorizon : InvTimeHorizon) * relativePosition;

        /* Vector from cutoff center to relative velocity. */
        float wLengthSq = math.lengthsq(w);
        float dotProduct1 = math.dot(w, relativePosition);

        float combinedRadius = velocityObstacle.radius + otherVelocityObstacle.radius;
        float combinedRadiusSq = math.pow(combinedRadius, 2);

        if (math.lengthsq(relativePosition) > combinedRadiusSq)
        {
            if (dotProduct1 < 0.0f && math.pow(dotProduct1, 2) > combinedRadiusSq * wLengthSq)
            {
                /* Project on cut-off circle. */
                float wLength = math.sqrt(wLengthSq);
                Vector2 unitW = w / wLength;

                line.direction = new Vector2(unitW.y, -unitW.x);
                u = (combinedRadius * (isStatic ? StaticInvTimeHorizon : InvTimeHorizon) - wLength) * unitW;
            }
            else
            {
                float distSq = math.dot(relativePosition, relativePosition);

                /* Project on legs. */
                float leg = math.sqrt(distSq - combinedRadiusSq);

                if (math.determinant(new float2x2(relativePosition, w)) > 0.0f)
                {
                    /* Project on left leg. */
                    line.direction = new Vector2(relativePosition.x * leg - relativePosition.y * combinedRadius,
                                         relativePosition.x * combinedRadius + relativePosition.y * leg) /
                                     distSq;
                }
                else
                {
                    /* Project on right leg. */
                    line.direction =
                        -new Vector2(relativePosition.x * leg + relativePosition.y * combinedRadius,
                            -relativePosition.x * combinedRadius + relativePosition.y * leg) / distSq;
                }

                float dotProduct2 = math.dot(relativeVelocity, line.direction);
                u = dotProduct2 * line.direction - relativeVelocity;
            }
        }
        else
        {
            /*  Collision. Project on cut-off circle of time timeStep. */
            float invTimeStep = 1.0f / CollisionResolveTime;

            /* Vector from cutoff center to relative velocity. */
            w = relativeVelocity - invTimeStep * relativePosition;

            float wLength = math.length(w);
            Vector2 unitW = w / wLength;

            line.direction = new Vector2(unitW.y, -unitW.x);
            u = (combinedRadius * invTimeStep - wLength) * unitW;
        }
        
        if(!isStatic)
        {
            /* Agents avoid collision reciprocally (they share the responsibility equally). 
             * Therefore, we correct each agent's course by half the required amount. */
            u *= 0.5f;
        }
        
        /* Static obstacles are avoided more strictly. */
        line.isNegotiable = !isStatic;
        line.point = optimalVelocity + u;
        return line;
    }

    private void DebugDraw()
    {
        Entities.WithAll<DebugDrawTag, AgentMovement>().ForEach((
            in Translation translation,
            in DynamicBuffer<LinearConstraint> constraints,
            in DesiredVelocity desiredVelocity
            ) =>
        {
            foreach (var lc in constraints)
            {
                float3 orcaPoint = new float3(translation.Value.x + lc.point.x, translation.Value.y, translation.Value.z + lc.point.y);
                float3 orcaDirection = new float3(lc.direction.x, 0, lc.direction.y);
                Debug.DrawLine(orcaPoint+orcaDirection*1000, orcaPoint-orcaDirection*1000, lc.isNegotiable ? Color.yellow : Color.red);
            }
            Debug.DrawLine(translation.Value, translation.Value+desiredVelocity.value, Color.blue);
        }).WithoutBurst().Run();
    }
}