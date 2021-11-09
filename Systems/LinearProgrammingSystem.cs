﻿using System;
 using ECS.Components;
 using ECS.Structs;
 using Unity.Collections;
 using Unity.Entities;
 using Unity.Mathematics;
 using UnityEngine;

 
// This file contains source code derived from the RVO2 Library, available under the Apache 2 License at:
// https://gamma.cs.unc.edu/RVO2/
// The code was modified to utilize the Unity DOTS.
 
/* Implements a linear programming algorithm for finding an optimal, collision-free velocity for agents. */
 [UpdateAfter(typeof(OrcaSystem))]
 public class LinearProgrammingSystem : SystemBase
{
    private const float Epsilon = 0.00001f;

    protected override void OnUpdate()
    {
        Entities.WithAll<MovementTag>().ForEach((
                ref CollisionFreeVelocity velocity,    
                ref DynamicBuffer<LinearConstraint> constraints,
                in DesiredVelocity desiredVelocity,
                in AgentMovement agentMovement
            ) =>
            {
                float2 currentSolution = new float2(desiredVelocity.value.x, desiredVelocity.value.z);
                int lineNumber = LinearProgram2D(constraints.AsNativeArray(), agentMovement, ref currentSolution, new float2(desiredVelocity.value.x, desiredVelocity.value.z));
                if (lineNumber < constraints.Length)
                {
                    LinearProgram3D(constraints, agentMovement, ref currentSolution, lineNumber);
                }
                if (math.length(currentSolution) > agentMovement.movementSpeed * 1.1f)
                {
                    currentSolution = currentSolution * agentMovement.movementSpeed / math.length(currentSolution);
                }
                velocity.value = new float3(currentSolution.x, desiredVelocity.value.y, currentSolution.y);
            }
        ).ScheduleParallel();
    }

    private static int LinearProgram2D(NativeArray<LinearConstraint> constraints,
        AgentMovement agentMovement, ref float2 currentSolution, float2 desiredVelocity)
    {
        for (var i = 0; i < constraints.Length; i++)
        {
            if (math.determinant(new float2x2(constraints[i].direction, constraints[i].point - currentSolution)) > 0.0f)
            {
                bool isFeasible = AddConstraint(agentMovement.movementSpeed, desiredVelocity,
                    constraints, i, ref currentSolution);
                if (!isFeasible)
                {
                    return i;
                }
            }
        }
        return constraints.Length;
    }

    private static void LinearProgram3D(DynamicBuffer<LinearConstraint> lines, AgentMovement agentMovement, ref float2 currentSolution, int beginLine)
    {
        float distance = 0.0f;

        for (int i = beginLine; i < lines.Length; ++i)
        {
            if (math.determinant(new float2x2(lines[i].direction, lines[i].point - currentSolution)) > distance)
            {
                /* Result does not satisfy constraint of line i. */
                NativeArray<LinearConstraint> projLines = new NativeArray<LinearConstraint>(lines.Length, Allocator.Temp);
                
                for (int j = 0; j < i; ++j)
                {
                    if (lines[j].isNegotiable == false)
                    {
                        projLines[j] = lines[j];
                        continue;
                    }
                    LinearConstraint line = new LinearConstraint();

                    float determinant = math.determinant(new float2x2(lines[i].direction, lines[j].direction));

                    if (math.abs(determinant) <= Epsilon)
                    {
                        /* Line i and line j are parallel. */
                        if (math.dot(lines[i].direction, lines[j].direction) > 0.0f)
                        {
                            /* Line i and line j point in the same direction. */
                            continue;
                        }
                        else
                        {
                            /* Line i and line j point in opposite direction. */
                            line.point = 0.5f * (lines[i].point + lines[j].point);
                        }
                    }
                    else
                    {
                        line.point = lines[i].point + (math.determinant(new float2x2(lines[j].direction, lines[i].point - lines[j].point) / determinant)) * lines[i].direction;
                    }

                    line.direction = math.normalize(lines[j].direction - lines[i].direction);
                    projLines[j] = line;
                }
                
                Vector2 tempResult = currentSolution;
                if (LinearProgram2D(projLines, agentMovement, ref currentSolution, new float2(-lines[i].direction.y, lines[i].direction.x)) < projLines.Length)
                {
                    currentSolution = tempResult;
                }

                distance = math.determinant(new float2x2(lines[i].direction, lines[i].point - currentSolution));
            }
        }
    }

    private static bool AddConstraint(float maxSpeed, float2 desiredVelocity, in NativeArray<LinearConstraint> constraints, int lineNumber, ref float2 currentSolution)
    {
        var lc = constraints[lineNumber];

        currentSolution = FindOptimalVelocityInSection(desiredVelocity, lc);

        float dotProduct = math.dot(lc.point, lc.direction);
        float discriminant = math.pow(dotProduct,2) + math.pow(maxSpeed,2) - math.length(lc.point);

        if (discriminant < 0.0f)
        {
            return false;
        }

        float sqrtDiscriminant = math.sqrt(discriminant);
        float tLeft = -dotProduct - sqrtDiscriminant;
        float tRight = -dotProduct + sqrtDiscriminant;

        for (int i = 0; i < lineNumber; ++i)
        {
            float denominator = math.determinant(new float2x2(lc.direction, constraints[i].direction));
            float numerator = math.determinant(new float2x2(constraints[i].direction, lc.point - constraints[i].point));

            if (math.abs(denominator) <= Epsilon)
            {
                /* Lines lineNo and i are (almost) parallel. */
                if (numerator < 0.0f)
                {
                    return false;
                }

                continue;
            }

            float t = numerator / denominator;

            if (denominator >= 0.0f)
            {
                /* Line i bounds line lineNo on the right. */
                tRight = Math.Min(tRight, t);
            }
            else
            {
                /* Line i bounds line lineNo on the left. */
                tLeft = Math.Max(tLeft, t);
            }

            if (tLeft > tRight)
            {
                return false;
            }
        }

        {
            /* Optimize closest point. */
            float t = math.dot(lc.direction, (new float2(desiredVelocity.x, desiredVelocity.y) - lc.point));

            if (t < tLeft)
            {
                currentSolution = lc.point + tLeft * lc.direction;
            }
            else if (t > tRight)
            {
                currentSolution = lc.point + tRight * lc.direction;
            }
            else
            {
                currentSolution = lc.point + t * lc.direction;
            }
        }

        if (math.length(currentSolution) > maxSpeed * 1.2f)
        {
            currentSolution = currentSolution * maxSpeed / math.length(currentSolution);
        }

        return true;
    }

    private static float2 FindOptimalVelocityInSection(float2 desiredVelocity, LinearConstraint lc)
    {
        var pointDistance = desiredVelocity - lc.point;
        var dotProduct = math.dot(pointDistance, lc.direction);
        var result = lc.point + lc.direction * dotProduct;

        return new float2(result.x, result.y);
    }
}
