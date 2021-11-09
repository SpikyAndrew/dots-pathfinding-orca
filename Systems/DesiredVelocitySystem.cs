using ECS.Components;
using ECS.Structs;
using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;
using Unity.Transforms;
using static Unity.Mathematics.math;

/* Determines the desired velocity of agents based on their current target node. */
public class DesiredVelocitySystem : SystemBase
{
    /* The minimum acceptable distance from the final node when checking if it has been reached */
    public const float EndReachedDistance = 0.5f;
    /* The minimum acceptable distance from the intermediate nodes when checking if they have been reached */
    public const float NodeReachedDistance = 1.25f;

    public const float EndReachedDistanceSquared = EndReachedDistance * EndReachedDistance;
    public const float NodeReachedDistanceSquared = NodeReachedDistance * NodeReachedDistance;

    protected override void OnUpdate()
    {
        Entities.WithAll<MovementTag>().ForEach((
            Entity entity,
            ref DesiredVelocity desiredVelocity,
            ref PathInstructions currentPathInstructions,
            in Translation translation,
            in AgentMovement agentMovement,
            in DynamicBuffer<PathNode> pathBufferElements
            ) =>
        {
            if (currentPathInstructions.nodeIndex != -1 && pathBufferElements.Length > 0)
            {            
                /* There is a node to walk towards. */
                var target = pathBufferElements[currentPathInstructions.nodeIndex].position;
                desiredVelocity.value = CalculateVelocity(target, translation.Value, agentMovement.movementSpeed);
                if (IsTargetNodeReached(target, translation))
                {
                    /* The agent has reached the next node. */
                    currentPathInstructions.nodeIndex -= 1;
                }
            }
            else
            {
                /* There is nowhere the agent should go. */
                desiredVelocity.value = new float3(0,0,0);
            }
        }).ScheduleParallel();
    }

    private static bool IsTargetNodeReached(float3 target, Translation translation)
    {
        return math.lengthsq(target - translation.Value) < NodeReachedDistanceSquared;
    }

    private static float3 CalculateVelocity(float3 target, float3 position, float movementSpeed)
    {
            float3 distance = target - position;
            float3 direction = math.normalize(distance);    
            return direction * movementSpeed;
    }
}