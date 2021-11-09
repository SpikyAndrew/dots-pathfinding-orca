using System;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;


/* Represents the desired velocity of an agent, i.e. one the agent would have if there were no obstacles. */
[Serializable]
public struct DesiredVelocity : IComponentData
{
    public float3 value;
}
