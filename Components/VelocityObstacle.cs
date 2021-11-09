using System;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;


/* Represents a velocity obstacle for the ORCA algorithm. */
[Serializable]
public struct VelocityObstacle : IComponentData
{
    public float3 velocity;
    public float radius;
}
