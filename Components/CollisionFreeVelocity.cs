using System;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;

/* Represents an velocity chosen for an agent based on its desired velocity and surrounding obstacles. */
[Serializable]
public struct CollisionFreeVelocity : IComponentData
{
   public float3 value;
}
