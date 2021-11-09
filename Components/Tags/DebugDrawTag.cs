using System;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;

/* Added to agent Entities that should have their collision avoidance data drawn during debugging. */
[Serializable]
public struct DebugDrawTag : IComponentData
{
    
}
