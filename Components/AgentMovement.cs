using System;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;

/* Contains an agent's data related to its movement capabilities. */
[Serializable]
public struct AgentMovement : IComponentData
{
    public readonly float movementSpeed;


    public AgentMovement(float movementSpeed)
    {
        this.movementSpeed = movementSpeed;
    }
}
