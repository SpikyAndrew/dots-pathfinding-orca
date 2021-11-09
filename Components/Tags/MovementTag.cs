using System;
using Unity.Entities;

namespace ECS.Components
{
    /* Added to agent Entities that should be able to move. */
    [Serializable]
    public struct MovementTag : IComponentData
    {
    }
}
