using System;
using Unity.Entities;
using Unity.Mathematics;

namespace ECS.Components
{
    /* Contains the data related to a pathfinding request. */
    [Serializable]
    public struct PathfindingRequest : IComponentData
    {
        public int2 startPosition;
        public int2 endPosition;
    }
}
