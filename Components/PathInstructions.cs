using System;
using Unity.Entities;
using Unity.Mathematics;

namespace ECS.Components
{
    /* Metadata for a path. */
    [Serializable]
    public struct PathInstructions : IComponentData
    {
        /* Index of the currently followed node. */
        public int nodeIndex;
    }
}
