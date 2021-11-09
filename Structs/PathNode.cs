using Unity.Entities;
using Unity.Mathematics;

namespace ECS.Structs
{
    /* Represents a single square node of an agent's path. */
    [InternalBufferCapacity(32)]
    public struct PathNode : IBufferElementData
    {
        public int x;
        public int y;
        public float3 position;
    }
}
