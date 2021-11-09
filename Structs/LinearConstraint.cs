using Unity.Entities;
using Unity.Mathematics;

namespace ECS.Structs
{
    [InternalBufferCapacity(32)]
    /* Represents a single constraint for linear programming. */
    public struct LinearConstraint : IBufferElementData
    {
        public float2 point;
        public float2 direction;
        public bool isNegotiable;
    }
}
