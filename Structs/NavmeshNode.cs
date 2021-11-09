using System;
using System.ComponentModel;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Transforms;

namespace ECS.Components
{
    /* Represents the immutable data of a square navigation node. */
    [Serializable]
    public struct NavmeshNode
    {
        public bool isWalkable;
        public int x;
        public int y;
        public float3 position;
    }
}
