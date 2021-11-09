using System;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;

/* Added to Entities that should be indexed by the SectorSystem. */
[Serializable]
public struct SectorTag : IComponentData
{
}
