using System.Collections.Generic;
using System.Text;
using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;
using Unity.Transforms;
using UnityEditor;
using UnityEngine;
using static Unity.Mathematics.math;

/* Indexes Entities based on their position. Each indexed Entity is assigned to a square area called a Sector. 
 * This allows the OrcaSystem to only calculate collision avoidance for entities that are nearby each other. */
[UpdateBefore(typeof(OrcaSystem))]
public class SectorSystem : SystemBase
{
    private const float SectorSize = 1.5f;
    
    private static readonly int2[] AdjacentSectorKeyOffsets = new[]
    {
         new int2( 0,  0),
         new int2( 1,  0),
         new int2(-1,  0),
         new int2( 0,  1),
         new int2( 1,  1),
         new int2(-1,  1),
         new int2( 0, -1),
         new int2( 1, -1),
         new int2(-1, -1),
    };

    private static NativeMultiHashMap<int2, Entity> _sectorMultiHashMap;


    public static NativeMultiHashMap<int2, Entity> SectorMultiHashMap => _sectorMultiHashMap;

    public static NativeList<Entity> GetNearbyEntities(float3 position,
        NativeMultiHashMap<int2, Entity> sectorMultiHashMap)
    {
        int2 sectorKey = GetKeyFromPosition(position);
        var list = new NativeList<Entity>(Allocator.Temp); 
        for (int i = 0; i < AdjacentSectorKeyOffsets.Length; i++)
        {
            list.AddRange(GetEntitiesInSector(sectorKey+AdjacentSectorKeyOffsets[i], sectorMultiHashMap));
        }
        return list;
    }
    
    protected override void OnCreate()
    {
        _sectorMultiHashMap = new NativeMultiHashMap<int2, Entity>(0, Allocator.Persistent);
        base.OnCreate();
    }

    protected override void OnDestroy()
    {
        _sectorMultiHashMap.Dispose();
        base.OnDestroy();
    }

    protected override void OnUpdate()
    {
        /* Uncomment to draw the sector hovered by the mouse cursor. */
        //DebugDrawHoveredSector();

        EntityQuery query = GetEntityQuery(typeof(Translation), typeof(SectorTag));

        _sectorMultiHashMap.Clear();
        if(_sectorMultiHashMap.Capacity < query.CalculateEntityCount())
            _sectorMultiHashMap.Capacity = query.CalculateEntityCount();
        
        var concurrentHashMap = _sectorMultiHashMap.AsParallelWriter();
        Entities.WithAll<SectorTag>().WithNativeDisableContainerSafetyRestriction(concurrentHashMap).ForEach(
            (Entity entity, ref Translation translation) =>
            {
                int2 hashMapKey = GetKeyFromPosition(translation.Value);
                concurrentHashMap.Add(hashMapKey, entity);
            }).ScheduleParallel();
        
    }
    
    private void DebugDrawSector(float3 position)
    {
        Vector3 lowerLeft = new Vector3(math.floor(position.x / SectorSize) * SectorSize,
            0, math.floor(position.z / SectorSize) * SectorSize);
        Debug.DrawLine(lowerLeft, lowerLeft + Vector3.right * SectorSize);
        Debug.DrawLine(lowerLeft, lowerLeft + Vector3.forward * SectorSize);
        Debug.DrawLine(lowerLeft + Vector3.forward * SectorSize, lowerLeft + (Vector3.forward+Vector3.right) * SectorSize);
        Debug.DrawLine(lowerLeft + Vector3.right * SectorSize, lowerLeft + (Vector3.forward+Vector3.right) * SectorSize);
    }

    private void DebugDrawHoveredSector()
    {
        Ray ray = Camera.main.ScreenPointToRay(Input.mousePosition);
        RaycastHit hit;
        if (Physics.Raycast(ray, out hit))
        {
            DebugDrawSector(hit.point);
        }

        var enumerator = _sectorMultiHashMap.GetValuesForKey(GetKeyFromPosition(hit.point));
        int count = 0;
        while (enumerator.MoveNext())
        {
            count++;
        }
    }    

    private static int2 GetKeyFromPosition(float3 pos)
    {
        return new int2((int)(math.floor(pos.x / SectorSize)), (int)(math.floor(pos.z / SectorSize)));
    }
    
    
    private static NativeList<Entity> GetEntitiesInSector(int2 sectorKey,
        NativeMultiHashMap<int2, Entity> sectorMultiHashMap)
    {
        var list = new NativeList<Entity>(Allocator.Temp);
        var values = sectorMultiHashMap.GetValuesForKey(sectorKey);
        while (values.MoveNext())
        {
            list.Add(values.Current);
        }
        return list;
    }
} 