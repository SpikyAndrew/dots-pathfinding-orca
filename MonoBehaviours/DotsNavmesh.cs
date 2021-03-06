using System;
using System.Collections.Generic;
using System.Linq;
using ECS.Components;
using ECS.Systems;
using Pathfinding;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Transforms;
using UnityEngine;

namespace ECS.MonoBehaviours
{
    /* Copies the data generated by Astar Pathfinding asset into a DOTS-friendly array. */
    public class DotsNavmesh : MonoBehaviour
    {
        public static readonly int2[] NeighbourOffsets = new[]
        {
            new int2(-1, 0), // Left
            new int2(+1, 0), // Right
            new int2(0, +1), // Up
            new int2(0, -1), // Down
            new int2(-1, -1), // Left and Down
            new int2(-1, +1), // Left and Up
            new int2(+1, -1), // Right and Down
            new int2(+1, +1) // Right and Up 
        };

        [HideInInspector]
        public static DotsNavmesh Singleton;
    
        private NativeArray<NavmeshNode> _nodeArray;

        public NativeArray<NavmeshNode> NodeArray => this._nodeArray;
        public int Width { get; private set; }
        public int Depth { get; private set; }
        public int2 Size => new int2(Width, Depth);
        public float3 Center { get; private set; }
        public float NodeSize { get; private set; }

        private void Awake()
        {
            if (Singleton != null)
            {
                Debug.LogError("There is more than one DotsNavmesh in the scene. Remove one of them.");
            }
            else
            {
                Singleton = this;
            }
        }

        void OnEnable()
        {
            NavGraph[] allGraphs = AstarPath.active.data.graphs;
            GridGraph oldGraph = (GridGraph)allGraphs[0];
            List<GraphNode> nodes = new List<GraphNode>();
            oldGraph.GetNodes(nodes.Add);
            this.Width = oldGraph.width;
            this.Depth = oldGraph.depth;
            this.Center = oldGraph.center;
            this.NodeSize = oldGraph.nodeSize;
            
            CreateMapArray(nodes);
        
            World.DefaultGameObjectInjectionWorld.GetExistingSystem<PathfindingSystem>().Enabled = true;
        }
    
        private void CreateMapArray(List<GraphNode> astarNodes)
        {
            var em = World.DefaultGameObjectInjectionWorld.EntityManager;
            this._nodeArray = new NativeArray<NavmeshNode>(astarNodes.Count, Allocator.Persistent);
       
            for (int i = 0; i < astarNodes.Count; i++)
            {
                this._nodeArray[i] = new NavmeshNode
                {
                    isWalkable = astarNodes[i].Walkable,
                    x = i % this.Width,
                    y = i / this.Width,
                    position = new float3((Vector3)astarNodes[i].position)
                };
            }
        }

        private void OnDrawGizmos()
        {
            var walkableColor = new Color(0, 1, 0, 0.2f);
            foreach (var navmeshNode in this._nodeArray.Where(n=>n.isWalkable))
                DrawGizmoTile(walkableColor, navmeshNode.position);
            
        }

        private void DrawGizmoTile(Color color, Vector3 position)
        {
                Gizmos.color = color;
                Gizmos.DrawWireCube((Vector3)position+Vector3.up/2, new Vector3(NodeSize, 0.04f, NodeSize));
        }

        public bool IsPositionInsideGrid(int2 gridPosition)
        {
            return
                gridPosition.x >= 0 && 
                gridPosition.y >= 0 &&
                gridPosition.x < this.Width &&
                gridPosition.y < this.Depth;
        }
        
        public static bool IsPositionInsideGrid(int width, int depth, int2 gridPosition)
        {
            return
                gridPosition.x >= 0 && 
                gridPosition.y >= 0 &&
                gridPosition.x < width &&
                gridPosition.y < depth;
        }
        
        public int2 WorldPositionToNode(Vector3 position)
        {
            return new int2((int)((position.x-Center.x)/NodeSize + Width/2f), (int)((position.z-Center.z)/ NodeSize + Depth/2f));
        }
        
        public static int2 WorldPositionToNode(float3 center, float nodeSize, int width, int depth, float3 position)
        {
            return new int2((int)((position.x-center.x)/nodeSize + width/2f), (int)((position.z-center.z)/ nodeSize + depth/2f));
        }
        
        public static int CalculateIndex(int2 position, int gridWidth) {
            return position.x + position.y * gridWidth;
        }
    }
}
