using System.Collections.Generic;
using System.Linq;
using ECS.Components;
using ECS.MonoBehaviours;
using ECS.Structs;
using Unity.Burst;
using Unity.Collections;
using Unity.Core;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;

namespace ECS.Systems
{
    /* Implements the A* pathfinding algorithm for high-level pathfinding. */
    [AlwaysUpdateSystem]
    public class PathfindingSystem : ComponentSystem
    {
        private const int StraightMoveCost = 10;
        /* Approximation of  10*sqrt(2) */
        private const int DiagonalMoveCost = 14;
        
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

        private List<JobHandle> jobHandles = new List<JobHandle>();
        private List<AStarJob> jobs = new List<AStarJob>();
        
        
        /* Finds a path from startPosition to endPosition, represented as nodes on a square mesh */
        [BurstCompile]
        public struct AStarJob : IJob
        {
            public int2 gridSize;

            
            [ReadOnly] public NativeArray<NavmeshNode> nodes;
            [ReadOnly] public int2 startPosition;
            [ReadOnly] public int2 endPosition;
            [ReadOnly] public Entity entity;
            [ReadOnly] public NativeArray<int2> neighbourOffsetArray;
            
            public NativeList<int> openList;
            public NativeList<int> closedList;
            public NativeArray<PathfindingResult> pathfindingResults;

            public void Execute()
            {
                int endNodeIndex = DotsNavmesh.CalculateIndex(this.endPosition, this.gridSize.x);
                var startNodeIndex =
                    DotsNavmesh.CalculateIndex(this.startPosition, this.gridSize.x);
                PathfindingResult startNodePathfindingResult = this.pathfindingResults[startNodeIndex];
                startNodePathfindingResult.gCost = 0;
                startNodePathfindingResult.hCost = CalculateDiagonalHeuristicCost(this.startPosition, this.endPosition);
                this.pathfindingResults[startNodeIndex] = startNodePathfindingResult;

                openList.Add(startNodeIndex);

                var currentNodeIndex = startNodeIndex;

                while (currentNodeIndex != endNodeIndex)
                {
                    if (this.openList.Length == 0)
                    {
                        /* There is no path */
                        return;
                    }
                    currentNodeIndex = GetLowestCostFNodeIndex(openList, this.pathfindingResults);

                    openList.RemoveAtSwapBack(openList.IndexOf(currentNodeIndex));

                    closedList.Add(currentNodeIndex);

                    var currentNode = this.nodes[currentNodeIndex];
                    var currentNodePathfindingResult = this.pathfindingResults[currentNodeIndex];

                    for (int i = 0; i < neighbourOffsetArray.Length; i++)
                    {
                        int2 neighbourOffset = neighbourOffsetArray[i];
                        int2 neighbourPosition =
                            new int2(currentNode.x + neighbourOffset.x, currentNode.y + neighbourOffset.y);
                        if (!DotsNavmesh.IsPositionInsideGrid(this.gridSize.x, this.gridSize.y, neighbourPosition))
                        {
                            /* Out of bounds */
                            continue;
                        }

                        int neighbourNodeIndex = DotsNavmesh.CalculateIndex(neighbourPosition, this.gridSize.x);
                        var neighbourNode = this.nodes[neighbourNodeIndex];
                        var neighbourNodePathfindingResult = this.pathfindingResults[neighbourNodeIndex];

                        if (!neighbourNode.isWalkable)
                        {
                            /* Not walkable */
                            continue;
                        }

                        var cost = currentNodePathfindingResult.gCost + MovementCost(currentNode, neighbourNode);
                        bool neighbourInOpen = openList.Contains(neighbourNodeIndex);
                        bool neighbourInClosed = closedList.Contains(neighbourNodeIndex);
                        if (cost < neighbourNodePathfindingResult.gCost)
                        {
                            if (neighbourInOpen)
                            {
                                openList.RemoveAtSwapBack(openList.IndexOf(neighbourNodeIndex));
                                neighbourInOpen = false;
                            }

                            if (neighbourInClosed)
                            {
                                closedList.RemoveAtSwapBack(closedList.IndexOf(neighbourNodeIndex));
                                neighbourInClosed = false;
                            }
                        }

                        if (!neighbourInOpen && !neighbourInClosed)
                        {
                            neighbourNodePathfindingResult.gCost = cost;
                            neighbourNodePathfindingResult.hCost = CalculateDiagonalHeuristicCost(neighbourPosition, this.endPosition);
                            neighbourNodePathfindingResult.cameFromNodeIndex = currentNodeIndex;
                            this.pathfindingResults[neighbourNodeIndex] = neighbourNodePathfindingResult;
                            openList.Add(neighbourNodeIndex);
                        }
                    }
                }
            }
        }  
        
        protected override void OnStartRunning()
        {
            base.OnStartRunning();
            if (DotsNavmesh.Singleton == null)
            {
                Debug.Log("There is no Navmesh in the Scene! PathfindingSystem will shut down. ");
                Enabled = false;
                return;
            }
            
            var em = World.DefaultGameObjectInjectionWorld.EntityManager;
        }

        
        protected override void OnDestroy()
        {
            /* Complete all the jobs before destroying. */
            this.jobHandles.ForEach(j=>j.Complete());
            this.neighbourOffsets.Dispose();
            for (int i = 0; i < this.jobHandles.Count(); i++)
            {
                var jobHandle = this.jobHandles[i];
                var j = jobs[i];
                
                jobHandle.Complete();
                j.closedList.Dispose();
                j.openList.Dispose();
                j.pathfindingResults.Dispose();
            }
            DotsNavmesh.Singleton.NodeArray.Dispose();
        }

        protected override void OnUpdate()
        {
            var pathBuffers = GetBufferFromEntity<PathNode>();

            
            for (int i = 0; i < this.jobHandles.Count; i++)
            {
                var jobHandle = this.jobHandles[i];
                if (!jobHandle.IsCompleted) 
                    continue;
                jobHandle.Complete();
                this.jobHandles.RemoveAt(i);
                var j = this.jobs[i];
                    
                if (pathBuffers.Exists(j.entity))
                {
                    var pathBuffer = pathBuffers[j.entity];
                    PutPathInBuffer(pathBuffer, j.pathfindingResults, DotsNavmesh.CalculateIndex(j.endPosition, DotsNavmesh.Singleton.Width));
                    var currentPathInstructions = new PathInstructions
                    {
                        nodeIndex = pathBuffer.Length-1,
                    };
                    PostUpdateCommands.AddComponent(j.entity, currentPathInstructions);
                }

                j.closedList.Dispose();
                j.openList.Dispose();
                j.pathfindingResults.Dispose();
                    
                this.jobs.RemoveAt(i);
            }
            
            Entities.ForEach((Entity entity, ref PathfindingRequest pathfindingRequestComponent) =>
            {
                var pathBuffer = pathBuffers[entity];
                pathBuffer.Clear();

                AStarJob aStarJob = new AStarJob
                {
                    gridSize = DotsNavmesh.Singleton.Size,
                    nodes = DotsNavmesh.Singleton.NodeArray,
                    pathfindingResults = new NativeArray<PathfindingResult>(DotsNavmesh.Singleton.Width * DotsNavmesh.Singleton.Depth, Allocator.Persistent),
                    startPosition = pathfindingRequestComponent.startPosition,
                    endPosition = pathfindingRequestComponent.endPosition,
                    entity = entity,
                    neighbourOffsetArray = this.neighbourOffsets,
                    openList = new NativeList<int>(Allocator.Persistent),
                    closedList = new NativeList<int>(Allocator.Persistent)
                };
                jobs.Add(aStarJob);
                jobHandles.Add(aStarJob.Schedule());
                PostUpdateCommands.RemoveComponent<PathfindingRequest>(entity);
            });
        }

        private void PutPathInBuffer(DynamicBuffer<PathNode> pathBuffer,
            NativeArray<PathfindingResult> pathfindingResults, int endNodeIndex)
        {
            var endNode  = DotsNavmesh.Singleton.NodeArray[endNodeIndex];
            var endNodePathfindingData = pathfindingResults[endNodeIndex];
            if (endNodePathfindingData.cameFromNodeIndex == -1)
            {
                Debug.LogError("Couldn't find a path to target!");
            }
            else
            {
                /* Found a path. */
                pathBuffer.Add(new PathNode
                {
                    x = endNode.x,
                    y = endNode.y,
                    position = endNode.position
                });

                var currentNode = endNode;
                int2 direction = int2.zero;
                PathfindingResult nodeResult = endNodePathfindingData;
                while (nodeResult.cameFromNodeIndex != 0)
                {
                    var precedingNode = DotsNavmesh.Singleton.NodeArray[nodeResult.cameFromNodeIndex];
                    
                    int2 newDirection = new int2(currentNode.x - precedingNode.x, currentNode.y - precedingNode.y);

                    if (!newDirection.Equals(direction))
                    {
                        direction = newDirection;
                        pathBuffer.Add(new PathNode
                        {
                            x = precedingNode.x,
                            y = precedingNode.y,
                            position = precedingNode.position
                        });
                    }
                    else
                    {
                        direction = int2.zero;
                    }
                    currentNode = precedingNode;
                    nodeResult = pathfindingResults[nodeResult.cameFromNodeIndex];
                }
            }
        }
        
        private static int MovementCost(NavmeshNode fromNode, NavmeshNode toNode)
        {
            int xDistance = math.abs(fromNode.x - toNode.x);
            int yDistance = math.abs(fromNode.y - toNode.y);
            int remaining = math.abs(xDistance - yDistance);
            return DiagonalMoveCost * math.min(xDistance, yDistance) + StraightMoveCost * remaining;    
        }
        
        private static float CalculateDiagonalHeuristicCost(int2 nodePosition, int2 endPosition)
        {
            int xDistance = math.abs(nodePosition.x - endPosition.x);
            int yDistance = math.abs(nodePosition.y - endPosition.y);
            int remaining = math.abs(xDistance - yDistance);
            return (DiagonalMoveCost * math.min(xDistance, yDistance) + StraightMoveCost * remaining) * 1.0001f;
        }
            
        private static int GetLowestCostFNodeIndex(NativeList<int> list, NativeArray<PathfindingResult> pathfindingResults) {
            PathfindingResult lowestCostPathNode = pathfindingResults[list[0]];
            int index = list[0];
            for (int i = 1; i < list.Length; i++) {
                PathfindingResult testPathNode = pathfindingResults[list[i]];
                if (testPathNode.fCost < lowestCostPathNode.fCost) {
                    lowestCostPathNode = testPathNode;
                    index = list[i];
                }
            }
            return index;
        }
    }
}