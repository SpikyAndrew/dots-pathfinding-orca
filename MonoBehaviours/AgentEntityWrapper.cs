using System;
using System.IO;
using ECS.Components;
using ECS.Structs;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Transforms;
using UnityEngine;

namespace ECS.MonoBehaviours
{
    [RequireComponent(typeof(CapsuleCollider))]
    /* Add this to a game object to utilize the DOTS pathfinding and collision avoidance. 
     * Sends pathfinding requests to the PathfindingSystem and synchronizes the object's Transform to the position of an agent Entity. */
    public class AgentEntityWrapper : MonoBehaviour
    {
        private Entity entity;

        public Entity Entity => this.entity;

        [SerializeField]
        private int EntityIndex;
        
        [SerializeField]
        private bool drawDebugGeometry;
        
        public float3 Velocity { get; private set; }
       
        public float movementSpeed;

        public bool TargetReached
        {
            get
            {
                if (this.target == null)
                    return true;
                if(this.em.HasComponent<PathInstructions>(this.entity))
                    return (transform.position - this.targetNodePosition).sqrMagnitude < DesiredVelocitySystem.EndReachedDistanceSquared;
                return false;
            }
        }

        private bool canMove;
        private EntityManager em;
        private Transform target;
        private Vector3 targetNodePosition;
        
        private void Awake()
        {
            this.target = transform;
        }

        private void Start()
        {
            em = World.DefaultGameObjectInjectionWorld.EntityManager;
            this.entity = em.CreateEntity();

            if (this.drawDebugGeometry)
            {
                /* Debug drawing enabled. */
                this.em.AddComponentData(this.entity, new DebugDrawTag());
            }
            
            this.em.AddComponentData(this.entity, new AgentMovement(this.movementSpeed));
            this.em.AddComponentData(this.entity, new Translation{Value = transform.position});
            this.em.AddComponentData(this.entity, new VelocityObstacle(){radius = GetComponent<CapsuleCollider>().radius * transform.localScale.x});
            this.em.AddComponentData(this.entity, new SectorTag());
            this.em.AddComponentData(this.entity, new DesiredVelocity());
            this.em.AddComponentData(this.entity, new CollisionFreeVelocity());
            this.em.AddBuffer<PathNode>(this.entity);
            this.em.AddBuffer<LinearConstraint>(this.entity);
            this.EntityIndex = this.entity.Index;
        }
        
        void Update()
        {
            if (this.canMove)
            {
                /* Synchronize the position of the game object to match the entity. */
                var targetPosition = em.GetComponentData<Translation>(entity).Value;
                
                Vector3 direction = (Vector3)targetPosition - transform.position;
                direction.y = 0;

                if (direction == Vector3.zero)
                    return;
                
                Quaternion toRotation = Quaternion.LookRotation(direction);
                
                /* Lerp to make movement appear smoother. */

                transform.rotation = Quaternion.Lerp(transform.rotation, toRotation, Time.deltaTime * 1.5f);

                float3 oldPosition = new float3(transform.position);
                
                transform.position = Vector3.Lerp(transform.position, targetPosition, Time.deltaTime * 2);
                
                Velocity = (oldPosition - (float3) transform.position) / Time.deltaTime;
            }
        }
        
        /* Setting this creates a new request for the PathfindingSystem */
        public Transform Target
        {
            get
            {
                return target;
            }
            set
            {
                this.target = value;
                this.em.AddComponentData(this.entity, GeneratePathfindingRequest());
            }
        }

        /* Setting this to false will prevent the MovementSystem from updating the agent's position. */
        public bool CanMove
        {
            get => this.canMove;
            set
            {
                this.canMove = value;
                if (value)
                    this.em.AddComponentData(this.entity, new MovementTag());
                else
                {
                    this.em.RemoveComponent<MovementTag>(this.entity);
                }
            }
        }

        private PathfindingRequest GeneratePathfindingRequest()
        {
            var navmesh = DotsNavmesh.Singleton;
            int2 startNode = FindWalkableNodeNearby(navmesh.WorldPositionToNode(transform.position));
            int2 endNode = FindWalkableNodeNearby(navmesh.WorldPositionToNode(this.target.position));
            this.targetNodePosition = navmesh.NodeArray[DotsNavmesh.CalculateIndex(endNode, navmesh.Width)].position;
            return new PathfindingRequest
            {
                startPosition = startNode,
                endPosition = endNode
            };
        }

        private int2 FindWalkableNodeNearby(int2 node)
        {
            var navmesh = DotsNavmesh.Singleton;
            var nodeIndex = DotsNavmesh.CalculateIndex(node, navmesh.Width);
            if (navmesh.IsPositionInsideGrid(node) && navmesh.NodeArray[nodeIndex].isWalkable)
            {
                return node;
            }
            else
            {
                int i = 1;
                while (true)
                {
                    foreach (var offset in DotsNavmesh.NeighbourOffsets)
                    {
                        var nearbyNode = node + offset*i;
                        if (navmesh.IsPositionInsideGrid(nearbyNode)
                            && navmesh.NodeArray[DotsNavmesh.CalculateIndex(nearbyNode, navmesh.Width)].isWalkable)
                        {
                            return nearbyNode;
                        }
                    }
                    i++;
                }
            }
        }

        public bool IsFollowingAPath()
        {
            return em.GetComponentData<PathInstructions>(this.entity).nodeIndex != -1;
        }

        private void OnDestroy()
        {
            if (World.DefaultGameObjectInjectionWorld != null)
            {
                if(this.em.Exists(this.entity))
                    this.em.DestroyEntity(this.entity);
            }
        }

        public void SetPosition(Vector3 position)
        {
            transform.position = position;
            this.em.SetComponentData(this.entity, new Translation{Value = position});
        }
    }
}
