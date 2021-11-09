namespace ECS.Structs
{
    /* Represents A* algorithm data for a particular node-request pair. */
    public struct PathfindingResult
    {
        
        public float gCost;
        public float hCost;        
        public int cameFromNodeIndex;

        public float fCost => this.gCost + this.hCost;
    }
}