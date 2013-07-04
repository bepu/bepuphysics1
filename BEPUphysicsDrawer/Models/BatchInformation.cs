namespace BEPUphysicsDrawer.Models
{
    /// <summary>
    /// Contains some information about a ModelDisplayObject's batch.
    /// </summary>
    public class BatchInformation
    {
        /// <summary>
        /// Index in the batch's index buffer where this display object's vertices start.
        /// </summary>
        public int BaseIndexBufferIndex;

        /// <summary>
        /// Index in the batch's vertex buffer where this display object's vertices start.
        /// </summary>
        public int BaseVertexBufferIndex;

        /// <summary>
        /// Batch that a display object belongs to.
        /// </summary>
        public ModelDisplayObjectBatch Batch;

        /// <summary>
        /// Index in the batch's display object list where this display object will be inserted.
        /// </summary>
        public int BatchListIndex;

        /// <summary>
        /// Number of indices in the display object.
        /// </summary>
        public int IndexCount;

        /// <summary>
        /// Number of vertices in the display object.
        /// </summary>
        public int VertexCount;
    }
}