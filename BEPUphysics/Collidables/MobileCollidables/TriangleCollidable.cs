using BEPUphysics.CollisionShapes.ConvexShapes;

namespace BEPUphysics.Collidables.MobileCollidables
{
    ///<summary>
    /// Special case collidable for reuseable triangles.
    ///</summary>
    public class TriangleCollidable : ConvexCollidable<TriangleShape>
    {
        ///<summary>
        /// Constructs a new shapeless collidable.
        ///</summary>
        public TriangleCollidable()
            : base(null)
        {
        }

        ///<summary>
        /// Constructs the triangle collidable using the given shape.
        ///</summary>
        ///<param name="shape">TriangleShape to use in the collidable.</param>
        public TriangleCollidable(TriangleShape shape)
            : base(shape)
        {
        }

        ///<summary>
        /// Initializes the collidable using the new triangle shape, but does NOT
        /// fire any shape-changed events.
        ///</summary>
        ///<param name="triangleShape">New triangle shape to use.</param>
        public void Initialize(TriangleShape triangleShape)
        {
            shape = triangleShape;
        }

        ///<summary>
        /// Cleans up the collidable by removing all events and setting the shape to null.
        ///</summary>
        public void CleanUp()
        {
            events.RemoveAllEvents();
            shape = null;
        }
    }
}
