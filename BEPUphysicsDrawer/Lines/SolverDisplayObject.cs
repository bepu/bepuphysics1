using BEPUphysics.SolverSystems;

namespace BEPUphysicsDrawer.Lines
{
    /// <summary>
    /// Line-based graphical representation of an object.
    /// </summary>
    /// <typeparam name="T">Type of the object to be displayed.</typeparam>
    public abstract class SolverDisplayObject<T> : LineDisplayObject<T> where T:SolverUpdateable
    {
        protected SolverDisplayObject(LineDrawer drawer, T lineObject)
            : base(drawer, lineObject)
        {
        }

        public override bool IsActive
        {
            get { return LineObject.IsActive; }
        }

    }
}