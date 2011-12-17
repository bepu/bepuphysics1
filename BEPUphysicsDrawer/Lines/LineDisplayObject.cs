namespace BEPUphysicsDrawer.Lines
{
    /// <summary>
    /// Line-based graphical representation of an object.
    /// </summary>
    /// <typeparam name="T">Type of the object to be displayed.</typeparam>
    public abstract class LineDisplayObject<T> : LineDisplayObjectBase
    {
        protected LineDisplayObject(LineDrawer drawer, T lineObject)
            : base(drawer)
        {
            this.LineObject = lineObject;
        }

        /// <summary>
        /// Gets the object to represent with lines.
        /// </summary>
        public T LineObject { get; private set; }

    }
}