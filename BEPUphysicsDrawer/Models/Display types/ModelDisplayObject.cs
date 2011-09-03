namespace BEPUphysicsDrawer.Models
{
    /// <summary>
    /// Model-based graphical representation of an object.
    /// </summary>
    /// <typeparam name="T">Type of the object to be displayed.</typeparam>
    public abstract class ModelDisplayObject<T> : ModelDisplayObjectBase
    {
        protected ModelDisplayObject(ModelDrawer drawer, T displayedObject)
            : base(drawer)
        {
            DisplayedObject = displayedObject;
        }

        /// <summary>
        /// Gets the object to represent with a model.
        /// </summary>
        public T DisplayedObject { get; private set; }
    }
}