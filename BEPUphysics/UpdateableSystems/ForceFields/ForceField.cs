using System;
using System.Collections.Generic;
using BEPUphysics.BroadPhaseSystems;
using BEPUphysics.Entities;
using Microsoft.Xna.Framework;
using BEPUphysics.Threading;
using BEPUphysics.DataStructures;

namespace BEPUphysics.UpdateableSystems.ForceFields
{
    /// <summary>
    /// Superclass of objects which apply forces to entities in some field.
    /// </summary>
    public abstract class ForceField : Updateable, IDuringForcesUpdateable
    {
        private readonly Action<int> subfunction;
        private IList<Entity> affectedEntities;
        private float currentTimestep;
        private ForceFieldShape shape;


        ///<summary>
        /// Gets or sets whether or not threading is allowed.
        ///</summary>
        public bool AllowMultithreading { get; set; }

        ///<summary>
        /// Gets or sets the query accelerator used by the force field to find entities.
        ///</summary>
        public IQueryAccelerator QueryAccelerator { get; set; }
        ///<summary>
        /// Gets or sets the thread manager used by the force field.
        ///</summary>
        public IThreadManager ThreadManager { get; set; }

        /// <summary>
        /// Constructs a force field.
        /// </summary>
        /// <param name="shape">Shape to use for the force field.</param>
        /// <param name="queryAccelerator">Query accelerator used to find entities.</param>
        protected ForceField(ForceFieldShape shape, IQueryAccelerator queryAccelerator)
        {
            Shape = shape;
            QueryAccelerator = queryAccelerator;
            subfunction = new Action<int>(CalculateImpulsesSubfunction);
        }

        protected ForceField(ForceFieldShape shape, IQueryAccelerator queryAccelerator, IThreadManager threadManager)
            : this(shape, queryAccelerator)
        {
            ThreadManager = threadManager;
            AllowMultithreading = true;
        }

        /// <summary>
        /// Gets or sets whether the the force field will force affected entities to wake up.
        /// </summary>
        public bool ForceWakeUp { get; set; }

        /// <summary>
        /// Gets or sets the shape of the force field used to determine which entities to apply forces to.
        /// </summary>
        public ForceFieldShape Shape
        {
            get { return shape; }
            set
            {
                if (value != null && value.ForceField != null)
                    throw new ArgumentException("The force field shape already belongs to another force field.");
                //Get rid of my old shape.
                if (shape != null)
                {
                    shape.ForceField = null;
                }
                //Get my new shape!
                shape = value;
                if (shape != null)
                {
                    shape.ForceField = this;
                }
            }
        }

        /// <summary>
        /// Performs any custom logic desired prior to the force application.
        /// </summary>
        protected virtual void PreUpdate()
        {

        }

        /// <summary>
        /// Applies forces specified by the given calculation delegate to bodies in the volume.
        /// Called automatically when needed by the owning Space.
        /// </summary>
        /// <param name="dt">Time since the last frame in simulation seconds.</param>
        void IDuringForcesUpdateable.Update(float dt)
        {
            PreUpdate();
            affectedEntities = Shape.GetPossiblyAffectedEntities();
            if (AllowMultithreading && ThreadManager.ThreadCount > 1)
            {
                currentTimestep = dt;
                ThreadManager.ForLoop(0, affectedEntities.Count, subfunction);
            }
            else
            {
                currentTimestep = dt;
                //No multithreading, so do it directly.
                int count = affectedEntities.Count;
                for (int i = 0; i < count; i++)
                {
                    CalculateImpulsesSubfunction(i);
                }
            }
        }

        /// <summary>
        /// Calculates the impulse to apply to the entity.
        /// </summary>
        /// <param name="e">Affected entity.</param>
        /// <param name="dt">Duration between simulation updates.</param>
        /// <param name="impulse">Impulse to apply to the entity.</param>
        protected abstract void CalculateImpulse(Entity e, float dt, out Vector3 impulse);

        private void CalculateImpulsesSubfunction(int index)
        {
            Entity e = affectedEntities[index];
            if (e.isDynamic && (e.activityInformation.IsActive || ForceWakeUp) && Shape.IsEntityAffected(e))
            {
                if (ForceWakeUp)
                    e.activityInformation.Activate();
                Vector3 impulse;
                CalculateImpulse(e, currentTimestep, out impulse);
                e.ApplyLinearImpulse(ref impulse);
            }
        }

    }
}