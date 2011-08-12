using BEPUphysics.Constraints.SolverGroups;
using BEPUphysics.Entities;

namespace BEPUphysics.Constraints.Collision
{
    ///<summary>
    /// Constraint group containing multiple contact manifold constraints.
    /// Used by some pairs which manage multiple sub-pairs.
    ///</summary>
    public class ContactManifoldConstraintGroup : SolverGroup
    {

        protected Entity entityA;
        ///<summary>
        /// Gets the first entity in the pair.
        ///</summary>
        public Entity EntityA { get { return entityA; } }
        protected Entity entityB;
        ///<summary>
        /// Gets the second entity in the pair.
        ///</summary>
        public Entity EntityB { get { return entityB; } }

        ///<summary>
        /// Adds a constraint to the group.
        ///</summary>
        ///<param name="manifoldConstraint">Constraint to add.</param>
        public new void Add(EntitySolverUpdateable manifoldConstraint)
        {
            base.Add(manifoldConstraint);
        }

        ///<summary>
        /// Removes a constraint from the group.
        ///</summary>
        ///<param name="manifoldConstraint">Constraint to remove.</param>
        public new void Remove(EntitySolverUpdateable manifoldConstraint)
        {
            base.Remove(manifoldConstraint);
        }

        protected internal override void CollectInvolvedEntities(DataStructures.RawList<Entity> outputInvolvedEntities)
        {
            //The default implementation for solver groups looks at every single subconstraint.
            //That's not necessary for these special constraints.
            if (entityA != null)
                outputInvolvedEntities.Add(entityA);
            if (entityB != null)
                outputInvolvedEntities.Add(entityB);
        }



        protected internal override void OnInvolvedEntitiesChanged()
        {
            //The default implementation of this method is pretty complicated.
            //This is a special constraint that has certain guarantees that allow a simpler method to be used.
            //This updates the involved entities and connected members lists, but does not update references.
            //It does not have to update references because this 'constraint' can never change entities while belonging to a solver.
            //It doesn't even need to notify the parent solvergroup.
            CollectInvolvedEntities();
        }

        ///<summary>
        /// Initializes the constraint group.
        ///</summary>
        ///<param name="a">First entity of the pair.</param>
        ///<param name="b">Second entity of the pair.</param>
        public virtual void Initialize(Entity a, Entity b)
        {
            //This should only be called before the constraint has been added to the solver.
            entityA = a;
            entityB = b;
            OnInvolvedEntitiesChanged();
        }

        ///<summary>
        /// Cleans up the constraint group.
        ///</summary>
        public virtual void CleanUp()
        {
            //This should only be called after the constraint has been removed from the solver.
            entityA = null;
            entityB = null;
            OnInvolvedEntitiesChanged();
        }

    }
}
