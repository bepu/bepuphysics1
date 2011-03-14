using BEPUphysics.CollisionTests;
using System.Collections.ObjectModel;
using BEPUphysics.ResourceManagement;
using BEPUphysics.DataStructures;

namespace BEPUphysics.Constraints.Collision
{
    ///<summary>
    /// Collision constraint for non-convex manifolds.  These manifolds are usually used in cases
    /// where the contacts are coming from multiple objects or from non-convex objects.  The normals
    /// will likely face more than one direction.
    ///</summary>
    public class NonConvexContactManifoldConstraint : ContactManifoldConstraint
    {
        //This contact manifold constraint covers any number of contact points.
        //Unlike the convex manifold constraint, this constraint enforces no requirements
        //on the contact data.  The collisions can form a nonconvex patch.  They can have differing normals.
        //This is required for proper collision handling on large structures

        //The solver group is composed of multiple constraints.
        //One pentration constraint for each contact.
        //One friction constraint for each contact.

        internal RawList<ContactPenetrationConstraint> penetrationConstraints;
        ///<summary>
        /// Gets the penetration constraints in the manifold.
        ///</summary>
        public ReadOnlyCollection<ContactPenetrationConstraint> ContactPenetrationConstraints { get; private set; }

        ResourcePool<ContactPenetrationConstraint> penetrationConstraintPool = new UnsafeResourcePool<ContactPenetrationConstraint>(4);

        internal RawList<ContactFrictionConstraint> frictionConstraints;
        ///<summary>
        /// Gets the friction constraints in the manifold.
        ///</summary>
        public ReadOnlyCollection<ContactFrictionConstraint> ContactFrictionConstraints { get; private set; }

        ResourcePool<ContactFrictionConstraint> frictionConstraintPool = new UnsafeResourcePool<ContactFrictionConstraint>(4);


        ///<summary>
        /// Constructs a new nonconvex manifold constraint.
        ///</summary>
        public NonConvexContactManifoldConstraint()
        {
            //All of the constraints are always in the solver group.  Some of them are just deactivated sometimes.
            //This reduces some bookkeeping complications.


            penetrationConstraints = new RawList<ContactPenetrationConstraint>(4);
            frictionConstraints = new RawList<ContactFrictionConstraint>(4);
            ContactPenetrationConstraints = new ReadOnlyCollection<ContactPenetrationConstraint>(penetrationConstraints);
            ContactFrictionConstraints = new ReadOnlyCollection<ContactFrictionConstraint>(frictionConstraints);

            for (int i = 0; i < 4; i++)
            {
                var penetrationConstraint = new ContactPenetrationConstraint();
                penetrationConstraintPool.GiveBack(penetrationConstraint);
                Add(penetrationConstraint);

                var frictionConstraint = new ContactFrictionConstraint();
                frictionConstraintPool.GiveBack(frictionConstraint);
                Add(frictionConstraint);
            }
            
        }


        ///<summary>
        /// Cleans up the constraint.
        ///</summary>
        public override void CleanUp()
        {
            //Deactivate any remaining constraints.
            for (int i = penetrationConstraints.count - 1; i >= 0; i--)
            {
                var penetrationConstraint = penetrationConstraints.Elements[i];
                penetrationConstraint.CleanUp();
                penetrationConstraints.RemoveAt(i);
                penetrationConstraintPool.GiveBack(penetrationConstraint);
            }

            for (int i = frictionConstraints.count - 1; i >= 0; i--)
            {
                var frictionConstraint = frictionConstraints.Elements[i];
                frictionConstraint.CleanUp();
                frictionConstraints.RemoveAt(i);
                frictionConstraintPool.GiveBack(frictionConstraint);
            }


        }


 




        //TODO: PROBLEM IS that the add contact/remove contact, when they go from 0 -> !0 or !0 -> 0, the whole constraint is added/removed from the solver.
        //The Added/Removed contact methods here will run ambiguously before or after they are removed from the solver.
        //That ambiguous order doesn't really matter though, since everything that these add/remove methods do is local to this solver object and its children.
        //It doesn't go out and modify any external values on referenced entities.  That only happens when it's added or removed from the solver by whatever owns this object!

        //To avoid ANY ambiguity, some third party is now responsible for adding and removing contacts from this.

        ///<summary>
        /// Adds a contact to be managed by the constraint.
        ///</summary>
        ///<param name="contact">Contact to add.</param>
        public override void AddContact(Contact contact)
        {
            var penetrationConstraint = penetrationConstraintPool.Take();
            penetrationConstraint.Setup(this, contact);
            penetrationConstraints.Add(penetrationConstraint);

            var frictionConstraint = frictionConstraintPool.Take();
            frictionConstraint.Setup(this, penetrationConstraint);
            frictionConstraints.Add(frictionConstraint);

        }

        ///<summary>
        /// Removes a contact from the constraint.
        ///</summary>
        ///<param name="contact">Contact to remove.</param>
        public override void RemoveContact(Contact contact)
        {

            ContactPenetrationConstraint penetrationConstraint = null;
            for (int i = 0; i < penetrationConstraints.count; i++)
            {
                if ((penetrationConstraint = penetrationConstraints.Elements[i]).contact == contact)
                {
                    penetrationConstraint.CleanUp();
                    penetrationConstraints.RemoveAt(i);
                    penetrationConstraintPool.GiveBack(penetrationConstraint);
                    break;
                }
            }
            for (int i = frictionConstraints.count - 1; i >= 0; i--)
            {
                ContactFrictionConstraint frictionConstraint = frictionConstraints[i];
                if (frictionConstraint.PenetrationConstraint == penetrationConstraint)
                {
                    frictionConstraint.CleanUp();
                    frictionConstraints.RemoveAt(i); 
                    frictionConstraintPool.GiveBack(frictionConstraint);
                    break;
                }
            }

        }





    }
}
