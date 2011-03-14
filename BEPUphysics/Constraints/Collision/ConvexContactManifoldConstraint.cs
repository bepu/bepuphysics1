using BEPUphysics.CollisionTests;
using System.Collections.ObjectModel;
using BEPUphysics.ResourceManagement;
using BEPUphysics.DataStructures;

namespace BEPUphysics.Constraints.Collision
{
    ///<summary>
    /// Contact manifold constraint that is used by manifolds whose normals are assumed to be
    /// essentially the same.  This assumption can only be maintained between two convex objects.
    ///</summary>
    public class ConvexContactManifoldConstraint : ContactManifoldConstraint
    {
        //This contact manifold constraint covers a single, 4-contact pair.

        //The solver group is composed of multiple constraints.
        //One pentration constraint for each contact.
        //One sliding constraint.
        //One twist constraint.

        internal TwistFrictionConstraint twistFriction;
        ///<summary>
        /// Gets the twist friction constraint used by the manifold.
        ///</summary>
        public TwistFrictionConstraint TwistFriction
        {
            get
            {
                return twistFriction;
            }
        }
        internal SlidingFrictionTwoAxis slidingFriction;
        ///<summary>
        /// Gets the sliding friction constraint used by the manifold.
        ///</summary>
        public SlidingFrictionTwoAxis SlidingFriction
        {
            get
            {
                return slidingFriction;
            }
        }



        internal RawList<ContactPenetrationConstraint> penetrationConstraints;
        ///<summary>
        /// Gets the penetration constraints used by the manifold.
        ///</summary>
        public ReadOnlyCollection<ContactPenetrationConstraint> ContactPenetrationConstraints { get; private set; }

        ResourcePool<ContactPenetrationConstraint> penetrationConstraintPool = new UnsafeResourcePool<ContactPenetrationConstraint>(4);


        ///<summary>
        /// Constructs a new convex contact manifold constraint.
        ///</summary>
        public ConvexContactManifoldConstraint()
        {
            //All of the constraints are always in the solver group.  Some of them are just deactivated sometimes.
            //This reduces some bookkeeping complications.


            penetrationConstraints = new RawList<ContactPenetrationConstraint>(4);
            ContactPenetrationConstraints = new ReadOnlyCollection<ContactPenetrationConstraint>(penetrationConstraints);
  

            //Order matters in this adding process.  Sliding friction computes some information used by the twist friction, and both use penetration impulses.
            for (int i = 0; i < 4; i++)
            {
                var penetrationConstraint = new ContactPenetrationConstraint();
                Add(penetrationConstraint);
                penetrationConstraintPool.GiveBack(penetrationConstraint);
            }
            slidingFriction = new SlidingFrictionTwoAxis();
            Add(slidingFriction); 
            twistFriction = new TwistFrictionConstraint();
            Add(twistFriction);
            
            
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
            if (twistFriction.isActive)
            {
                twistFriction.CleanUp();
                slidingFriction.CleanUp();
            }


        }


        ///<summary>
        /// Adds a contact to be managed by the constraint.
        ///</summary>
        ///<param name="contact">Contact to add.</param>
        public override void AddContact(Contact contact)
        {
            var penetrationConstraint = penetrationConstraintPool.Take();
            penetrationConstraint.Setup(this, contact);
            penetrationConstraints.Add(penetrationConstraint);
            if (penetrationConstraints.count == 1)
            {
                //This is the first contact.  All constraints need to become active.
                twistFriction.Setup(this);
                slidingFriction.Setup(this);
            }
        }

        ///<summary>
        /// Removes a contact from the constraint.
        ///</summary>
        ///<param name="contact">Contact to remove.</param>
        public override void RemoveContact(Contact contact)
        {
            for (int i = 0; i < penetrationConstraints.count; i++)
            {
                ContactPenetrationConstraint penetrationConstraint;
                if ((penetrationConstraint = penetrationConstraints.Elements[i]).contact == contact)
                {
                    penetrationConstraint.CleanUp();
                    penetrationConstraints.RemoveAt(i);
                    penetrationConstraintPool.GiveBack(penetrationConstraint);
                    break;
                }
            }
            if (penetrationConstraints.count == 0)
            {
                //No more contacts.  Disable everything.
                twistFriction.CleanUp();
                slidingFriction.CleanUp();
            }
        }





    }
}
