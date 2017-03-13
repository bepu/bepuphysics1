using System;
using BEPUphysics.BroadPhaseEntries;
using BEPUutilities.DataStructures;
using BEPUutilities.ResourceManagement;

namespace BEPUphysics.CollisionTests.Manifolds
{
    ///<summary>
    /// Superclass of manifolds which manage persistent contacts over multiple frames.
    ///</summary>
    public abstract class ContactManifold
    {
        /// <summary>
        /// Gets or sets whether this pair handler should avoid doing any prefiltering on contacts that might destroy information for the user.
        /// Some pairs, like the convex-mesh types, will operate a little differently when used as a query. This is a hack that should be addressed later.
        /// </summary>
        /// <remarks>
        /// Very few manifolds actually make use of this. It exists solely as a hack to compensate for other design limitations surrounding collision detection APIs
        /// and specifically one sided mesh contact generation breaking character controller stance queries.
        /// </remarks>
        public bool IsQuery { get; set; }

        protected RawList<int> contactIndicesToRemove;
        protected internal RawList<Contact> contacts;


        ///<summary>
        /// Gets the contacts in the manifold.
        ///</summary>
        public ReadOnlyList<Contact> Contacts
        {
            get
            {
                return new ReadOnlyList<Contact>(contacts);
            }
        }

        protected UnsafeResourcePool<Contact> unusedContacts;


        protected void RemoveQueuedContacts()
        {
            //TOREMOVE MUST BE SORTED LEAST TO GREATEST INDEX.
            for (int i = contactIndicesToRemove.Count - 1; i >= 0; i--)
            {
                Remove(contactIndicesToRemove.Elements[i]);
            }
            contactIndicesToRemove.Clear();
        }

        protected virtual void Remove(int contactIndex)
        {
            Contact removing = contacts.Elements[contactIndex];
            contacts.FastRemoveAt(contactIndex);
            OnRemoved(removing);
            unusedContacts.GiveBack(removing);
        }

        protected virtual void Add(ref ContactData contactCandidate)
        {
            Contact adding = unusedContacts.Take();
            adding.Setup(ref contactCandidate);
            contacts.Add(adding);
            OnAdded(adding);
        }


        ///<summary>
        /// Fires when a contact is added.
        ///</summary>
        public event Action<Contact> ContactAdded;
        ///<summary>
        /// Fires when a contact is removed.
        ///</summary>
        public event Action<Contact> ContactRemoved;

        protected void OnAdded(Contact contact)
        {
            if (ContactAdded != null)
                ContactAdded(contact);
        }

        protected void OnRemoved(Contact contact)
        {
            if (ContactRemoved != null)
                ContactRemoved(contact);
        }

        ///<summary>
        /// Initializes the manifold.
        ///</summary>
        ///<param name="newCollidableA">First collidable.</param>
        ///<param name="newCollidableB">Second collidable.</param>
        public abstract void Initialize(Collidable newCollidableA, Collidable newCollidableB);


        ///<summary>
        /// Cleans up the manifold.
        ///</summary>
        public virtual void CleanUp()
        {
            for (int i = contacts.Count - 1; i >= 0; --i)
            {
                unusedContacts.GiveBack(contacts.Elements[i]);
                contacts.FastRemoveAt(i);
            }
        }

        ///<summary>
        /// Updates the manifold.
        ///</summary>
        ///<param name="dt">Timestep duration.</param>
        public abstract void Update(float dt);

        /// <summary>
        /// Clears the contacts associated with this manifold.
        /// </summary>
        public virtual void ClearContacts()
        {
            for (int i = contacts.Count - 1; i >= 0; i--)
            {
                Remove(i);
            }
        }

    }

}
