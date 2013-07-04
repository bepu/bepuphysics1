using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using BEPUphysics.Entities;
using BEPUutilities;

namespace BEPUphysicsDemos.AlternateMovement
{
    /// <summary>
    /// Defines a class which uniquely identifies a character.
    /// Exposes an identifier for use in ordering character locks to ensure multithreaded safety.
    /// </summary>
    public interface ICharacterTag
    {
        /// <summary>
        /// Gets the unique instance identifier for this character.
        /// </summary>
        long InstanceId { get; }
    }

    /// <summary>
    /// Links the body of the character to the character controller for locking.  Default implementation of the ICharacterTag.
    /// </summary>
    public class CharacterSynchronizer : ICharacterTag
    {
        /// <summary>
        /// This object is locked by the characters when performing any constraint changes.  There's no strong conceptual reason for this locker
        /// to be within the ICharacterTag implementation; it's just a convenient shared location for both the SphereCharacterController and CharacterController.
        /// </summary>
        public static SpinLock ConstraintAccessLocker = new SpinLock();

        Entity body;

        /// <summary>
        /// Constructs a new character tag.
        /// </summary>
        /// <param name="body">Body of the character.</param>
        public CharacterSynchronizer(Entity body)
        {
            this.body = body;
        }

        /// <summary>
        /// Gets the unique instance identifier for this character.
        /// </summary>
        public long InstanceId
        {
            get { return body.InstanceId; }
        }
    }
}
