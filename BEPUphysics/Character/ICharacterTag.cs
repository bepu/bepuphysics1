using BEPUphysics.Entities;

namespace BEPUphysics.Character
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
