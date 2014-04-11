using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading;
using BEPUphysics.Entities;
using BEPUutilities.DataStructures;

namespace BEPUphysics.Character
{
    /// <summary>
    /// Provides safety for interacting with collision pairs which involve other characters.
    /// </summary>
    public class CharacterPairLocker
    {
        private Entity characterBody;

        /// <summary>
        /// Constructs a new character pair locker.
        /// </summary>
        /// <param name="characterBody">Body associated with the character using this locker.</param>
        public CharacterPairLocker(Entity characterBody)
        {
            this.characterBody = characterBody;
        }

        RawList<ICharacterTag> involvedCharacters = new RawList<ICharacterTag>();
        /// <summary>
        /// Locks all pairs which involve other characters.
        /// </summary>
        public void LockCharacterPairs()
        {
            //If this character is colliding with another character, there's a significant danger of the characters
            //changing the same collision pair handlers.  Rather than infect every character system with micro-locks,
            //we lock the entirety of a character update.

            foreach (var pair in characterBody.CollisionInformation.Pairs)
            {
                //Is this a pair with another character?
                var other = pair.BroadPhaseOverlap.EntryA == characterBody.CollisionInformation ? pair.BroadPhaseOverlap.EntryB : pair.BroadPhaseOverlap.EntryA;
                var otherCharacter = other.Tag as ICharacterTag;
                if (otherCharacter != null)
                {
                    involvedCharacters.Add(otherCharacter);
                }
            }
            if (involvedCharacters.Count > 0)
            {
                //If there were any other characters, we also need to lock ourselves!
                involvedCharacters.Add((ICharacterTag)characterBody.CollisionInformation.Tag);

                //However, the characters cannot be locked willy-nilly.  There needs to be some defined order in which pairs are locked to avoid deadlocking.
                involvedCharacters.Sort(comparer);

                for (int i = 0; i < involvedCharacters.Count; ++i)
                {
                    Monitor.Enter(involvedCharacters[i]);
                }
            }
        }

        private static Comparer comparer = new Comparer();
        class Comparer : IComparer<ICharacterTag>
        {
            public int Compare(ICharacterTag x, ICharacterTag y)
            {
                if (x.InstanceId < y.InstanceId)
                    return -1;
                if (x.InstanceId > y.InstanceId)
                    return 1;
                return 0;
            }
        }

        /// <summary>
        /// Unlocks all pairs which involve other characters.
        /// </summary>
        public void UnlockCharacterPairs()
        {
            //Unlock the pairs, LIFO.
            for (int i = involvedCharacters.Count - 1; i >= 0; i--)
            {
                Monitor.Exit(involvedCharacters[i]);
            }
            involvedCharacters.Clear();
        }

    }
}
