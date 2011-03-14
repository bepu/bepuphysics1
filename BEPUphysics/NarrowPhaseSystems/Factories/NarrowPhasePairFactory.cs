using BEPUphysics.BroadPhaseSystems;
using BEPUphysics.NarrowPhaseSystems.Pairs;

namespace BEPUphysics.NarrowPhaseSystems.Factories
{
    ///<summary>
    /// Superclass of all factories which manufacture narrow phase pairs.
    ///</summary>
    public abstract class NarrowPhasePairFactory
    {

        ///<summary>
        /// Manufactures and returns a narrow phase pair for the given overlap.
        ///</summary>
        ///<param name="overlap">Overlap used to create a pair.</param>
        ///<returns>Narrow phase pair.</returns>
        public abstract INarrowPhasePair GetNarrowPhasePair(BroadPhaseOverlap overlap);

        /// <summary>
        /// Returns a pair to the factory for re-use.
        /// </summary>
        /// <param name="pair">Pair to return.</param>
        public abstract void GiveBack(INarrowPhasePair pair);
    }
}
