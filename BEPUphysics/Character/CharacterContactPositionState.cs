using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace BEPUphysics.Character
{
    /// <summary>
    /// State of a contact relative to a speculative character position.
    /// </summary>
    public enum CharacterContactPositionState
    {
        Accepted,
        Rejected,
        TooDeep,
        Obstructed,
        HeadObstructed,
        NoHit
    }
}
