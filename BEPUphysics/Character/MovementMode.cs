using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace BEPUphysics.Character
{
    /// <summary>
    /// Defines different movement states associated with the character.
    /// </summary>
    public enum MovementMode
    {
        /// <summary>
        /// The character has controlled contact with a support object. The character can move normally.
        /// </summary>
        Traction,
        /// <summary>
        /// The character has uncontrolled contact with a support object. The character should have limited control over its movement.
        /// Imagine sliding down a steep slope.
        /// </summary>
        Sliding,
        /// <summary>
        /// The character has no support.
        /// </summary>
        Floating
    }
}
