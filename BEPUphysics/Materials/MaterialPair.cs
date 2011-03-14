namespace BEPUphysics.Materials
{
    ///<summary>
    /// A pair of materials.
    ///</summary>
    public struct MaterialPair
    {
        ///<summary>
        /// First material in the pair.
        ///</summary>
        public Material MaterialA;
        ///<summary>
        /// Second material in the pair.
        ///</summary>
        public Material MaterialB;
        ///<summary>
        /// Constructs a new material pair.
        ///</summary>
        ///<param name="a">First material in the pair.</param>
        ///<param name="b">Second material in the pair.</param>
        public MaterialPair(Material a, Material b)
        {
            MaterialA = a;
            MaterialB = b;
        }

        /// <summary>
        /// Returns the hash code for this instance.
        /// </summary>
        /// <returns>
        /// A 32-bit signed integer that is the hash code for this instance.
        /// </returns>
        /// <filterpriority>2</filterpriority>
        public override int GetHashCode()
        {
            return MaterialA.GetHashCode() + MaterialB.GetHashCode();
        }
    }
}
