namespace BEPUphysics.Materials
{
    /// <summary>
    /// Contains the various options available for combining the physical properties of objects during an interaction, such as friction and bounciness.
    /// </summary>
    public enum PropertyBlendMethod
    {
        /// <summary>
        /// Averages the two property values.
        /// </summary>
        Average,
        /// <summary>
        /// Uses the larger of the two property values.
        /// </summary>
        Max,
        /// <summary>
        /// Uses the smaller of the two property values.
        /// </summary>
        Min,
        /// <summary>
        /// Factors in 75% of the high property value of the pair and 25% of the low property value of the pair.
        /// </summary>
        BiasHigh,
        /// <summary>
        /// Factors in 25% of the high property value of the pair and 75% of the low property value of the pair.
        /// </summary>
        BiasLow,
    } ;

    /// <summary>
    /// Function which takes information from each entity and computes a blended property.
    /// Commonly used for friction and bounciness.
    /// </summary>
    /// <param name="aValue">Value associated with the first object to blend.</param>
    /// <param name="bValue">Value associated with the second object to blend.</param>
    /// <param name="extraData">Extra data to include in the process.</param>
    /// <returns>Blended property value.</returns>
    public delegate float PropertyBlender(float aValue, float bValue, object extraData);


}