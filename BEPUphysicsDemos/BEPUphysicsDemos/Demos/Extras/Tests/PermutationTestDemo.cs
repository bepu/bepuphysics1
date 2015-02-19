using System;
using System.Collections.Generic;
using System.Text;
using BEPUphysics.Entities.Prefabs;
using BEPUphysicsDemos.Demos;
using BEPUutilities;
using Microsoft.Xna.Framework;

namespace BEPUphysicsDemos.Demos.Extras.Tests
{
    /// <summary>
    /// Demo showing a wall of blocks stacked up.
    /// </summary>
    public class PermutationTestDemo : StandardDemo
    {





        private struct Permutation : IEquatable<Permutation>
        {
            public int[] Set;

            public Permutation(int[] permutation)
            {
                Set = permutation;
            }



            public bool Equals(Permutation other)
            {
                if (other.Set.Length == Set.Length)
                {
                    for (int i = 0; i < Set.Length; ++i)
                    {
                        if (other.Set[i] != Set[i])
                            return false;
                    }
                    return true;
                }
                return false;
            }

            public override bool Equals(object obj)
            {
                return Equals((Permutation)obj);
            }

            public override int GetHashCode()
            {
                int hash = 0;
                for (int i = 0; i < Set.Length; ++i)
                {
                    hash += Set[i] * i;
                }
                return hash;
            }

            public override string ToString()
            {
                var builder = new StringBuilder(1 + 1 + (Set.Length - 1) * 3 + 1);
                builder.Append("{");
                if (Set.Length > 0)
                    builder.Append(Set[0]);
                for (int i = 1; i < Set.Length; ++i)
                {
                    builder.Append(", ");
                    builder.Append(Set[i]);
                }
                builder.Append("}");
                return builder.ToString();
            }

        }

        /// <summary>
        /// Constructs a new demo.
        /// </summary>
        /// <param name="game">Game owning this demo.</param>
        public PermutationTestDemo(DemosGame game)
            : base(game)
        {
            var permutations = new HashSet<Permutation>();
            var mapper = new PermutationMapper();

            var representedIndices = new HashSet<int>();
            mapper.PermutationIndex = 0;
            const int setSize = 250;
            const int iterationCount = 150000;
            var conflicts = 0;
            for (int i = 0; i < iterationCount; ++i)
            {
                var permutedIndices = new int[setSize];
                for (int setIndex = 0; setIndex < setSize; ++setIndex)
                {
                    permutedIndices[setIndex] = mapper.GetMappedIndex(setIndex, setSize);
                }

                for (int index = 0; index < permutedIndices.Length; ++index)
                {
                    if (!representedIndices.Add(permutedIndices[index]))
                        throw new Exception("Not a permutation!");
                }
                representedIndices.Clear();

                var permutation = new Permutation(permutedIndices);


                if (!permutations.Add(permutation))
                {
                    ++conflicts;
                }
                ++mapper.PermutationIndex;

                if (i % (iterationCount / 100) == 0)
                {
                    Console.WriteLine("{0}% complete", ((double) i * 100) / iterationCount);
                }
            }

            Console.WriteLine("Set size: {0}", setSize);
            Console.WriteLine("Iteration count: {0}", iterationCount);
            Console.WriteLine("Permutation count: {0}", permutations.Count);
            Console.WriteLine("Conflict count: {0}", conflicts);
        }

        /// <summary>
        /// Gets the name of the simulation.
        /// </summary>
        public override string Name
        {
            get { return "Not Much"; }
        }
    }
}