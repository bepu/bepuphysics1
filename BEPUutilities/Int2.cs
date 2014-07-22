using System;

namespace BEPUutilities
{
    /// <summary>
    /// Provides simple 2d cell hashing.
    /// </summary>
    public struct Int2 : IEquatable<Int2>
    {
        public int X;
        public int Y;

        public Int2(int x, int y)
        {
            X = x;
            Y = y;
        }

        public override bool Equals(object obj)
        {
            return Equals((Int2)obj);
        }

        public bool Equals(Int2 other)
        {
            return X == other.X && Y == other.Y;
        }

        public static bool operator ==(Int2 lhs, Int2 rhs)
        {
            return lhs.X == rhs.X && lhs.Y == rhs.Y;
        }

        public static bool operator !=(Int2 lhs, Int2 rhs)
        {
            return lhs.X != rhs.X || lhs.Y != rhs.Y;
        }

        public override int GetHashCode()
        {
            return (X * 533000401) ^ (Y * 920419813);
        }

        public override string ToString()
        {
            return "{" + X + ", " + Y + "}";
        }


    }
}
