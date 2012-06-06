using System;
using System.Globalization;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Graphics;

namespace BEPUphysicsDrawer.Font
{
    /// <summary>
    /// Simple text-drawing helper class.
    /// </summary>
    public class TextDrawer
    {
        private static readonly string[] Digits = {"0", "1", "2", "3", "4", "5", "6", "7", "8", "9"};
        private const string Dot = ".";
        private static readonly string[] CharacterBuffer = new string[310];
        private static readonly float[] XPositionBuffer = new float[310];
        private static readonly string MinValue = Int32.MinValue.ToString(CultureInfo.InvariantCulture);

        /// <summary>
        /// Constructs a new text drawer.
        /// </summary>
        /// <param name="spriteBatch">Sprite batch to use.</param>
        /// <param name="font">Font to use.</param>
        /// <param name="color">Color to use.</param>
        public TextDrawer(SpriteBatch spriteBatch, SpriteFont font, Color color)
        {
            SpriteBatch = spriteBatch;
            Font = font;
            Color = color;
        }

        /// <summary>
        /// Gets the sprite batch used by the drawer.
        /// </summary>
        public SpriteBatch SpriteBatch { get; private set; }

        /// <summary>
        /// Gets or sets the font used by the drawer.
        /// </summary>
        public SpriteFont Font { get; set; }

        /// <summary>
        /// Gets or sets the color used by the drawer.
        /// </summary>
        public Color Color { get; set; }

        /// <summary>
        /// Draws the text at a position.
        /// </summary>
        /// <param name="text">Text to draw.</param>
        /// <param name="position">Position to draw the text at.</param>
        public void Draw(string text, Vector2 position)
        {
            SpriteBatch.DrawString(Font, text, position, Color);
        }

        /// <summary>
        /// Draws text followed by an integer.
        /// </summary>
        /// <param name="text">Text to draw.</param>
        /// <param name="value">Value to draw.</param>
        /// <param name="position">Position to draw the text at.</param>
        public void Draw(string text, int value, Vector2 position)
        {
            SpriteBatch.DrawString(Font, text, position, Color);
            Draw(value, new Vector2(position.X + Font.MeasureString(text).X, position.Y));
        }


        /// <summary>
        /// Draws text followed by a double.
        /// </summary>
        /// <param name="text">Text to draw.</param>
        /// <param name="value">Number to draw.</param>
        /// <param name="position">Position to draw the text at.</param>
        public void Draw(string text, double value, Vector2 position)
        {
            SpriteBatch.DrawString(Font, text, position, Color);
            Draw(value, new Vector2(position.X + Font.MeasureString(text).X, position.Y));
        }

        /// <summary>
        /// Draws text followed by a double.
        /// </summary>
        /// <param name="text">Text to draw.</param>
        /// <param name="value">Number to draw.</param>
        /// <param name="numPlaces">Number of digits to include after the decimal.</param>
        /// <param name="position">Position to draw the text at.</param>
        public void Draw(string text, double value, int numPlaces, Vector2 position)
        {
            SpriteBatch.DrawString(Font, text, position, Color);
            Draw(value, numPlaces, new Vector2(position.X + Font.MeasureString(text).X, position.Y));
        }

        //These last bits are based on some sample code posted by Stephen Styrchak (http://forums.xna.com/forums/p/45512/271953.aspx).

        /// <summary>  
        /// Draws an integer.
        /// </summary>  
        /// <param name="value">The integer value to draw.</param>  
        /// <param name="position">The screen position specifying where to draw the value.</param>  
        /// <returns>The next position on the line to draw text. This value uses position.Y and position.X plus the equivalent of calling spriteFont.MeasureString on value.ToString(CultureInfo.InvariantCulture).</returns>  
        public Vector2 Draw(int value, Vector2 position)
        {
            Vector2 nextPosition = position;

            if (value == Int32.MinValue)
            {
                nextPosition.X = nextPosition.X + Font.MeasureString(MinValue).X;
                SpriteBatch.DrawString(Font, MinValue, position, Color);
                position = nextPosition;
            }
            else
            {
                if (value < 0)
                {
                    nextPosition.X = nextPosition.X + Font.MeasureString("-").X;
                    SpriteBatch.DrawString(Font, "-", position, Color);
                    value = -value;
                    position = nextPosition;
                }

                int index = 0;

                do
                {
                    int modulus = value % 10;
                    value = value / 10;

                    CharacterBuffer[index] = Digits[modulus];
                    XPositionBuffer[index] = Font.MeasureString(Digits[modulus]).X;
                    index += 1;
                } while (value > 0);

                for (int i = index - 1; i >= 0; --i)
                {
                    nextPosition.X = nextPosition.X + XPositionBuffer[i] + Font.Spacing;
                    SpriteBatch.DrawString(Font, CharacterBuffer[i], position, Color);
                    position = nextPosition;
                }
            }
            return position;
        }

        /// <summary>  
        /// Draws a double.
        /// </summary>  
        /// <param name="value">The double value to draw.</param>  
        /// <param name="position">The screen position specifying where to draw the value.</param>  
        /// <returns>The next position on the line to draw text. This value uses position.Y and position.X plus the equivalent of calling spriteFont.MeasureString on value.ToString(CultureInfo.InvariantCulture).</returns>  
        public Vector2 Draw(double value, Vector2 position)
        {
            return Draw(value, 1, position);
        }

        /// <summary>  
        /// Draws a double.
        /// </summary>  
        /// <param name="value">The double value to draw.</param>  
        /// <param name="numPlaces">Maximum number of digits to include after the decimal.</param>
        /// <param name="position">The screen position specifying where to draw the value.</param>  
        /// <returns>The next position on the line to draw text. This value uses position.Y and position.X plus the equivalent of calling spriteFont.MeasureString on value.ToString(CultureInfo.InvariantCulture).</returns>  
        public Vector2 Draw(double value, int numPlaces, Vector2 position)
        {
            Vector2 nextPosition = position;

            if (value < 0)
            {
                nextPosition.X = nextPosition.X + Font.MeasureString("-").X;
                SpriteBatch.DrawString(Font, "-", position, Color);
                value = -value;
                position = nextPosition;
            }

            int index = 0;
            value = Math.Abs(value);

            double wholeValue = value;
            value = value % 1;
            if (value != 0)
            {
                value *= Math.Pow(10, numPlaces);

                do
                {
                    long modulus = (long) value % 10;
                    value = (long) value / 10;

                    CharacterBuffer[index] = Digits[modulus];
                    XPositionBuffer[index] = Font.MeasureString(Digits[modulus]).X;
                    index += 1;
                } while (value > 0 || index < numPlaces);

                CharacterBuffer[index] = Dot;
                XPositionBuffer[index] = Font.MeasureString(Dot).X;
                index += 1;
            }

            do
            {
                long modulus = (long) wholeValue % 10;
                wholeValue = (long) wholeValue / 10;

                CharacterBuffer[index] = Digits[modulus];
                XPositionBuffer[index] = Font.MeasureString(Digits[modulus]).X;
                index += 1;
            } while (wholeValue > 0);


            for (int i = index - 1; i >= 0; --i)
            {
                nextPosition.X = nextPosition.X + XPositionBuffer[i] + Font.Spacing;
                SpriteBatch.DrawString(Font, CharacterBuffer[i], position, Color);
                position = nextPosition;
            }

            return position;
        }
    }
}